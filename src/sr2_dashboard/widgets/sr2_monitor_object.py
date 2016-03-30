# Author: Aleksandar Vladimirov Atanasov
# Description: Infrastructure for executing external processes, controlling and monitoring these

from os import kill, mkdir, remove, rmdir, stat, error, path, makedirs, umask, chmod
from exceptions import IOError
from os.path import exists, isdir
from shutil import rmtree
from signal import SIGINT
import errno
# ROS
import rospy
# PyQt, RQt related
from python_qt_binding.QtCore import QObject, pyqtSignal, pyqtSlot, QProcess, QThread

class ProcStatus():
  '''
  Represents the status of the external ROS process
  '''
  INACTIVE      = 0      # The process hasn't been launched yet or has exited properly
  RUNNING       = 1      # The process has started successfully
  FAILED_START  = 2      # The process has failed to start
  FINISHED      = 3      # The process has exited succesfully (exit code == 0)
  FAILED_STOP   = 4      # The process has failed to exit properly (exit code != 0) or the clean up of the files associated with the process has failed

# Note: Errors will remain visible until use doesn't confirm the error (menu entry or view button will remain toggled until then)

class SR2Worker_v2(QObject):
  # TODO
  pass

class SR2Worker(QObject):
  '''
  Launches a bash script as detached process which spawns a given ROS process and also receives the process' exit code
  In addition it allows control (start/stop) of the external ROS process and monitoring its status (inactive, finished, running etc.)
  It can be used by a menu entry without view and by view's components
  '''

  block_signal = pyqtSignal(bool)
  status_signal = pyqtSignal(int)
  recover_signal = pyqtSignal(int) # Emitted if PID file is present and process is running; this signal is used ONLY when starting the application and recovering the state of the UI

  def __init__(self, cmd, pkg, args=None):
    super(SR2Worker, self).__init__()

    if not cmd: return
    if not cmd and not pkg and not args: return

    self.cmd = cmd
    if args:
      self.args = args.split() # Split args-string on every whitespace and create args-list of strings
      if pkg:
        self.args = [pkg] + self.args
    else: self.args = []


    self.proc_status = ProcStatus.INACTIVE
    # Use None as default value for the PID otherwise you are in for a BIG surprise if you run kill -SIGINT with the incorrect PID)
    # Invoking kill with None as the PID argument results in exception which is much easier and better to handle
    self.proc_pid = None  # Stores PID of command process

    # Setup files
    self.dir_name = '/tmp'
    self.proc_path = self.dir_name + '/pid_' + cmd + ((' ' + pkg) if pkg else '') + ((' ' + args) if args == [] else '')
    self.proc_path = self.proc_path.replace('=','').replace('.','').replace(' ','').replace(':','')

    self.checkFileExists(self.proc_path)

  def checkFileExists(self, _path):
    '''
    Checks if given file exists (not directory!) and creates one if it doesn't
    :param path: path to file that needs to be checked
    :return False if file was nonexistent
    '''
    try:
      if not path.isfile(_path):
        open(_path, 'w').close()
        rospy.loginfo('SR2: File %s was successfully created', _path)
        return False
      else: return True
    except IOError:
      rospy.logerr('SR2: Unable to write PID file "%s". Maybe you don\'t have the permissions to write to the given path?', _path)
      print(_path)
      return False

  def writePidToFile(self, _path, source):
    '''
    Writes PID to file. It invokes a check whether file exists and whether it's empty. If either of these preconditions fail, no data is written to file
    :param path: path of PID file
    :param source: PID
    '''
    rospy.loginfo('SR2: Attempting to write %s to %s' % (str(source), _path))
    if not source:
      return
    # Writing to a non-existent file automatically creates a new one however we also want to check if it's empty or not!
    if self.checkFileExists(_path) and self.isFileEmpty(_path):
      rospy.loginfo('SR2: File exists and is not empty. Will write to file')
      with open(_path, 'w') as pidFile:
        pidFile.seek(0)
        try:
          pidFile.write(str(source))
        except IOError:
          rospy.logerr('SR2: Failed to write to file')
      rospy.loginfo('SR2: Writing to file completed')

  def isFileEmpty(self, _path):
    '''
    Checks if a given file is empty
    :param path: path to file that needs to be checked
    '''
    # Use of exception handling here is necessary because if file doesn't exist the following won't work
    try: return stat(_path).st_size == 0
    except error: return False

  def loadPid(self, _path):
    '''
    Returns a PID stored in a valid file _path; else returns None
    :param _path: path to an existing and not empty file
    '''
    if self.checkFileExists(_path) and not self.isFileEmpty(_path):
      with open(_path, 'r') as pidFile:
        return int(pidFile.readline())
    else: return None

  def checkPid(self, pid):
    '''
    Checks if a process with a given PID is running or not
    :param pid: PID of process that needs to be checked
    '''
    if not pid: return False
    try:
      kill(pid, 0)
    except OSError:
      return False
    else:
      return True

  def checkProcessRunning(self):
    '''
    Invokes checkPid() in order to check whether the process is running or not including cleanup afterwards
    '''
    proc_running = self.checkPid(self.proc_pid)

    # A running external process here is considered to be a tuple of the running process itself and the bash that has spawned
    # If this requirement is not fulfilled we kill the one that is running
    if not proc_running:
      if self.proc_pid:
        try:
          kill(self.proc_pid, SIGINT)
        except:
          self.cleanup()
          self.proc_status = ProcStatus.FAILED_START
          return False
      return False

    return True

  def cleanup(self):
    '''
    Removes all files related to the external process: .proc, .bash and root directory containing these two files
    '''
    try:
      remove(self.proc_path)
    except:
      if self.proc_pid:
        self.proc_status = ProcStatus.FAILED_STOP
      self.status_signal.emit(self.proc_status)

  @pyqtSlot()
  def checkRecoveryState(self):
    '''
    Checks if PID for given process can be loaded from PID file
    Based on the success a recovery state is triggered in the owner of this object
    '''
    self.proc_pid = self.loadPid(self.proc_path)
    if self.proc_pid and self.checkProcessRunning(): self.proc_status = ProcStatus.RUNNING
    self.recover_signal.emit(self.proc_status) # Recover the button that started this process to its toggled states

  @pyqtSlot()
  def start(self):
    '''
    Starts a bash script as detached process. The script will then spawn the given ROS process as a child
    process and receive its exit code once it stops. The result will be written inside the PID file .proc
    '''
    self.block_signal.emit(True) # "Disable" the UI component that controls the worker
    rospy.loginfo('Blocking UI component')
    # If we have restored the UI or triggered the start slot again while the PID of the bash process and external process are present we can skip the creation of the external processes
    if self.proc_pid:
      rospy.loginfo('SR2: Restoring state of UI component')
      self.proc_status = ProcStatus.RUNNING
      self.status_signal.emit(self.proc_status)
      return

    rospy.loginfo('SR2: Attempting to launch ROS process')
    # Start the detached process with the bash script as the command and the rest as the arguments for the script
    # The working directory argument (here '/tmp') HAS to be present otherwise no PID will be returned (see Qt documentation)

    print('**************************************** CMD: %s | ARGS: %s' % (self.cmd, str(self.args)))
    self.proc_status, self.proc_pid = QProcess.startDetached(self.cmd, self.args, self.dir_name)
    QThread.sleep(10)
    print('**************************************** STATUS: %s | PID: %d' % (('True' if self.proc_status else 'False'), self.proc_pid))

    # Check if process has started properly
    if self.proc_status and self.proc_pid:
      self.writePidToFile(self.proc_path, self.proc_pid)
      rospy.loginfo('SR2: ROS process successfully started')
      self.proc_status = ProcStatus.RUNNING
      self.proc_pid = self.loadPid(self.proc_path)
    else: self.proc_status = ProcStatus.FAILED_START

    self.status_signal.emit(self.proc_status)
    rospy.loginfo('Unblocking UI component')
    self.block_signal.emit(False)

  @pyqtSlot()
  def stop(self):
    '''
    Emit signal based on final state of process, kill process based on its PID
    and cleanup PID file used while the process was alive
    '''
    rospy.loginfo('SR2: Attempting to stop ROS process')
    self.block_signal.emit(True)
    rospy.loginfo('Blocking UI component')
    # Check if both the bash script and the ROS process are running
    if not self.checkProcessRunning():
      rospy.logwarn('SR2: External process not running so there is nothing to stop')
      self.proc_status = ProcStatus.INACTIVE
      self.status_signal.emit(self.proc_status)
      return

#    if not self.bash_pid or not self.proc_pid: return

    # Make sure that directories exist and both PID files are there (in case between init of the worker object and calling the start slot these have been deleted)
    # We need this here just to avoid exception handling
    self.checkFileExists(self.proc_path)

    # First send SIGINT to external ROS process
    if self.proc_pid: kill(self.proc_pid, SIGINT)
#    QThread.sleep(2)
    # Reset PIDs
    self.proc_pid = None

    # After the ROS process has been stopped the bash script that has spawned it will receive its exit code
    # and write it to the .proc file
    # Check if exit code of ROS process was okay or not
    if self.isFileEmpty(self.proc_path):
      rospy.logwarn('SR2: ROS process has stopped prematurely')
      self.proc_status = ProcStatus.FAILED_STOP
      return

    self.proc_status = ProcStatus.FINISHED

    # Clean up files and directory
    self.cleanup()

    self.status_signal.emit(self.proc_status)
    rospy.loginfo('Unblocking UI component')
    self.block_signal.emit(False)

  @pyqtSlot()
  def status(self):
    '''
    An external timer triggers this slot at a specific interval
    The slot checks if the external ROS process is still running and reports back by emitting a status_signal
    '''

    # In case the ROS process has finished, is inactive or failed we don't change the status
    running = self.checkProcessRunning()

    if running:
      self.proc_status = ProcStatus.RUNNING
    else:
      if self.proc_pid and self.proc_status == ProcStatus.RUNNING:
        self.stop()

    #if self.proc_pid and self.proc_status == ProcStatus.RUNNING: self.stop()

#    print('PROC_STATUS =', self.proc_status)
    self.status_signal.emit(self.proc_status)

