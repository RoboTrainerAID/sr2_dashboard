# Author: Aleksandar Vladimirov Atanasov
# Description: Infrastructure for executing external processes, controlling and monitoring these

from os import kill, remove, stat, error, path
from exceptions import IOError
from signal import SIGINT
# ROS
import rospy
# PyQt, RQt related
from python_qt_binding.QtCore import QObject, pyqtSignal, pyqtSlot, QProcess, QThread, pyqtProperty

class ProcStatus():
  '''
  Represents the status of the external ROS process
  '''
  INACTIVE      = 0      # The process hasn't been launched yet or has exited properly
  RUNNING       = 1      # The process has started successfully
  FAILED_START  = 2      # The process has failed to start
  FINISHED      = 3      # The process has exited succesfully (exit code == 0)
  FAILED_STOP   = 4      # The process has failed to exit properly (exit code != 0) or the clean up of the files associated with the process has failed

class SR2Worker(QObject):
  recover_signal = pyqtSignal()
  statusChanged_signal = pyqtSignal(int)
  block_signal = pyqtSignal(bool)

  def __init__(self, cmd, pkg, args=None):
    super(SR2Worker, self).__init__()

    if not cmd: return
    if not cmd and not pkg and not args: return

    self.cmd = cmd
    if args: self.args = args.split()
    else: self.args = []

    if pkg: self.args = [pkg] + self.args

    self._status = -1
    self.active = False
    self.pid = None

    self.dir_name = '/tmp'
    self.path = self.dir_name + '/pid_' + cmd + (pkg if pkg else '') + (args if args else '')
    self.path = self.path.replace('=','').replace('.','').replace(' ','').replace(':','').replace('~','')
  
  def __del__(self):
    if self._status != ProcStatus.RUNNING:
      rospy.loginfo('SR2: External process not running. Will delete reserved (empty) PID file')
      self.cleanup()

  @pyqtSlot()
  def recover(self):
    '''
    At the beginning the worker attemps to load a PID that was previously stored in the PID file for the given command (and arguments if present)
    If the attempt succeeds, the UI receives a RUNNING status and the worker regains control over the external process
    '''
    # Attempt recovery
    self.pid = self.loadPid()
    if self.pid:
      rospy.logwarn('SR2: Process with PID %d already running. Will connect to it', self.pid)
      self.active = True
      self.recover_signal.emit()
      self.setStatus(ProcStatus.RUNNING)
    else: rospy.loginfo('SR2: Unable to recover previous state')

  @pyqtSlot()
  def start(self):
    self.block_signal.emit(True)
    rospy.loginfo('SR2: Attempting to start ')
    if not self.active and not self.pid:
      self.active, self.pid = QProcess.startDetached(self.cmd, self.args, self.dir_name)
      QThread.sleep(5)

      # Check if launching the external process was successful
      if not self.active or not self.pid:
        self.setStatus(ProcStatus.FAILED_START)
        self.block_signal(False)
        self.cleanup()
        return

      self.writePidToFile()
      rospy.loginfo('SR2: Process started with PID %d', self.pid)
      self.setStatus(ProcStatus.RUNNING)

    self.block_signal.emit(False)

  @pyqtSlot()
  def stop(self):
    self.block_signal.emit(True)
    if self.active and self.pid:
        try:
          kill(self.pid, SIGINT)
          QThread.sleep(5)
        except OSError:
          self.setStatus(ProcStatus.FAILED_STOP)
          rospy.logerr('SR2: Sending SIGINT to given PID %d failed', self.pid)

    self.cleanup()
    self.active = False
    self.pid = None
    self.setStatus(ProcStatus.FINISHED)
    self.block_signal.emit(False)
    print('STOP END')

  @pyqtSlot()
  def status(self):
#    print(' --- worker thread ID: %d ---' % QThread.currentThreadId())
    if self.active and self.pid:
      running = self.checkProcessRunning(self.pid)
      if not running:
        # This case occurs if the started external process has crashed or was stopped with other mean outside the dashboard
        self.setStatus(ProcStatus.FAILED_STOP)
        self.cleanup()
        self.active = False
        self.pid = None

  @pyqtSlot()
  def clear_error(self):
    self.setStatus(ProcStatus.INACTIVE)
    self.active = False
    self.pid = None

  def getStatus(self):
    return self._status

  def setStatus(self, status):
    if self._status == status: return
    self._status = status
    self.statusChanged_signal.emit(self._status)

  statusProp = pyqtProperty(int, fget=getStatus, fset=setStatus, notify=statusChanged_signal)

  def checkPid(self, pid):
    '''
    Checks if a process with a given PID is running or not
    :param pid: PID of process that needs to be checked
    '''
    if not pid: return False

    try: kill(pid, 0)
    except OSError: return False
    else: return True

  def checkPidFileExists(self):
    '''
    Checks if given file exists (not directory!) and creates one if it doesn't
    :param path: path to file that needs to be checked
    :return False if file was nonexistent
    '''
    try:
      if not path.isfile(self.path):
        open(self.path, 'w').close()
        rospy.loginfo('SR2: File %s was successfully created', self.path)
        return False
      else: return True
    except IOError:
      rospy.logerr('SR2: Unable to write PID file "%s". Maybe you don\'t have the permissions to write to the given path?', self.path)
      print(self.path)
      return False

  def writePidToFile(self):
    '''
    Writes PID to file. It invokes a check whether file exists and whether it's empty. If either of these preconditions fail, no data is written to file
    :param path: path of PID file
    :param source: PID
    '''
    rospy.loginfo('SR2: Attempting to write %s to %s' % (str(self.pid), self.path))
    if not self.pid: return
    # Writing to a non-existent file automatically creates a new one however we also want to check if it's empty or not!
    if self.checkPidFileExists() and self.isPidFileEmpty():
      rospy.loginfo('SR2: File exists and is not empty. Will write to file')
      with open(self.path, 'w') as pidFile:
        pidFile.seek(0)
        try: pidFile.write(str(self.pid))
        except IOError: rospy.logerr('SR2: Failed to write to file')
      rospy.loginfo('SR2: Writing to file completed')

  def isPidFileEmpty(self):
    '''
    Checks if a given file is empty
    :param path: path to file that needs to be checked
    '''
    # Use of exception handling here is necessary because if file doesn't exist the following won't work
    try: return stat(self.path).st_size == 0
    except error: return False

  def loadPid(self):
    '''
    Returns a PID stored in a valid file _path; else returns None
    '''
    if self.checkPidFileExists() and not self.isPidFileEmpty():
      with open(self.path, 'r') as pidFile:
        pid = int(pidFile.readline())
        if not pid: return None
        else:
          pid_running = self.checkPid(pid)
          if not pid_running:
            # Delete PID file if the contained PID is invalid (represents a process that isn't currently running)
            remove(self.path)
            return None
          else: return pid
    else: return None

  def checkProcessRunning(self, pid):
    '''
    Invokes checkPid() in order to check whether the process is running or not including cleanup afterwards
    '''
    running = self.checkPid(pid)

    # A running external process here is considered to be a tuple of the running process itself and the bash that has spawned
    # If this requirement is not fulfilled we kill the one that is running
    if not running:
      if pid:
#        self.cleanup()
#        self.setStatus(ProcStatus.FAILED_START)
        return False
      return False
    return True

  def cleanup(self):
    '''
    Removes all files related to the external process: .proc, .bash and root directory containing these two files
    '''
    try:
      if self.checkPidFileExists(): remove(self.path)
      else: rospy.loginfo('SR2: No PID file found. Nothing to cleanup')
    except: rospy.logerr('SR2: Failed to delete PID file')

##########################################################################################

#class SR2Worker(QObject):
#  '''
#  Launches a bash script as detached process which spawns a given ROS process and also receives the process' exit code
#  In addition it allows control (start/stop) of the external ROS process and monitoring its status (inactive, finished, running etc.)
#  It can be used by a menu entry without view and by view's components
#  '''
#
#  block_signal = pyqtSignal(bool)
#  status_signal = pyqtSignal(int)
#  recover_signal = pyqtSignal(int) # Emitted if PID file is present and process is running; this signal is used ONLY when starting the application and recovering the state of the UI
#
#  def __init__(self, cmd, pkg, args=None):
#    super(SR2Worker, self).__init__()
#
#    if not cmd: return
#    if not cmd and not pkg and not args: return
#
#    self.cmd = cmd
#    if args:
#      self.args = args.split() # Split args-string on every whitespace and create args-list of strings
#      if pkg:
#        self.args = [pkg] + self.args
#    else: self.args = []
#
#
#    self.proc_status = ProcStatus.INACTIVE
#    # Use None as default value for the PID otherwise you are in for a BIG surprise if you run kill -SIGINT with the incorrect PID)
#    # Invoking kill with None as the PID argument results in exception which is much easier and better to handle
#    self.proc_pid = None  # Stores PID of command process
#
#    # Setup files
#    self.dir_name = '/tmp'
#    self.proc_path = self.dir_name + '/pid_' + cmd + ((' ' + pkg) if pkg else '') + ((' ' + args) if args == [] else '')
#    self.proc_path = self.proc_path.replace('=','').replace('.','').replace(' ','').replace(':','')
#
#    self.checkFileExists(self.proc_path)
#
#  def checkFileExists(self, _path):
#    '''
#    Checks if given file exists (not directory!) and creates one if it doesn't
#    :param path: path to file that needs to be checked
#    :return False if file was nonexistent
#    '''
#    try:
#      if not path.isfile(_path):
#        open(_path, 'w').close()
#        rospy.loginfo('SR2: File %s was successfully created', _path)
#        return False
#      else: return True
#    except IOError:
#      rospy.logerr('SR2: Unable to write PID file "%s". Maybe you don\'t have the permissions to write to the given path?', _path)
#      print(_path)
#      return False
#
#  def writePidToFile(self, _path, source):
#    '''
#    Writes PID to file. It invokes a check whether file exists and whether it's empty. If either of these preconditions fail, no data is written to file
#    :param path: path of PID file
#    :param source: PID
#    '''
#    rospy.loginfo('SR2: Attempting to write %s to %s' % (str(source), _path))
#    if not source:
#      return
#    # Writing to a non-existent file automatically creates a new one however we also want to check if it's empty or not!
#    if self.checkFileExists(_path) and self.isFileEmpty(_path):
#      rospy.loginfo('SR2: File exists and is not empty. Will write to file')
#      with open(_path, 'w') as pidFile:
#        pidFile.seek(0)
#        try:
#          pidFile.write(str(source))
#        except IOError:
#          rospy.logerr('SR2: Failed to write to file')
#      rospy.loginfo('SR2: Writing to file completed')
#
#  def isFileEmpty(self, _path):
#    '''
#    Checks if a given file is empty
#    :param path: path to file that needs to be checked
#    '''
#    # Use of exception handling here is necessary because if file doesn't exist the following won't work
#    try: return stat(_path).st_size == 0
#    except error: return False
#
#  def loadPid(self, _path):
#    '''
#    Returns a PID stored in a valid file _path; else returns None
#    :param _path: path to an existing and not empty file
#    '''
#    if self.checkFileExists(_path) and not self.isFileEmpty(_path):
#      with open(_path, 'r') as pidFile:
#        return int(pidFile.readline())
#    else: return None
#
#  def checkPid(self, pid):
#    '''
#    Checks if a process with a given PID is running or not
#    :param pid: PID of process that needs to be checked
#    '''
#    if not pid: return False
#    try:
#      kill(pid, 0)
#    except OSError:
#      return False
#    else:
#      return True
#
#  def checkProcessRunning(self):
#    '''
#    Invokes checkPid() in order to check whether the process is running or not including cleanup afterwards
#    '''
#    proc_running = self.checkPid(self.proc_pid)
#
#    # A running external process here is considered to be a tuple of the running process itself and the bash that has spawned
#    # If this requirement is not fulfilled we kill the one that is running
#    if not proc_running:
#      if self.proc_pid:
#        try:
#          kill(self.proc_pid, SIGINT)
#        except:
#          self.cleanup()
#          self.proc_status = ProcStatus.FAILED_START
#          return False
#      return False
#
#    return True
#
#  def cleanup(self):
#    '''
#    Removes all files related to the external process: .proc, .bash and root directory containing these two files
#    '''
#    try:
#      remove(self.proc_path)
#    except:
#      if self.proc_pid:
#        self.proc_status = ProcStatus.FAILED_STOP
#      self.status_signal.emit(self.proc_status)
#
#  @pyqtSlot()
#  def checkRecoveryState(self):
#    '''
#    Checks if PID for given process can be loaded from PID file
#    Based on the success a recovery state is triggered in the owner of this object
#    '''
#    self.proc_pid = self.loadPid(self.proc_path)
#    if self.proc_pid and self.checkProcessRunning(): self.proc_status = ProcStatus.RUNNING
#    self.recover_signal.emit(self.proc_status) # Recover the button that started this process to its toggled states
#
#  @pyqtSlot()
#  def start(self):
#    '''
#    Starts a bash script as detached process. The script will then spawn the given ROS process as a child
#    process and receive its exit code once it stops. The result will be written inside the PID file .proc
#    '''
#    self.block_signal.emit(True) # "Disable" the UI component that controls the worker
#    rospy.loginfo('Blocking UI component')
#    # If we have restored the UI or triggered the start slot again while the PID of the bash process and external process are present we can skip the creation of the external processes
#    if self.proc_pid:
#      rospy.loginfo('SR2: Restoring state of UI component')
#      self.proc_status = ProcStatus.RUNNING
#      self.status_signal.emit(self.proc_status)
#      return
#
#    rospy.loginfo('SR2: Attempting to launch ROS process')
#    # Start the detached process with the bash script as the command and the rest as the arguments for the script
#    # The working directory argument (here '/tmp') HAS to be present otherwise no PID will be returned (see Qt documentation)
#
#    print('**************************************** CMD: %s | ARGS: %s' % (self.cmd, str(self.args)))
#    self.proc_status, self.proc_pid = QProcess.startDetached(self.cmd, self.args, self.dir_name)
#    QThread.sleep(10)
#    print('**************************************** STATUS: %s | PID: %d' % (('True' if self.proc_status else 'False'), self.proc_pid))
#
#    # Check if process has started properly
#    if self.proc_status and self.proc_pid:
#      self.writePidToFile(self.proc_path, self.proc_pid)
#      rospy.loginfo('SR2: ROS process successfully started')
#      self.proc_status = ProcStatus.RUNNING
#      self.proc_pid = self.loadPid(self.proc_path)
#    else: self.proc_status = ProcStatus.FAILED_START
#
#    self.status_signal.emit(self.proc_status)
#    rospy.loginfo('Unblocking UI component')
#    self.block_signal.emit(False)
#
#  @pyqtSlot()
#  def stop(self):
#    '''
#    Emit signal based on final state of process, kill process based on its PID
#    and cleanup PID file used while the process was alive
#    '''
#    rospy.loginfo('SR2: Attempting to stop ROS process')
#    self.block_signal.emit(True)
#    rospy.loginfo('Blocking UI component')
#    # Check if both the bash script and the ROS process are running
#    if not self.checkProcessRunning():
#      rospy.logwarn('SR2: External process not running so there is nothing to stop')
#      self.proc_status = ProcStatus.INACTIVE
#      self.status_signal.emit(self.proc_status)
#      return
#
##    if not self.bash_pid or not self.proc_pid: return
#
#    # Make sure that directories exist and both PID files are there (in case between init of the worker object and calling the start slot these have been deleted)
#    # We need this here just to avoid exception handling
#    self.checkFileExists(self.proc_path)
#
#    # First send SIGINT to external ROS process
#    if self.proc_pid: kill(self.proc_pid, SIGINT)
##    QThread.sleep(2)
#    # Reset PIDs
#    self.proc_pid = None
#
#    # After the ROS process has been stopped the bash script that has spawned it will receive its exit code
#    # and write it to the .proc file
#    # Check if exit code of ROS process was okay or not
#    if self.isFileEmpty(self.proc_path):
#      rospy.logwarn('SR2: ROS process has stopped prematurely')
#      self.proc_status = ProcStatus.FAILED_STOP
#      return
#
#    self.proc_status = ProcStatus.FINISHED
#
#    # Clean up files and directory
#    self.cleanup()
#
#    self.status_signal.emit(self.proc_status)
#    rospy.loginfo('Unblocking UI component')
#    self.block_signal.emit(False)
#
#  @pyqtSlot()
#  def status(self):
#    '''
#    An external timer triggers this slot at a specific interval
#    The slot checks if the external ROS process is still running and reports back by emitting a status_signal
#    '''
#
#    # In case the ROS process has finished, is inactive or failed we don't change the status
#    running = self.checkProcessRunning()
#
#    if running:
#      self.proc_status = ProcStatus.RUNNING
#    else:
#      if self.proc_pid and self.proc_status == ProcStatus.RUNNING:
#        self.stop()
#
#    #if self.proc_pid and self.proc_status == ProcStatus.RUNNING: self.stop()
#
##    print('PROC_STATUS =', self.proc_status)
#    self.status_signal.emit(self.proc_status)
#
