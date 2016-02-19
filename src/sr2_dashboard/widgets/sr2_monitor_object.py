# General
from os import kill, mkdir, remove, rmdir, stat, error, path
from exceptions import IOError
from os.path import exists
from shutil import rmtree
from signal import SIGINT
import errno
# ROS
import rospy
from rospkg import RosPack
# PyQt, RQt related
from python_qt_binding.QtCore import QObject, pyqtSignal, pyqtSlot, QProcess

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

class SR2Worker(QObject):
  '''
  Launches a bash script as detached process which spawns a given ROS process and also receives the process' exit code
  In addition it allows control (start/stop) of the external ROS process and monitoring its status (inactive, finished, running etc.)
  It can be used by a menu entry without view and by view's components
  '''

  statusSignal = pyqtSignal(int)

  @staticmethod
  def checkPid(pid):
    '''
    Checks if a process with a given PID is running or not
    '''
    try:
      kill(pid, 0)
    except OSError as ose:
      return ose.errno == errno.EPERM
    except TypeError:
      return False
    else:
        return True
#    try:
#      if pid: kill(pid, 0)
#      return True
#    except OSError:
#      return False

  def __init__(self, command, pkg, args=None):
    super(SR2Worker, self).__init__()

    if not command or not pkg or not args: return

    self.bash_status = False
    self.proc_status = ProcStatus.INACTIVE
    # Use None as default value for the PID otherwise you are in for a BIG surprise if you run kill -SIGINT with the incorrect PID)
    # Invoking kill with None as the PID argument results in exception which is much easier and better to handle
    self.bash_pid = None # Stores PID of bash process
    self.proc_pid = None  # Stores PID of command process
    self.exit_code = -1 # Stores exit code from command process

    # Setup files
    self.dir_name = '/tmp/' + command + pkg + args
    self.dir_name = self.dir_name.strip() # Remove all spaces so that the bash script doesn't get confused
    self.bash_path = self.dir_name + '/.bash'
    self.proc_path = self.dir_name + '/.proc'

    # Get path to BASH script
    rpkg = RosPack()
    pkg_name = 'sr2_dashboard'
    pkg_path = rpkg.get_path(pkg_name)
    bash_script_path = pkg_path + '/scripts/get_exit_code.sh'
    # The script is located in "<SR2_DASHBOARD_ROOT>/scripts/"
    # QProcess.startDetached(COMMAND, ARGS_LIST, WORK_DIR)
    self.command = bash_script_path
    # The bash script requires the first argument to be the directory (NO SPACES allowed in the name of the direcotry)
    # where the process PID and its exit code will be stored
    self.args = [self.dir_name, command, pkg, args]

    self.checkRootDirExists()
    #self.checkBashProcFilesExist()
    self.checkFileExists(self.bash_path)
    self.checkFileExists(self.proc_path)

  def checkRootDirExists(self):
    '''
    Check is the process folder is present and creates it if not
    '''
    if not exists(self.dir_name):
      rospy.loginfo('SR2: Process directory doesn\'t exist and therefore will be created')
      mkdir(self.dir_name)

  def checkFileExists(self, _path):
    '''
    Checks if given file exists (not directory!) and creates one if it doesn't
    :param path: path to file that needs to be checked
    :return False if file was nonexistent
    '''
    try:
      if not path.isfile(_path):
        open(_path, 'w').close()
        return False
      else: return True
    except IOError as ioe:
      rospy.logerr('SR2: Unable to write PID file "%s". Maybe you don\'t have the permissions to write to the given path?', _path)
      return False

  def writePidToFile(self, _path, source):
    '''
    Writes PID to file. It invokes a check whether file exists and whether it's empty. If either of these preconditions fail, no data is written to file
    :param path: path of PID file
    :param source: PID
    '''
    # TODO Check if source is int?
    if self.checkFileExists(_path) and not self.isFileEmpty(_path):
      if source:
        with open(_path, 'w') as pidFile:
          pidFile.write(str(source))

#  def checkBashProcFilesExist(self):
#    '''
#    Creates the .proc and .bash files if they don't exists. If they exist the information stored in them is loaded
#    Note that the bash script that retrieves the exit code from the external process is the one that actually creates the files
#    The creation here can only happen if a PID for either .bash or .proc files is available (>0) which happens if the UI is closed but the two processes are not shutdown
#    Warning: call this function after running checkAndCreateBashProcRootDir() to ensure that process folder is present
#    '''
#    pass
    # The default PID that the bash and external ROS process have is -1
    # The bash script is the one that creates and writes the data (PID and exit code) to the .proc file, which is then loaded in self.pid_proc and self.exit_code
    # In this case we return without doing nothing

    # .bash
#    try:
#      if not path.isfile(self.bash_path):
#        if self.bash_pid > 0:
#          # Here we make sure that the .bash file is present during the execution of the application
#          # It is useful only when the .bash file is deleted not by the application itself
#          with open(self.bash_path, 'w') as pidBashFile:
#            pidBashFile.write(str(self.bash_pid))
#        else: open(self.bash_path, 'w').close() # Create empty file
#      else:
#        # If the .bash file is present then we try to load the information stored in it
#        if not self.isFileEmpty(self.bash_path):
#          # Load only in case the bash PID isn't already stored in the application
#          if self.bash_pid < 0:
#            with open(self.bash_path, 'r') as pidBashFile:
#              # The single line inside .bash contains the PID of the running bash script
#              self.bash_pid = int(pidBashFile.readline())
#          else:
#            # We also use checkBashProcFilesExist() to write the PID to the file
#            with open(self.bash_path, 'w') as pidBashFile:
#              pidBashFile.write(str(self.bash_pid))

      # .proc
#      if not path.isfile(self.proc_path):
#        if self.proc_pid > 0:
#          # This ensures that the .bash file is present during the execution of the application
#          # It is useful only when the .bash file is deleted not by the application itself
#          with open(self.proc_path, 'w') as pidProcFile:
#            pidProcFile.write(str(self.proc_pid))
#        else: open(self.proc_path, 'w').close() # Create empty file
#      else:
#        # If the .proc file is present then we try to load the information stored in it (PID only)
#        if not self.isFileEmpty(self.proc_path):
#          with open(self.proc_path, 'r') as pidProcFile:
#            # The first line inside .proc contains the PID of the running ROS process spawned by the bash script while the second (not needed here) is its exit code
#            self.proc_pid = int(pidProcFile.readline())
#    except:
#      rospy.logerr('SR2: Something happend while trying to read/write from/to the PID files')

  def isFileEmpty(self, _path):
    '''
    Checks if a given file is empty
    :param path: path to file that needs to be checked
    '''
    # Use of exception handling here is necessary because if file doesn't exist the following won't work
    try: return stat(_path).st_size == 0
    except error: return False

  def loadPid(self, _path, target):
    if self.checkFileExists(_path) and not self.isFileEmpty(_path):
      with open(_path, 'r') as pidFile:
        target = int(pidFile)

  def checkProcessRunning(self):
    '''
    Using self.checkPid() it checks whether the process is running or not
    '''
    # Call ps with the PID of the external process
#      self.monitoring_proc.start('ps', ['-p', str(self.pid)])
    # Wait for ps to finish
#      if self.monitoring_proc.waitForFinished():
      # If the exit status of ps is 0 then it has discovered the external process so True is returned
#        if not self.monitoring_proc.exitCode() and not self.monitoring_proc.exitStatus(): return True

    bash_runnning = self.checkPid(self.bash_pid)
    proc_running = self.checkPid(self.proc_pid)

    # A running external process here is considered to be a tuple of the running process itself and the bash that has spawned
    # If this requirement is not fulfilled we kill the one that is running
    if not bash_runnning or not proc_running:
      if self.bash_pid:
        try:
          kill(self.bash_pid, SIGINT)
        except:
          return False

      if self.proc_pid:
        try:
          kill(self.proc_pid, SIGINT)
        except:
          return False
      self.cleanup()
      return False

    return True

  def cleanup(self):
    '''
    Removes all files related to the external process: .proc, .bash and root directory containing these two files
    '''
    try:
      # Sometimes rmtree may fail
      rmtree(self.dir_name)
    except:
      # in which case we can simply remove the separate files one by one
      try:
        remove(self.proc_path)
        remove(self.bash_path)
        rmdir(self.dir_name)
      except:
        if self.bash_pid or self.proc_pid:
          self.proc_status = ProcStatus.FAILED_STOP
        self.statusSignal.emit(self.proc_status)

  def getExitStatus(self):
    '''
    Checks inside the .proc file of the external process (if it exists) for its exit code

    A .proc file (stored at "./PROCESS_NAME/.proc") contains one or two lines based on the state of the external process:
      - single line only: if the process is still running its PID is stored here
      - two lines: if the process has stopped the first line stores its (now useless) PID and second one - the process' exit code
    '''
    # Check if .proc file exits
    self.checkFileExists(self.proc_path) # If file was not present we create a new empty one. If file is empty the code that follows will be omitted

    # Check if .proc file has a second line (remember: first line -> PID, second line -> exit code (if present))
    with (self.proc_path, 'r') as proc_file:
      lines =  []
      lines = proc_file.readlines()

      # Keep iterating until the bash script doesn't write the exit code of the process
      # TODO If the bash script fails to write the exit code to the file this will lead to an infinite loop! See how to avoid that. Maybe add a timeout but then what's next?
      while len(lines) != 2:
        lines = proc_file.readlines()

      self.exit_code = int(lines[1]) # Second line contains the exit code so we convert it from string to integer and store it


  @pyqtSlot()
  def start(self):
    '''
    Starts a bash script as detached process. The script will then spawn the given ROS process as a child
    process and receive its exit code once it stops. The result will be written inside the PID file .proc
    '''

    # Make sure that directories exist and both PID files are there (in case between init of the worker object and calling the start slot these have been deleted)
    self.checkRootDirExists()
    # If files are present and not empty then load PIDs
    self.loadPid(self.bash_path, self.bash_pid)
    self.loadPid(self.proc_path, self.proc_pid)

    # If we have restored the UI or triggered the start slot again while the PID of the bash process and external process are present we can skip the creation of the external processes
    if self.bash_pid and self.proc_pid:
      rospy.loginfo('SR2: Restoring state of UI component')
      self.proc_status = ProcStatus.RUNNING
      self.statusSignal.emit(self.proc_status)
      return

    rospy.loginfo('SR2: Attempting to launch ROS process')
    # Start the detached process with the bash script as the command and the rest as the arguments for the script
    # The working directory argument (here '/tmp') HAS to be present otherwise no PID will be returned (see Qt documentation)
    print('******************CHECK START********************')
    print(self.command, self.args)
    self.bash_status, self.bash_pid = QProcess.startDetached(self.command, self.args, '/tmp')
    print(self.bash_status, self.bash_pid)
    #self.bash_status, self.bash_pid = QProcess.startDetached('htop', [], '/tmp')
    self.writePidToFile(self.dir_name, self.bash_pid) # Write the PID of the started bash script inside the .bash file
    print('******************CHECK END********************')

    # Check if process has started properly
    if self.bash_status and self.bash_pid:
      rospy.loginfo('SR2: ROS process successfully started')
      self.proc_status = ProcStatus.RUNNING
      self.loadPid(self.proc_path, self.proc_pid)
    else: self.proc_status = ProcStatus.FAILED_START

    self.statusSignal.emit(self.proc_status)

  @pyqtSlot()
  def stop(self):
    rospy.loginfo('SR2: Attempting to stop ROS process')

    # Check if both the bash script and the ROS process are running
    if not self.checkProcessRunning():
      rospy.logwarn('SR2: External process not running so there is nothing to stop')
      self.proc_status = ProcStatus.INACTIVE
      self.statusSignal.emit(self.proc_status)
      return

    if not self.bash_pid or not self.proc_pid: return

    # Make sure that directories exist and both PID files are there (in case between init of the worker object and calling the start slot these have been deleted)
    # We need this here just to avoid exception handling
    self.checkRootDirExists()
    self.checkFileExists(self.proc_path)
    self.checkFileExists(self.bash_path)

    # First send SIGINT to external ROS process
    if self.proc_pid: kill(self.proc_pid, SIGINT)

    # After the ROS process has been stopped the bash script that has spawned it will receive its exit code
    # and write it to the .proc file
    # Check if exit code of ROS process was okay or not
    if self.isFileEmpty(self.proc_path):
      rospy.logwarn('SR2: ROS process has stopped prematurely')
      self.proc_status = ProcStatus.FAILED_STOP
      return

    # Retrieve the exit status of the ROS process from the .proc file (second line)
    self.getExitStatus()

    # Based on the exit code of the ROS process emit a signal
    if not self.exit_code: self.proc_status = ProcStatus.FINISHED
    else: self.proc_status = ProcStatus.FAILED_STOP
    self.statusSignal.emit(self.proc_status)
    # Send SIGINT to bash script
    if self.bash_pid: kill(self.bash_pid, SIGINT)

    # Clean up files and directory
    self.cleanup()

    # Reset PIDs
    self.bash_pid = None
    self.proc_pid = None

  @pyqtSlot()
  def status(self):
    '''
    An external timer triggers this slot at a specific interval
    The slot checks if the external ROS process is still running and reports back by emitting a statusSignal
    '''

    # In case the ROS process has finished, is inactive or failed we don't change the status
    if self.checkProcessRunning():
      self.proc_status = ProcStatus.RUNNING

#    print('PROC_STATUS =', self.proc_status)
    self.statusSignal.emit(self.proc_status)

