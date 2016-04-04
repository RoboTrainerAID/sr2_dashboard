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
  recover_signal = pyqtSignal()           # Signal triggers UI recovery routine
  statusChanged_signal = pyqtSignal(int)  # Q_PROPERTY notify for status
  block_signal = pyqtSignal(bool)         # (Un)blocks UI trigger (button)

  def __init__(self, name, cmd, pkg, args=None, parent=None):
    super(SR2Worker, self).__init__(parent)

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
    self.path = self.dir_name + '/pid_' + cmd + (pkg if pkg else '') + (args if args else '') + name
    self.path = self.path.replace('=','').replace('.','').replace(' ','').replace(':','').replace('~','')

# TODO Add cleanup upon destruction of worker and IF process is not running
#  def __del__(self):
#    if self._status != ProcStatus.RUNNING:
#      rospy.loginfo('SR2: External process not running. Will delete reserved (empty) PID file')
#      self.cleanup()

  @pyqtSlot()
  def recover(self):
    '''
    At the beginning the worker attemps to load a PID that was previously stored in the PID file for the given command (and arguments if present)
    If the attempt succeeds, the UI receives a RUNNING status and the worker regains control over the external process
    '''
    # Attempt recovery, it also creates the PID file if not present
    self.pid = self.loadPid()
    if self.pid:
      rospy.logwarn('SR2: Process with PID %d already running. Will connect to it', self.pid)
      self.active = True
      self.recover_signal.emit()
      self.setStatus(ProcStatus.RUNNING)
    else:
      # In case the recovery fails the created PID file is deleted
      # It will be recreated ONLY if the user triggers the start() slot
      rospy.loginfo('SR2: Unable to recover previous state')
      # if empty: self.cleanup()

  @pyqtSlot()
  def start(self):
    '''
    If specified external process isn't already running, it is created here
    In case the launch fails FAILED_START status is set and the user has to acknowledge the error before attempting to start the external process again
    '''
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
    '''
    If the external process is running, SIGINT signal is sent to it in an attempt to terminate it
    No matter the outcome of the SIGINT cleanup is triggered
    '''
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

  @pyqtSlot()
  def status(self):
    '''
    Timer in the same thread where the worker is triggers this slot in order to check if the external process'
    state has changed. In case the external process has crashed or has been terminated outside the dashboard
    FAILED_STOP status is set and the user has to acknowledge the error before attempting to stop the external process again
    '''
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
    '''
    Resets worker state to INACTIVE and removes the stored PID (if any present)
    Triggered whenever an error state is detected (FAILED_START or FAILED_STOP)
    '''
    self.setStatus(ProcStatus.INACTIVE)
    self.active = False
    self.pid = None

  def getStatus(self):
    '''
    Q_PROPERTY getter for status
    '''
    return self._status

  def setStatus(self, status):
    '''
    Q_PROPERTY setter for status
    If status has changed notification is triggered

    :param status: new status
    '''
    if self._status == status: return
    self._status = status
    self.statusChanged_signal.emit(self._status)

  statusProp = pyqtProperty(int, fget=getStatus, fset=setStatus, notify=statusChanged_signal)

  def checkPid(self, pid):
    '''
    Checks if a process with a given PID is running or not

    :param pid: PID of process that needs to be checked

    :return: False if PID doesn't represent a running process or an error has occurred
    '''
    if not pid: return False

    try: kill(pid, 0)
    except OSError: return False
    else: return True

  def checkPidFileExists(self):
    '''
    Checks if given file exists (not directory!) and creates one if it doesn't

    :return: False if file was nonexistent
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
    Loads a PID from a PID file if file exists and the PID stored in it represents a running process

    :return: PID if loading successful, else None
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

    :param pid: PID of process that needs to be checked

    :return: True PID represents a running process
    '''
    running = self.checkPid(pid)

    # A running external process here is considered to be a tuple of the running process itself and the bash that has spawned
    # If this requirement is not fulfilled we kill the one that is running
    if not running:
      if pid: # TODO Why did I put this here? :D
#        self.cleanup()
#        self.setStatus(ProcStatus.FAILED_START)
        return False
      return False
    return True

  def cleanup(self):
    '''
    Removes PID file
    '''
    try:
      if self.checkPidFileExists(): remove(self.path)
      else: rospy.loginfo('SR2: No PID file found. Nothing to cleanup')
    except: rospy.logerr('SR2: Failed to delete PID file')