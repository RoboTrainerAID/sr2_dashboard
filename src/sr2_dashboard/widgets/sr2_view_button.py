# Author: Aleksandar Vladimirov Atanasov
# Description: View UI component

# General
from os import kill, mkdir, exists, remove, rmdir, stat, error, path
from shutil import rmtree
from time import sleep
from exceptions import SIGINT
# ROS
import rospy as ROS
from rospkg import rospack
# PyQt, RQt related
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QToolButton, QIcon
from python_qt_binding.QtCore import QSize, Qt, QSizePolicy, QProcess, QThread, QObject, pyqtSignal, pyqtSlot, QTimer

from enum import Enum
class ProcStatus(Enum):
  INACTIVE      = 0      # The process hasn't been launched yet or has exited properly
  RUNNING       = 1       # The process has started successfully
  FAILED_START  = 2  # The process has failed to start
  FINISHED      = 3      # The process has exited succesfully (exit code == 0)
  FAILED_STOP   = 4   # The process has failed to exit properly (exit code != 0)

# TODO Add recovery functionality to both the ButtonWidget (for recovering the state of its UI) and the ExtProcessControlObj (for the external process management)

class SR2ButtonWidgetFactory:
  '''
  Generates simple ButtonWidget instances that are used to populate an SR2 menu entry view
  '''

  '''
  Useful information: https://idea.popcount.org/2012-12-11-linux-process-states/
  '''
  class ExtProcessControlObj(QObject):
    '''
    Used for controlling and
    '''
    @staticmethod
    def checkPid(pid):
      '''
      Checks if a process with a given PID is running or not
      '''
      try:
        kill(pid, 0)
        return True
      except OSError:
        return False

    def __init__(self, command, args=None):
      QObject.__init__(self)

      self.monitoring_proc = QProcess() # This is used to launch a detached bash script which will spawn the external ROS process
      self.signal_process_status = pyqtSignal('send_status')
      self.status = ProcStatus.INACTIVE

#      self.command = command
      self.args = args if True else ''

      self.pid_bash = -1
      self.pid_proc = -1
      self.exit_code = -1

      # The external process has two files (.proc and .bash (see below for more details)) which are stored inside ".CMD_ARG/" (inside the folder where THIS file is stored)
      # Both the folder and its content are hidden to prevent accidental deletion. Both files stored inside are required for the UI to restore its state (whether after a deliberate exit of the application or a crash)
      self.dir_name = './' + self.command + self.args
      self.proc_path = self.dir_name + '/.proc'
      self.bash_path = self.dir_name + '/.bash'

      # Bash script that spawns the external process (in itself the bash script is an external process too!)
      self.command = 'GET_EXIT_STATUS.SH with params'

      self.checkAndCreateProcDir()
      self.checkProcBashFileExist()

    def checkAndCreateProcDir(self):
      '''
      Check is the process folder is present and creates it if not
      '''
      if not exists(self.dir_name):
        ROS.loginfo('SR2: Process directory doesn\' exist and therefore will be created')
        mkdir(self.dir_name)

    def checkProcBashFilesExist(self):
      '''
      Creates the .proc and .bash files if those don't exist
      Note that the bash script that retrieves the exit code from the external process is the one that actually creates the files
      The creation here can only happen if a PID for either .bash or .proc files is available (>0) which happens if the UI is closed but the two processes are not shutdown
      Warning: must be called after checkAndCreateProcDir() to ensure that process folder is present
      '''
      # The default PID that the bash and external ROS process have is -1
      # The bash script is the one that creates and writes the data (PID and exit code) to the .proc file, which is then loaded in self.pid_proc and self.exit_code
      # In this case we return without doing nothing

      if not path.isfile(self.dir_name + '/.bash'):
        if self.pid_bash < 0: return
        elif self.pid_bash > 0:
          pidBashFile = open(self.bash_path, 'w')
          pidBashFile.write(str(self.pid_bash))
          pidBashFile.close()

      if not path.isfile(self.dir_name + '/.proc'):
        if self.pid_proc < 0: return
        elif self.pid_proc > 0:
          pidProcFile = open(self.proc_path, 'w')
          pidProcFile.write(str(self.pid_proc))
          pidProcFile.close()

    def isFileEmpty(self, path):
      '''
      Checks if a given file is empty
      '''
      # Use of exception handling here is necessary because if file doesn't exist the following won't work
      try: return stat(path).st_size == 0
      except error: return False

    @pyqtSlot()
    def slot_process_start(self):
      ROS.loginfo('SR2: Process will now start')
      # TODO Move toggleProcess for TRUE statement here
      self.status, self.pid_bash = QProcess.startDetached(self.command, self.args, '.')

      # Check again if process folder is present and create if required
      self.checkAndCreateProcDir()

      # Check if process has started properly
      if self.status and self.pid_bash:
        self.emit(self.signal_process_status, ProcStatus.RUNNING)
        # Create .proc (optional) and .bash file
        self.checkProcBashFilesExist()
      else: self.emit(self.signal_process_status, ProcStatus.FAILED_START)

      # Start monitoring the process' activity
      #self.runMonitoring()

    @pyqtSlot()
    def slot_process_stop(self):
      ROS.loginfo('SR2: Process will now stop')
      # Move toggleProcess for FALSE statement here

      # Check if processes (bash and ROS one) are running
      if not self.checkProcessRunning():
        ROS.logerr('SR2: External process is not running so there is nothing to stop')
        self.emit(self.signal_process_status, ProcStatus.INACTIVE)
        return

      # First send SIGINT to external ROS process
      kill(self.pid_proc, SIGINT)

      # TODO Check for success of kill() command: success = kill(...) => if success == 0 or success == None: OKAY

      # Check if external process has exited properly
      self.checkProcBashFilesExist()
      if self.isFileEmpty(self.proc_path):
        ROS.logwarn('SR2: External process has exited prematurely')

      # Check .proc file for second line and the exit code stored there
      self.getExitStatus()

      # Emit signal to UI based on exit code (ProcStatus.FAILED_FINISH for != 0 and ProcStatus.FINISHED for == 0)
      if not self.exit_code: self.emit(self.signal_process_status, ProcStatus.FINISHED)
      else: self.emit(self.signal_process_status, ProcStatus.FAILED_STOP)

      # Send SIGINT to bash
      kill(self.pid_bash, SIGINT)

      # TODO Check for success of kill() command: success = kill(...) => if success == 0 or success == None: OKAY

      # Delete .proc and .bash files
      self.deleteDirs()

    def checkProcessRunning(self):
      '''
      Using self.checkPid() it checks whether the process is running or not and then emits the appropriate signal
      '''
      # Call ps with the PID of the external process
#      self.monitoring_proc.start('ps', ['-p', str(self.pid)])
      # Wait for ps to finish
#      if self.monitoring_proc.waitForFinished():
        # If the exit status of ps is 0 then it has discovered the external process so True is returned
#        if not self.monitoring_proc.exitCode() and not self.monitoring_proc.exitStatus(): return True

      bash_runnning = self.checkPid(self.pid_bash)
      proc_running = self.checkPid(self.pid_proc)

      # A running external process here is considered to be a tuple of the running process itself and the bash that has spawned
      # If this requirement is not fulfilled we kill the one that is running
      if not bash_runnning or not proc_running:
        kill(self.pid_bash, SIGINT)
        kill(self.pid_proc, SIGINT)
        self.deleteDirs()
        return False

      return True

    def deleteDirs(self):
      try:
        rmtree(self.dir_name)
      except:
        remove(self.proc_path)
        remove(self.bash_path)
        rmdir(self.dir_name)

    @pyqtSlot()
    def runMonitoring(self):
      '''
      Every N seconds the thread checks the state of the external process and reports back to the UI emitting the appropriate signals
      '''
      #while self.status not in [ProcStatus.INACTIVE, ProcStatus.FAILED_STOP, ProcStatus.FINISHED]:
      if self.status in [ProcStatus.INACTIVE, ProcStatus.FAILED_STOP, ProcStatus.FINISHED]: return;

      # Check if process has stopped in which case we need to check for the exit status inside the .proc file
# TODO See if calling a slot funciton like this (and not by emitting a signal) works
      if not self.checkProcessRunning():
        # self.getExitStatus()
        self.slot_process_stop()
      else:
        self.signal_process_status.emit(ProcStatus.RUNNING)

      # No need to start the process every milisecond (TODO see if this interval cannot be increased)
      #sleep(1) # Replaced by timer timeout signal

    def getExitStatus(self):
      '''
      Checks inside the .proc file of the external process (if it exists) for its exit code

      A .proc file (stored at "./PROCESS_NAME/.proc") contains one or two lines based on the state of the external process:
        - single line only: if the process is still running its PID is stored here
        - two lines: if the process has stopped the first line stores its (now useless) PID and second one - the process' exit code
      '''
      # Check if .proc file exits
      self.checkProcBashFilesExist() # If file was not present we create a new empty one. If file is empty the code that follows will be omitted

      # Check if .proc file has a second line (remember: first line -> PID, second line -> exit code (if present))
      lines =  []
      with (self.proc_path, 'r') as proc_file:
        lines = proc_file.readlines()

        # Keep iterating until the bash script doesn't write the exit code of the process
# TODO If the bash script fails to write the exit code to the file this will lead to an infinite loop! See how to avoid that. Maybe add a timeout but then what's next?
        while len(lines) != 2:
          lines = proc_file.readlines()

      self.exit_code = int(lines[1]) # Second line contains the exit code so we convert it from string to integer and store it

  class ButtonWidget(QWidget):
    '''
    A widget with a button controlling a single process (in case of roslaunch it controlls
    indirectly all processes that were spawned by that command due to the nature of roslaunch)
    '''
    # TODO Remove checkPid and all low-level process related since it is being move to ExtProcessControlObj
    @staticmethod
    def checkPid(pid):
      '''
      Checks if a process with a given PID is running or not
      '''
      try:
        kill(pid, 0)
        return True
      except OSError:
        return False

    def onResize(self, event):
      '''
      Called whenever a resize event is triggered on the widget. It resizes the icon (if present) inside the button
      '''
      if self.icons != None:
          self.qbtn.setIconSize(QSize(self.qbtn.width()/2, self.qbtn.height()/2))

    def createPidFile(self):
      '''
      Creates a PID file

      A PID file is used for storage of the detached process
      In case the UI crashes or is closed intentionally this
      file is used to restore the UI's state and reconnect it
      to the detached processes so that these can be controlled
      The format of a PID file for now is as follows:
      rosrun:       rospkg + '_' + 'rosrun' + '_' + 'nodeName' + '.pid'
      roslaunch:    rospkg + '_' + 'roslaunch' + '_' + 'launchFile1' + '_' + 'launchFile2' + '_' + ... + '.pid'
      '''
      if path.isfile(self.pidFilePath):
        with open(self.pidFilePath) as pidF:
          self.pid = int(pidF.readline())
          ROS.loginfo('SR2: Found "%s". Restoring connection to detached process with PID %d', self.pidFilePath, self.pid)
          self.status = True
      else:
        ROS.logwarn('SR2: No "%s" detected. If you have started the detached process, closed the UI and deleted this file, the application will be unable to restore its state and the external process will be orphaned!', self.pidFilePath)


    def checkPidDir(self):
      '''
      Ensures that the .pid directory is present. The directory is where all PID files are written. It is hidden in order to avoid involuntary deletion
      '''
      if not exists('.pid'):
        ROS.loginfo('SR2: Unable to detect existing ".pid" folder. Creating (hidden) folder ".pid" for storing PID files')
        mkdir('.pid')

    def __init__(self, pkg, cmd, args, captions, icons=None):
      super(SR2ButtonWidgetFactory.ButtonWidget, self).__init__()
      QWidget.__init__(self)

      print('SR2: Generating button')
      self.captions = captions
      self.icons = icons
      self.command = cmd
      self.args = args
      self.status = False
      self.pkg = pkg
      self.pid = 0

      self.thread = QThread()
      # Control/monitor the process
      self.signal_process_start = pyqtSignal('send_start')           # Starts the detachted process
      self.signal_process_stop = pyqtSignal('send_stop')             # Stops the detached process

      self.slot_process_status = pyqtSlot('receive_status')     # Gets information about the process

      # See if .pid directory is present (otherwise we will be unable to write files there)
      self.checkPidDir()

      arg2str = ''
      for i in range(0, len(self.args)): arg2str = arg2str + self.args[i]
      self.pidFilePath =  '.pid/%s_%s_%s.pid' % (self.pkg, self.command, arg2str)

      # Create PID file if not present (unless the user has deliberately deleted it the file will be present as long as the process hasn't been shutdown by the UI)
      self.createPidFile()

      # Initialize the components of the ButtonWidget
      self.initUi()

      # Initialize monitoring and control infrastructure
      self.initProcControllAndMonitoring()

    def __del__(self):
      # Stop thread
      self.thread.quit()
      # Wait until thread has really stopped
      while not self.thread.isFinished(): pass

    def initUi(self):
      '''
      Creates a simple UI for the widget with a single button in it
      For the functionality behind the button see toggleProcess()
      '''
      self.hbox = QHBoxLayout()

      self.qbtn = QToolButton(self)
      self.qbtn.setText(self.captions[0])
      self.qbtn.setIcon(QIcon(self.icons[0]))
      self.qbtn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)  # Not the best looking layout of a button's content but it's the only decent thing that works out of the box

      self.qbtn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
      self.qbtn.setCheckable(True)
      self.qbtn.resizeEvent = self.onResize

      if self.status:
        self.qbtn.setChecked(True)
        self.qbtn.setText(self.captions[1])
        self.qbtn.setIcon(QIcon(self.icons[1]))

      self.qbtn.clicked.connect(self.toggleProcess)
      self.qbtn.resize(self.qbtn.sizeHint())
      self.hbox.addWidget(self.qbtn)
      self.setLayout(self.hbox)
      self.setGeometry(300, 300, 250, 150)
      self.show()

    def initProcControllAndMonitoring(self):
      '''
      Initializes the ExtProcessControlThread object, connects the UI to it (and vice versa), starts a thread and moves it to it
      '''
      self.procObj = SR2ButtonWidgetFactory.ExtProcessControlObj()
      # Set the command and its arguments
      # Example: [cmd][args] -> [roslaunch][lt listener.launcher]
      self.procObj.command = self.command
      self.procObj.args = self.pkg + ' ' + self.args

      # Connect the slots and the signals
      self.signal_process_start.connect(self.procObj.slot_process_start)
      self.signal_process_stop.connect(self.procObj.slot_process_stop)
      self.procObj.signal_process_status.connect(self.slot_process_status)

      # Create a timer that will trigger runMonitoring() slot of worker whenever its timeout() signal is emitted
      self.timer = QTimer()
      self.timer.setInterval(10)  # Emit timeout() every 10ms
      # Connect the monitoring slot of the worker to the timer
      self.timer.timeout.connect(self.procObj.runMonitoring)
      # Start the timer once the thread has started (we ensure that the signals also belong to the thread => same thread affinity as the worker)
      self.thread.started.connect(self.timer.start)

      # deleteLater() ensures that the allocated memory by the worker and timer is released once the thread is stopped (we need to do that because PyQt is basically a wrapper and we have to consider the C++ that lies underneath)
      self.thread.finished.connect(self.procObj.deleteLater)
      self.thread.finished.connect(self.timer.deleteLater)

      # Change thread affinity of both the worker and the timer
      self.timer.moveToThread(self.thread)
      self.procObj.moveToThread(self.thread)

      # Start the thread (=>start the timer)
      self.thread.start()

    def slot_process_status(self, status):
      ROS.loginfo('Received STATUS change from process: %d', status)

      # TODO Add change of pixmap of button below
      if status in [ProcStatus.FAILED_START, ProcStatus.FAILED_STOP]: pass
        #self.qbtn.setPixmap(...)
      else: pass
        #if status == Status.INACTIVE: self.qbtn.setPixmap(...)
        #elif status == Status.RUNNING: self.qbtn.setPixmap(...)
        #else: self.qbtn.setPixmap(...)

      self.qbtn.setDisabled(False)

#    def broadcastedStop(self):
#      '''
#      Turns the detached process down
#      '''
#      ROS.loginfo('SR2: Stop signal')
#      self.toggleProcess(False)

    def toggleProcess(self, val):
      '''
      Controls a process (on/off states only) that is controlled by the button calling it
      It also stores the PID of the process inside a special PID file (*.pid), which is used
      in the recovery mechanism of the UI in a case of an UI crash or when the UI is closed
      but the spawned detached processes are not terminated using it
      '''
      # If button pressed
      if val:
        cmd_args = [self.pkg] + self.args
        ROS.loginfo('SR2: Starting process for command "%s" with arguments "%s"', self.command, cmd_args) #self.pkg CONCAT with self.args
        # Note: when roslaunch is terminated all processes spawned by it are also terminated
        # thus even if roscore has been started by the roslaunch process it will too be stopped :)
        # Note that all arguments (including package name) have to be inside a list of strings
        self.status, self.pid = QProcess.startDetached(self.command, cmd_args, '.')

        if self.status:
          # Change to pressed state
          ROS.loginfo('SR2: Process successfully started with PID "%d"', self.pid)
          pidFile = open(self.pidFilePath, 'w')
          pidFile.write(str(self.pid))
          pidFile.close()
          self.qbtn.setText(self.captions[1])
          self.qbtn.setIcon(QIcon(self.icons[1]))
        else:
          self.qbtn.setChecked(False)
          # Change to warning state
          self.qbtn.setText(self.captions[2])
          self.qbtn.setIcon(QIcon(self.icons[2]))
          ROS.logerr('SR2: Failed to start process')
      else:
        ROS.loginfo('SR2: Stopping process')
        if self.status:
          # kill takes a very short amount of time hence we can call it from inside the main thread without freezing the UI
          self.success = None
          # WARNING: ROS documentation states that "rosnode kill" is not guaranteed to succeed
          # especially when the node that is to be killed has its "respawn" property turned on
          # hence we use the kill command with SIGINT signal for both cases - roslaunch and rosrun

          if SR2ButtonWidgetFactory.ButtonWidget.checkPid(self.pid):
            self.success = kill(self.pid, SIGINT)
          else:
            ROS.logerr('SR2: No process with PID "%d" detected', self.pid)
            if path.isfile(self.pidFilePath): remove(self.pidFilePath)

          if self.success == 0 or self.success == None:
            ROS.loginfo("SR2: Process stopped!")
            self.status = False
            self.pid = 0

            if path.isfile(self.pidFilePath): remove(self.pidFilePath)

            self.qbtn.setText(self.captions[0])
            self.qbtn.setIcon(QIcon(self.icons[0]))
          else:
            self.qbtn.setChecked(True)
            self.qbtn.setText(self.captions[2])
            self.qbtn.setIcon(QIcon(self.icons[2]))
            ROS.logerr('SR2: Failed to stop process')

  # TODO Change all ROS.loginfo() to ROS.logdebug() wherever necessary
  @staticmethod
  def createButtonWidget(yaml_button_config):
    '''
    Parses YAML content and generates a ButtonWidget from it
    '''
    ROS.logdebug('SR2: Parsing button configuration', yaml_button_config)
    assert yaml_button_config != None, 'SR2: Empty button configuration'

    pkg = ''
    cmd = ''
    args = []
    # Try each of the possible configurations: node, launch and service
    # TODO Check if launch files etc exist or always launch the chosen command and let it fail if these are not present (let ROS handle the search for the files) - we can check the processes
    try:
      pkg = yaml_button_config['package']
      ROS.loginfo('Using package %s', pkg)
      try:
        args = yaml_button_config['node']
        ROS.loginfo('Nodes detected. Will use "rosrun"')
        cmd = 'rosrun'

      except KeyError:
        try:
          _args = yaml_button_config['launch']
          ROS.loginfo('Launch file(s) detected. Will use "roslaunch"')
          cmd = 'roslaunch'

          args.append(_args + '.launch')
          ROS.loginfo('Single launch file detected: "%s"', args)

        except KeyError:
          try:
            args.append(yaml_button_config['serivce'])
            ROS.loginfo('Service deteceted. Will use "rosservice call"')
            cmd = 'rosservice call'

          except KeyError as exc:
            ROS.logerr('Button does not contain data that can be executed by the supported ROS tools. Add node, launch or service to the button\'s description')
            raise exc

      captions = []
      captions.append(yaml_button_config['captions']['default'])
      captions.append(yaml_button_config['captions']['pressed'])
      captions.append(yaml_button_config['captions']['warning'])


      '''
      TODO Currently the parsing of the paths for the icons is not working properly
      The configuration file has to support 2 types of location procedures for the given files inside it based on how much information is provided by the configuration string:
       1. Given absolute path for file - using os.path.isabs(STRING) we can check if the given string is a valid abosulte path that points to a file
       2. Given file name (with parent directory's name attached to it) for the icon (example: status/status_ok.png) - string doesn't contain full path so we need to find it:
        1.1 Look for file inside sr2_dashboard/resources/images (example: sr2_dashboard/resources/images/status/status_ok.png) (use rospkg to get package root folder and from there go into resources/images/)
        1.2 If 1.1 fails look for file inside shared resources for the installed cob_dashboard, pr2_dashboard etc. located in /opt/ros/indigo/share/... (use rospkg to get installation dir of ROS if possible?)

       If string is invalid for both 1. and 2. we go with the default (pr2 or cob dashboard images)
      '''

      #icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]
      icons = []
#                        for path in icon_paths:
#                            paths.append(os.path.join(rp.get_path(path[0]), path[1]))
#                        _icon_helper = IconHelper(paths, name)
#                        converted_icons = self._icon_helper.set_icon_lists(icons)
#                        _icons = converted_icons[0]
      # TODO Use the IconHelper to populate the icons list using the names of the OR NOT!
      # TODO Change icons to INACTIVE, RUNING, STOPPED, FAILED (used for both failed to start and failed to quit properly)
      icons.append(yaml_button_config['icons']['default'])
      icons.append(yaml_button_config['icons']['pressed'])
      icons.append(yaml_button_config['icons']['warning'])

    except KeyError as exc:
      ROS.logerr('SR2: error while loading YAML file.')
      ROS.loginfo('SR2: full message: \n"%s"' % exc)
      return None

    #return SR2DashboardItemFactory.SR2MenuView.SR2ButtonWidgetFactory.ButtonWidget(yamlButtonConfig['caption'], yamlButtonConfig['rospkg'], yamlButtonConfig['roscommand'], yamlButtonConfig['target'], yamlButtonConfig['icon'])
    return SR2ButtonWidgetFactory.ButtonWidget(pkg, cmd, args, captions, icons)