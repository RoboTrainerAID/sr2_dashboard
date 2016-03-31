# Author: Aleksandar Vladimirov Atanasov
# Description: A SR2 buttons (for both toolbar and view widgets)

# YAML
from yaml import YAMLError

# PyQt
# QtGui modules
from python_qt_binding.QtGui import QPushButton, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QSpacerItem, QSizePolicy, QFrame
# QtCore modules
from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer, QThread, pyqtSlot, pyqtSignal, QThreadPool, QSize
#from PyQt4.QtSvg import QSvgRenderer # TODO Add support for SVG for the icons and pixmaps: http://stackoverflow.com/a/35138314/1559401 | http://www.qtcentre.org/threads/7321-Loading-SVG-icons

import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
# ROS-related  modules: RQT Robot Dashboard
from rqt_robot_dashboard.icon_tool_button import IconToolButton
#from rqt_robot_dashboard.widgets import ...

from sr2_monitor_object import SR2Worker_v2, ProcStatus#, SR2Worker
from sr2_runnable_object import ServiceRunnable
from ..misc.sr2_grid_generator import SR2GridGenerator as sr2gg
from ..misc.sr2_ros_entry_extraction import SR2PkgCmdExtractor, IconType

class Status():
  '''
  Remaps the IconType values for better code semantics and readability
  '''
  inactive = IconType.inactive
  running = IconType.running
  error = IconType.error

class SR2Button():
  @staticmethod
  def createButton(context, yaml_entry_data, name):
    '''
    Parses a YAML node for either a toolbar or a view. Based on successful parsing results on of the following types of buttons will be returned:

      - SR2ButtonExtProcess (subclass of QPushButton; for toolbar) - a menu entry that launches an external process, stops it and also monitors its running status
      - SR2ViewButtonExtProcess (subclass of QWidget; for view) - a view entry that launches an external process, stops it and also monitors its running status
      - SR2ButtonService (subclass of QPushButton; for toolbar) - a menu entry that calls a ROS Trigger-based service and displays its reply
      - SR2ViewButtonService (subclass of QWidget; for view) - a view entry that calls a ROS Trigger-based service and displays its reply
      - SR2ToolbarButtonWithView (subclass of QToolButton; for toolbar) - a menu entry that opens a view in the main view of the SR2 Dashaboard
    '''
    if not yaml_entry_data: return None

    pkg = ''
    cmd = ''
    args = ''
    timeout = 0

    if 'type' in yaml_entry_data:
      # We have an entry that is part of the toolbar
      if yaml_entry_data['type'] == 'noview':
        pkg, cmd, args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data['menu_entry'])
        if (not cmd and not pkg and not args and not timeout): return None
        rospy.loginfo('\n----------------------------------\n\tCREATE TOOLBAR BUTTON\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n\t@Timeout: %d\n----------------------------------', name, cmd, args, pkg, timeout)
        if timeout:
          # Service
          if not args:
            rospy.logerr('SR2: Trying to create noview service button but service target is empty')
            return None
          return SR2ButtonService(name, args, timeout)
        else:
          # External process (roslaunch, rosrun or app)
          return SR2ButtonExtProcess(name, cmd, pkg, args)
      elif yaml_entry_data['type'] == 'view':
        # View
        rospy.loginfo('\n----------------------------------\n\tCREATE TOOLBAR VIEW\n@Yaml_Contents: %s\n----------------------------------', yaml_entry_data)
        return SR2ButtonWithView(name, yaml_entry_data['menu_entry'], context)
      else:
        rospy.logerr('SR2: Unknown type of entry. Please make sure to specify "type" as either "noview" or "view"')
        return None
    else:
      # We have an entry that is part of a view
      pkg, cmd, args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data)
      if (not cmd and not pkg and not args and not timeout): return None
      rospy.loginfo('\n----------------------------------\n\tCREATE VIEW BUTTON\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n\t@Timeout: %d\n----------------------------------', name, cmd, args, pkg, timeout)
      if timeout:
        # Service
        if not args:
          rospy.logerr('SR2: Trying to create noview service button but service target is empty')
          return None
        return SR2ViewButtonService(name, args, timeout)
      else:
        # External process (roslaunch, rosrun or app)
        return SR2ViewButtonExtProcess(name, cmd, pkg, args)

##############################################################################################################################################
#########################################################  SR2ButtonExtProcess  ##############################################################
##############################################################################################################################################
class SR2ButtonExtProcess(IconToolButton):
  '''
  Part of a toolbar; launches an external application, provides status feedback while monitoring for that application and the ability to stop it
  '''
  start_signal = pyqtSignal()
  stop_signal = pyqtSignal()

  def __init__(self, name, cmd, pkg, args, surpress_overlays=False, minimal=True):
    _icons = IconType.loadIcons(name)
    super(SR2ButtonExtProcess, self).__init__(name, icons=_icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    rospy.loginfo('\n----------------------------------\n\tEXT.PROC\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n----------------------------------', name, cmd, args, pkg)

    self.minimal = minimal
    self.setObjectName(name)
    self.name = name
    self.icons = _icons[0]
    self.setStyleSheet('QToolButton {border: none;}')
    self.setToolTip(self.name)
    self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))

    self.cmd = cmd
    self.args = args
    self.pkg = pkg

    self.disabled = False # True when starting/stopping is ongoing else False
    self.active = False   # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
    self._status = Status.inactive
    self.setIcon(self.icons[IconType.inactive])

    self.clicked.connect(self.start)
    self.clicked.connect(self.stop)

    self.mutex_recovery = QMutex()
    self.mutex_status = QMutex()

    self.worker_thread = QThread()
    self.createWorker()

  def createWorker(self):
    '''
    Creates the following components:
      - Separate thread - used for launching and monitoring a detached process
      - SR2 Worker - a QObject which runs inside the thread and is the control entity for the external process
      - Timer - used for triggering a lot inside the SR2 Worker that sends back information about the status of the external process
    '''
    self.worker = None
    if self.pkg: self.worker = SR2Worker_v2(self.cmd, self.pkg, self.args)
    else: self.worker = SR2Worker_v2(cmd=self.cmd, pkg=None, args=self.args)

    self.timer = QTimer()
    self.timer.setInterval(1000)

    self.worker.moveToThread(self.worker_thread)
    self.timer.moveToThread(self.worker_thread)

    self.start_signal.connect(self.start)
    self.stop_signal.connect(self.stop)
    self.start_signal.connect(self.worker.start)
    self.stop_signal.connect(self.worker.stop)

    self.worker_thread.finished.connect(self.worker_thread.deleteLater)
    self.worker_thread.finished.connect(self.worker.deleteLater)
    self.worker_thread.finished.connect(self.timer.deleteLater)
    self.worker_thread.started.connect(self.timer.start)

    self.timer.timeout.connect(self.worker.status)

    self.worker.statusChanged_signal.connect(self.status)
    self.worker.block_signal.connect(self.block)

    self.worker_thread.start()

  @pyqtSlot()
  def start(self):
    '''
    Attempt to start the external process
    '''
    pass
#    if self.disabled: return
#
#    if not self.active:
#      rospy.loginfo('SR2: Attempting to start external process')
#      self._status = Status.running
#      self.setIcon(self.icons[IconType.running])
#      self.start_signal.emit()

  @pyqtSlot()
  def stop(self):
    '''
    Attempt to stop the external process
    '''
    if self.disabled: return

    if self.active:
      rospy.loginfo('SR2: External process stopped')
      self.stop_signal.emit()

  @pyqtSlot(bool)
  def block(self, block):
    '''
    Blocks the button until the external process is started or stopped
    It prevents a situation where the external process requires relatively long time to start/stop but the user
    attempts to click on the button during that time resulting in a faulty state
    '''
    self.disabled = block

  @pyqtSlot(int)
  def status(self, status):
    '''
    Receive status from the worker based on the state of the external process this button represents
    '''
    pass
#    lock = QMutexLocker(self.mutex_status)
#
#    if status == ProcStatus.RUNNING:
#      self._status = Status.running
#      self.setIcon(self.icons[IconType.running])
#    elif status in [ProcStatus.FINISHED, ProcStatus.INACTIVE]:
#      self._status = Status.inactive
#      self.setIcon(self.icons[IconType.inactive])
#    elif status in [ProcStatus.FAILED_START, ProcStatus.FAILED_STOP]:
#      self._status = Status.error
#      self.setIcon(self.icons[IconType.error])
#
#    if self._status != Status.running: self.active = False
#    else: self.active = True

  def __del__(self):
    if(self.worker_thread.isRunning()):
      self.worker_thread.exit()
      while(not self.worker_thread.isFinished()):
        pass

##############################################################################################################################################
#########################################################  SR2ButtonExtProcess  ##############################################################
##############################################################################################################################################
class SR2ViewButtonExtProcess(QWidget):
  '''
  '''
  start_signal = pyqtSignal()
  stop_signal = pyqtSignal()
  clear_error_signal = pyqtSignal()

  def __init__(self, name, cmd, pkg, args):
    _icons = IconType.loadIcons(name)
    super(SR2ViewButtonExtProcess, self).__init__()

    rospy.loginfo('\n----------------------------------\n\tEXT.PROC\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n----------------------------------', name, cmd, args, pkg)

    self.setObjectName(name)
    self.name = name
    self.icons = _icons[0]
    #self.setStyleSheet('QToolButton {border: none;}')
    #self.setToolTip(self.name)
    #self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))

    self.cmd = cmd
    self.args = args
    self.pkg = pkg
    self.setWindowTitle('Ext.process "' + self.cmd + '"')

    layout = QVBoxLayout(self)
    layout.setObjectName(name + 'Layout Ext Proc')

    controls_layout = QHBoxLayout()
    spacer = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
    controls_layout.addItem(spacer)
    self.status_label = QLabel(self)
    self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    self.status_label.setScaledContents(True);
    self.status_label.setToolTip('Ext.process "' + self.cmd + ' ' + self.args + '"' + ((' from package "' + self.pkg + '"') if self.pkg else '') + ' inactive')
    self.status_label.setPixmap(self.icons[IconType.inactive].pixmap(self.icons[IconType.inactive].availableSizes()[0])) # Convert icon to pixmap: http://stackoverflow.com/a/27057295/1559401
    controls_layout.addWidget(self.status_label)
    self.execute_button = QPushButton('Execute', self)
    self.execute_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    controls_layout.addWidget(self.execute_button)
    spacer = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
    controls_layout.addItem(spacer)
    layout.addLayout(controls_layout)

    info_layout = QVBoxLayout()
    line = QFrame(self)
    line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    info_layout.addWidget(line)
    cmdL = QLabel('Cmd: ' + self.cmd, self)
    cmdL.setWordWrap(True)
    info_layout.addWidget(cmdL)
    if self.pkg:
      pkgL = QLabel('Pkg: ' + self.pkg, self)
      pkgL.setWordWrap(True)
      info_layout.addWidget(pkgL)
    if self.args:
      argsL = QLabel('Args: ' + self.args, self)
      argsL.setWordWrap(True)
      info_layout.addWidget(argsL)
    layout.addLayout(info_layout)

    self.statusOkay = True # Used for activating the acknowledgement mode where the user has to confirm the error before trying to launch the process again
    self.active = False    # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
    self.toggleControl = False

    self.mutex_recovery = QMutex()
    self.mutex_status = QMutex()
    
    self.worker_thread = QThread()
    self.createWorker()
    self.execute_button.clicked.connect(self.toggle)

    self.setLayout(layout)
    self.resize(layout.sizeHint())

  def onResize(self, event):
    '''
    Resize icon in label based on size of parent widget
    '''
    # Resize icon of button
    # OR implement custom button and override paintEvent(event)
    self.status_label.setIconSize(QSize(self.status_label.width()/2, self.status_label.height()/2))
    super.onResize(event)

  def createWorker(self):
    '''
    Creates a worker (controls and monitors an external process), timer (worker reprots to the UI every 1s) and a thread (holds both the worker and timer)
    '''
    # Create worker and connect the UI to it
    self.worker = None
    if self.pkg: self.worker = SR2Worker_v2(self.cmd, self.pkg, self.args)
    else: self.worker = SR2Worker_v2(cmd=self.cmd, pkg=None, args=self.args)
#    self.worker.recover()
    QTimer.singleShot(1, self.worker.recover)
    self.worker.statusChanged_signal.connect(self.statusChangedReceived)
    self.worker.block_signal.connect(self.block)
    self.worker.recover_signal.connect(self.recover)
    self.start_signal.connect(self.worker.start)
    self.stop_signal.connect(self.worker.stop)
    self.clear_error_signal.connect(self.worker.clear_error)

    # Create a timer which will trigger the status slot of the worker every 1s (the status slot sends back status updates to the UI (see statusChangedReceived(self, status) slot))
    self.timer = QTimer()
    self.timer.setInterval(1000)
    self.timer.timeout.connect(self.worker.status)

    # Connect the thread to the worker and timer
    self.worker_thread.finished.connect(self.worker.deleteLater)
    self.worker_thread.finished.connect(self.timer.deleteLater)
    self.worker_thread.started.connect(self.timer.start)

    # Move the worker and timer to the thread...
    self.worker.moveToThread(self.worker_thread)
    self.timer.moveToThread(self.worker_thread)

    # Start the thread
    self.worker_thread.start()


  def __del__(self):
    if(self.worker_thread.isRunning()):
      self.worker_thread.exit()
      while(not self.worker_thread.isFinished()):
        pass


  @pyqtSlot(int)
  def statusChangedReceived(self, status):
    '''
    Update the UI based on the status of the running process
    :param status - status of the process started and monitored by the worker
    Following values for status are possible:
      - INACTIVE/FINISHED - visual indicator is set to INACTIVE icon; this state indicates that the process has stopped running (without error) or has never been started
      - RUNNING - if process is started successfully visual indicator
      - FAILED_START - occurrs if the attempt to start the process has failed
      - FAILED_STOP - occurrs if the process wasn't stop from the UI but externally (normal exit or crash)
    '''
    print(' --- main thread ID: %d ---' % QThread.currentThreadId())
    if status == ProcStatus.INACTIVE or status == ProcStatus.FINISHED:
      rospy.loginfo('SR2: Status has changed to: INACTIVE/FINISHED')
      self.status_label.setPixmap(self.icons[IconType.inactive].pixmap(self.icons[IconType.inactive].availableSizes()[0]))
#      self.execute_button.setDisabled(False)
      self.execute_button.setText('Execute')
      self.active = False
    elif status == ProcStatus.RUNNING:
      rospy.loginfo('SR2: Status has changed to: RUNNING')
      self.status_label.setPixmap(self.icons[IconType.running].pixmap(self.icons[IconType.running].availableSizes()[0]))
#      self.execute_button.setDisabled(False)
      self.execute_button.setText('Stop')
      self.active = True
    elif status == ProcStatus.FAILED_START:
      rospy.logerr('SR2: Status has changed to: FAILED_START')
#      self.execute_button.setDisabled(False)
      self.status_label.setPixmap(self.icons[IconType.error].pixmap(self.icons[IconType.error].availableSizes()[0]))
      self.execute_button.setText('Confirm')
      self.statusOkay = False
      self.active = True
    elif status == ProcStatus.FAILED_STOP:
      rospy.logerr('SR2: Status has changed to: FAILED_STOP')
#      self.execute_button.setDisabled(False)
      self.status_label.setPixmap(self.icons[IconType.error].pixmap(self.icons[IconType.error].availableSizes()[0]))
      self.execute_button.setText('Confirm')
      self.statusOkay = False
      self.active = True

  @pyqtSlot(bool)
  def block(self, block_flag):
    '''
    Enable/Disable the button which starts/stops the external process
    This slot is used for preventing the user to interact with the UI while starting/stopping the external process after a start/stop procedure has been initiated
    After the respective procedure has been completed the button will be enabled again
    :param block_flag - enable/disable flag for the button
    '''
    self.execute_button.setDisabled(block_flag)

  @pyqtSlot()
  def toggle(self):
    '''
    Handles the start, stopping and error confirmation triggered by the button
      - statusOkay is False - this occurs ONLY if the process status is an error (FAILED_START or FAILED_STOP)
                                 In this case the user has to click twice on the button in order to reinitiate the starting procedure
      - both statusOkay and toggleControl are True - attempt to start the process
      - statusOkay is True but toggleControl is False - attempt to stop the process
    '''
    if not self.statusOkay:
      # If an error has occurred the first thing the user has to do is reset the state by acknowleding the error
      self.statusOkay = True
      print('Error acknowledged')
      self.clear_error_signal.emit()
      return

    self.toggleControl = not self.toggleControl
    if self.toggleControl: self.start_signal.emit()
    else: self.stop_signal.emit()
    
  @pyqtSlot()
  def recover(self):
    self.toggleControl = True
    self.active = True
    self.statusOkay = True

##############################################################################################################################################
######################################################  SR2ToolbarButtonService  #############################################################
##############################################################################################################################################
class SR2ButtonService(IconToolButton):
  '''
  Part of a toolbar; initiates a service call and reports back once service has replied or timeout
  '''
  @pyqtSlot()
  def call(self):
    if not self.disabled:
      rospy.loginfo('SR2: Calling service %s from thread %d with timeout set to %d', self.args, int(QThread.currentThreadId()), self.timeout)
      self.thread_pool.start(self.service)
    else:
      rospy.loginfo('SR2: Service call is currently being processed. Please wait...')
      self.setIcon(self.icons[IconType.running])
      self.disabled = True

  @pyqtSlot(bool)
  def block(self, state):
    if state: self.setIcon(self.icons[IconType.running])
    self.disabled = state

  @pyqtSlot(int, str)
  def reply(self, status, msg):
    if status in [ServiceRunnable.CallStatus.SUCCESS_TRUE, ServiceRunnable.CallStatus.SUCCESS_FALSE]:
      self.setIcon(self.icons[IconType.inactive])
      rospy.loginfo('SR2: Calling service %s from thread %d successful. Service returned status %s with message "%s"', self.args, int(QThread.currentThreadId()), ('True' if not status else 'False'), msg)
    else:
      self.setIcon(self.icons[IconType.error])
      rospy.logerr('SR2: Calling service %s from thread %d failed due to "%s"', self.args, int(QThread.currentThreadId()), msg)

    self.tooltip = '<nobr>' + self.name + ' : "rosservice call ' + self.args + '"</nobr><br/>Reply: ' + msg

  def __init__(self, name, args, timeout, minimal=True):
    # Load icons
    _icons = IconType.loadIcons(name)
    super(SR2ButtonService, self).__init__(name, icons=_icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    self.icons = _icons[0]
    self.setStyleSheet('QToolButton {border: none;}')
    self.minimal = minimal
    self.name = name
    self.timeout = timeout
    self.args = args  # Args contains the Trigger service that we want to call

    rospy.loginfo('\n----------------------------------\n\tSERVICE\n\t@Name: %s\n\t@Args: %s\n\t@Timeout: %d\n----------------------------------', name, args, timeout)

    self.setIcon(self.icons[IconType.inactive])
    self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))
    self.tooltip = self.name + ' : "' + 'rosservice call' + ' ' + self.args + '"<br/>Reply: --'
    self.setToolTip(self.tooltip)

    self.thread_pool = QThreadPool()
    self.service = ServiceRunnable(self.args, self.timeout)
    self.service.setAutoDelete(False)
    self.service.signals.srv_running.connect(self.block)
    self.service.signals.srv_status.connect(self.reply)
    self.clicked.connect(self.call)

    self.disabled = False

##############################################################################################################################################
##########################################################  SR2ViewButtonService  ############################################################
##############################################################################################################################################
class SR2ViewButtonService(QWidget):
  '''
  Part of a view; initiates a service call and reports back once service has replied or timeout
  '''
  @pyqtSlot()
  def call(self):
    '''
    If button is enabled, initiate a service call
    '''
    if not self.disabled:
      self.reply_statL.setText('Reply status:')
      self.reply_msgL.setText('Reply message:')
      rospy.loginfo('SR2: Calling service %s from thread %d with timeout %d', self.args, int(QThread.currentThreadId()), self.timeout)
      self.thread_pool.start(self.service)
    else:
      rospy.loginfo('SR2: Service call is currently being processed. Please wait...')
      #self.setIcon(self.icons[IconType.running].pixmap(self.icons[IconType.running].availableSizes()[0]))
      self.status_label.setPixmap(self.icons[IconType.running].pixmap(self.icons[IconType.running].availableSizes()[0]))
      self.disabled = True

  @pyqtSlot(bool)
  def block(self, state):
    '''
    Disables button from futher interaction while service is being called (or until timeout occurs)
    '''
    if state:
      #self.status_label.setPixmap(self.icons[IconType.running].pixmap(self.icons[IconType.running].availableSizes()[0]))
      self.status_label.setPixmap(self.icons[IconType.running].pixmap(self.icons[IconType.running].availableSizes()[0]))
      self.status_label.setToolTip('Service call "' + self.args + '" is being processed...')
#      self.message.setText('<nobr>' + self.name + ' : "rosservice call ' + self.args + '"</nobr><br/>Status: Running...')
      self.service_caller.setDisabled(True)
    self.disabled = state

  @pyqtSlot(int, str)
  def reply(self, status, msg):
    '''
    Based on the success status returned by the service call (SUCCESS_TRUE, SUCCESS_FALSE, FAILED)
    the button is enabled and changes its icon while the test of the status label displays information on how the service call went
    '''
    if status in [ServiceRunnable.CallStatus.SUCCESS_TRUE, ServiceRunnable.CallStatus.SUCCESS_FALSE]:
      #self.status_label.setPixmap(self.icons[IconType.inactive].pixmap(self.icons[IconType.inactive].availableSizes()[0]))
      self.status_label.setPixmap(self.icons[IconType.inactive].pixmap(self.icons[IconType.inactive].availableSizes()[0]))
      rospy.loginfo('SR2: Calling service %s from thread %d successful. Service returned status %s with message "%s"', self.args, int(QThread.currentThreadId()), ('True' if not status else 'False'), msg)
      self.reply_statL.setText('Reply status: ' + ('True' if status == ServiceRunnable.CallStatus.SUCCESS_TRUE else 'False'))
      self.reply_msgL.setText('Reply message: ' + msg)
    else:
      self.status_label.setPixmap(self.icons[IconType.error].pixmap(self.icons[IconType.error].availableSizes()[0]))
      rospy.logerr('SR2: Calling service %s from thread %d failed due to "%s"', self.args, int(QThread.currentThreadId()), msg)
      self.reply_statL.setText('Reply status: <font color=\"red\">Error</font>')
      self.reply_msgL.setText('Reply message: <font color=\"red\">' + msg.split('for service', 1)[0] + '</font>')

    self.service_caller.setDisabled(False)
#    self.message.setText('<nobr>' + self.name + ' : "rosservice call ' + self.args + '"</nobr><br/>Status: ' + msg)

  def __init__(self, name, args, timeout):
    super(SR2ViewButtonService, self).__init__()

    _icons = IconType.loadIcons(name)
    self.icons = _icons[0]

    self.args = args
    self.timeout = timeout
    self.name = name
    self.setObjectName(name)

    layout = QVBoxLayout(self)
    layout.setObjectName(name+' Layout Service')

    controls_layout = QHBoxLayout()
    spacer = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
    controls_layout.addItem(spacer)
    self.status_label = QLabel(self)
    self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    self.status_label.setScaledContents(True);
    self.status_label.setToolTip('Service call "' + self.args + '" inactive')
    self.status_label.setPixmap(self.icons[IconType.inactive].pixmap(self.icons[IconType.inactive].availableSizes()[0])) # Convert icon to pixmap: http://stackoverflow.com/a/27057295/1559401
    controls_layout.addWidget(self.status_label)
    self.service_caller = QPushButton('Call', self)
    self.service_caller.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    controls_layout.addWidget(self.service_caller)
    spacer2 = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
    controls_layout.addItem(spacer2)
    layout.addLayout(controls_layout)

    info_layout = QVBoxLayout()
    line = QFrame(self)
    line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    info_layout.addWidget(line)
    serviceL = QLabel('Service: ' + self.args, self)
    serviceL.setWordWrap(True)
    info_layout.addWidget(serviceL)
    timeoutL = QLabel('Timeout: ' + str(timeout) + 's', self)
    timeoutL.setWordWrap(True)
    info_layout.addWidget(timeoutL)
    self.reply_statL = QLabel('Reply status:', self)
    self.reply_statL.setWordWrap(True)
    info_layout.addWidget(self.reply_statL)
    self.reply_msgL = QLabel('Reply message:', self)
    self.reply_msgL.setWordWrap(True)
    info_layout.addWidget(self.reply_msgL)
    layout.addLayout(info_layout)

    self.setWindowTitle('Service "' + self.args + '"')

    self.thread_pool = QThreadPool(self)
    self.service = ServiceRunnable(self.args, self.timeout)
    self.service.setAutoDelete(False)
    self.service.signals.srv_running.connect(self.block)
    self.service.signals.srv_status.connect(self.reply)
    self.service_caller.clicked.connect(self.call)

    self.disabled = False

    self.setLayout(layout)
    self.resize(layout.sizeHint())

  def onResize(self, event):
    # Resize icon of button
    # OR implement custom button and override paintEvent(event)
    self.service_caller.setIconSize(QSize(self.service_caller.width()/2, self.service_caller.height()/2))
    super.onResize(event)

##############################################################################################################################################
######################################################  SR2ToolbarButtonWithView  ############################################################
##############################################################################################################################################
class SR2ButtonWithView(IconToolButton):
  '''
  Part of a toolbar; opens a view
  '''
  class SR2View(QWidget):
    def __init__(self, name, yaml_button_list):
      super(SR2ButtonWithView.SR2View, self).__init__()

      name = name + ' View'
      self.setObjectName(name)
      self.setWindowTitle(name)

      rospy.loginfo('SR2: Creating view for %s', yaml_button_list)

      grid = QGridLayout()
      grid.setMargin(20)
      grid.setContentsMargins(5,5,5,5)

      self.buttons = []
      idx = 0
      for yaml_button_entry in yaml_button_list:
        button = SR2Button.createButton(None, yaml_button_entry, name+str(idx))
        if not button: continue
        self.buttons.append(button)
        idx += 1

      # Get dimensions of grid based on number of VALID buttons after the YAML entries have been parsed (or failed to)
      (self.rows, self.cols) = sr2gg.get_dim(len(self.buttons))
      rospy.loginfo('SR2: View contains %d buttons which will be distributed on a %d by %d (rows by columns) grid layout' % (len(yaml_button_list), self.rows, self.cols))

#      for button in self.buttons:
#        self.grid.addWidget(button)
      positions = []
      for r in range(0, self.rows):
        for c in range(0, self.cols):
          positions.append([r, c])

      idx = 0
      for pos in positions:
        try:
          # There is a maximum of a single cell-gap (for example for 3 buttons we generate a 2x2 grid with 4 cells one of which would remain empty)
          # that needs to be handle that will cause an exception to be thrown
          grid.addWidget(self.buttons[idx], pos[0], pos[1])
          idx += 1
        except:
          break;

      self.setLayout(grid)

    def shutdown(self):
      pass

    def save_settings(self, plugin_settings, instance_settings):
      pass

    def restore_settings(self, plugin_settings, instance_settings):
      pass

  def __init__(self, name, yaml_entry_data, context, surpress_overlays=False, minimal=True):
    # Load icons
    icons = IconType.loadIcons(name, with_view=True)
    super(SR2ButtonWithView, self).__init__(name, icons=icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    self.icons = icons[0]
    self.setStyleSheet('QToolButton {border: none;}')

    try:
      # Buttons contains a list of YAML buttons inside the menu_entry of the view node. This list is used for populating the view with components
      self.yaml_view_buttons = yaml_entry_data
    except YAMLError:
      rospy.logerr('SR2: Detected menu entry with view which does not contain any buttons. Generated view will be empty')

#    self._plugin_settings = None
#    self._instance_settings = None
    self.view_widget = None
    self.minimal = minimal
    self.name = name
    self.toggled = False
    self.setIcon(self.icons[IconType.inactive])
    self.clicked.connect(self.toggleView)
    self.context = context
    self.setToolTip(self.name)
    self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))

    self.close_mutex = QMutex()
    self.show_mutex = QMutex()

  def toggleView(self):
    '''
    Toggles the visibility of the view (if such exists) that is connected to the menu entry
    '''
    # Sadly the way the views in the dashboard work doesn't allow
    # for a view's components to emit feedback to the menu entry
    # since those are destroyed every  time the view is hidden thus
    # only the menu entry remains...OR MAYBE NOT XD

    if not self.yaml_view_buttons: return

    with QMutexLocker(self.show_mutex):
      try:
        if self.toggled:
          # If menu entry is already displaying a view, remove it
          self.toggled = False
          print('Deleted view at %s' % self.view_widget)
          self.context.remove_widget(self.view_widget)
          self.close()
          self.setIcon(self._icons[IconType.inactive])
          rospy.loginfo('SR2: Closed SR2View "%s"', self.name)
        else:
          # If menu entry doesn't display a view, create it and display it
          self.toggled = True
          self.setIcon(self._icons[IconType.running])
          rospy.loginfo('SR2: Added SR2View "%s"', self.name)
          self.view_widget = SR2ButtonWithView.SR2View(self.name, self.yaml_view_buttons)
          print('Created view at %s' % self.view_widget)
          self.context.add_widget(self.view_widget)
      except Exception as e:
        if not self.view_widget:
          self.setIcon(self._icons[IconType.error])
          rospy.logerr('SR2: Error during showing SR2View : %s', e.message)
        self.toggled = False

  def close(self):
    '''
    Unloads the plugin from memory
    '''
    if self.toggled:
      with QMutexLocker(self.close_mutex):
#        if self._plugin_settings:
#          self.view_widget.save_settings(self._plugin_settings,
#                                         self._instance_settings)
        if self.view_widget:
          self.view_widget.shutdown()
          self.view_widget.close()
          self.view_widget = None

  def save_settings(self, plugin_settings, instance_settings):
#    if self.toggled:
#      self.view_widget.save_settings(self._plugin_settings,
#                            self._instance_settings)
    pass
  def restore_settings(self, plugin_settings, instance_settings):
    pass