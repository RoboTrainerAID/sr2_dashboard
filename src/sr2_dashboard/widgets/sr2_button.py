# Author: Aleksandar Vladimirov Atanasov
# Description: A SR2 buttons (for both toolbar and view widgets)

# TODO Finish rewriting all below and integrate with current code base


# YAML
from yaml import YAMLError

# PyQt
# QtGui modules
from python_qt_binding.QtGui import QToolButton, QPushButton, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QSpacerItem, QSizePolicy, QFrame
# QtCore modules
from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer, QThread, pyqtSlot, pyqtSignal, QThreadPool, QSize
#from PyQt4.QtSvg import QSvgRenderer # TODO Add support for SVG for the icons and pixmaps: http://stackoverflow.com/a/35138314/1559401 | http://www.qtcentre.org/threads/7321-Loading-SVG-icons

import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
import rospkg
# ROS-related  modules: RQT Robot Dashboard
from rqt_robot_dashboard.util import IconHelper
from rqt_robot_dashboard.icon_tool_button import IconToolButton
#from rqt_robot_dashboard.widgets import ...

from sr2_monitor_object import SR2Worker, ProcStatus
from sr2_runnable_object import ServiceRunnable
from sr2_view import SR2MenuEntryViewWidget as sr2mev
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

    # Parse the yaml_entry_data and if possible return the respective widget
#    if 'type' in yaml_entry_data and yaml_entry_data['type'] == 'view':
#      # Toolbar entry with view
#        rospy.loginfo('\n----------------------------------\n\tCREATE VIEW\n@Yaml_Contents: %s\n----------------------------------', yaml_entry_data)
#        return SR2ToolbarButtonWithView(name, yaml_entry_data, context)

    pkg = ''
    cmd = ''
    args = ''
    timeout = 0
#    try:
#      # For noview buttons
#      pkg, cmd, args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data['menu_entry'])
#    except KeyError:
#      # For view buttons
#      pkg, cmd, args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data)

    if 'type' in yaml_entry_data:
      # We have an entry that is part of the toolbar
      pkg, cmd, args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data['menu_entry'])
      if yaml_entry_data['type'] == 'noview':
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
#        if 'buttons' not in yaml_entry_data['menu_entry']:
#          rospy.logwarn('SR2: Found view but list of buttons is empty. No toolbar entry and a view associated with it will be created')
#          return None
        return SR2ToolbarButtonWithView(name, yaml_entry_data['menu_entry'], context)
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

#    if 'type' in yaml_entry_data:
#      # We have a toolbar entry
#      if yaml_entry_data['type'] == 'noview':
#        # Toolbar entry without view
#        if timeout > 0:
#          # Toolbar entry without view; service call
#          if not args: return None
#          return SR2ButtonService(name, args, timeout)
#        else:
#          # Toolbar entry without view; external app, rosrun or roslaunch
#          return SR2ButtonExtProcess(name, cmd, pkg, args)
#      else: return None
#    else:
#      rospy.loginfo('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
#      # We have a view entry
#      if timeout > 0:
#        # Service call
#        if not args: return None
#        return SR2ViewButtonService(name, args, timeout)
#      else:
#        # App, roslaunch or rosrun
#        return SR2ViewButtonExtProcess(name, cmd, pkg, args)

##############################################################################################################################################
#########################################################  SR2ButtonExtProcess  ##############################################################
##############################################################################################################################################
class SR2ButtonExtProcess(IconToolButton):
  '''
  Part of a toolbar; launches an external application, provides status feedback while monitoring for that application and the ability to stop it
  '''

  #statusSignal = pyqtSignal(int, str) # status, name
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

    self.active = False # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
    self._status = Status.inactive
    self.setIcon(self.icons[IconType.inactive])

    self.clicked.connect(self.start)
    self.clicked.connect(self.stop)

    self.mutex_recovery = QMutex()
    self.mutex_status = QMutex()

    self.createWorker()

  def createWorker(self):
    '''
    Creates the following components:
      - Separate thread - used for launching and monitoring a detached process
      - SR2 Worker - a QObject which runs inside the thread and is the control entity for the external process
      - Timer - used for triggering a lot inside the SR2 Worker that sends back information about the status of the external process
    '''
    self.worker = None
    if self.pkg: self.worker = SR2Worker(self.cmd, self.pkg, self.args)
    else: self.worker = SR2Worker(cmd=self.cmd, pkg=None, args=self.args)

    self.timer = QTimer()
    self.timer.setInterval(1000)

    self.worker_thread = QThread()

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

    self.worker.status_signal.connect(self.status)
    self.worker.recover_signal.connect(self.recover)

    self.worker_thread.start()

  @pyqtSlot()
  def start(self):
    if not self.active:
      rospy.loginfo('SR2: Attempting to start external process')
      self.active = True
      self._status = Status.running
      self.setIcon(self.icons[IconType.running])
      self.start_signal.emit()

  @pyqtSlot()
  def stop(self):
    '''
    Attempt to stop the external process
    '''
    if self.active:
      rospy.loginfo('SR2: External process stopped')
      self.stop_signal.emit()

  @pyqtSlot(int)
  def status(self, status):
    '''
    Receive status from the worker based on the state of the external process this button represents
    '''
    lock = QMutexLocker(self.mutex_status)

    if status == ProcStatus.RUNNING:
      self._status = Status.running
      self.setIcon(self.icons[IconType.running])
    elif status in [ProcStatus.FINISHED, ProcStatus.INACTIVE]:
      self._status = Status.inactive
      self.setIcon(self.icons[IconType.inactive])
    elif status in [ProcStatus.FAILED_START, ProcStatus.FAILED_STOP]:
      self._status = Status.error
      self.setIcon(self.icons[IconType.error])

    if self._status != Status.running: self.active = False

    #self.status_signal.emit(self._status, self.name) # Status is forwarded to the parent widget (view) if the button is part of a view (TODO or to the statusbar)

  @pyqtSlot(bool)
  def recover(self, recovery_status):
    '''
    Check if button can recover from a previous state before application was closed/has crashed
    '''
    lock = QMutexLocker(self.mutex_recovery)

    self.active = recovery_status


##############################################################################################################################################
#########################################################  SR2ButtonExtProcess  ##############################################################
##############################################################################################################################################
class SR2ViewButtonExtProcess(QWidget):
  '''
  '''
  # TODO
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

    controls_layout = QHBoxLayout()
    spacer = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
    controls_layout.addItem(spacer)

    self.status_label = QLabel(self)
    self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    self.status_label.setScaledContents(True);
    self.status_label.setToolTip('Ext.process "' + self.cmd + ' ' + self.args + '"' + ((' from package "' + self.pkg + '"') if self.pkg else '') + ' inactive')
    self.status_label.setPixmap(self.icons[IconType.inactive].pixmap(self.icons[IconType.inactive].availableSizes()[0])) # Convert icon to pixmap: http://stackoverflow.com/a/27057295/1559401
    controls_layout.addWidget(self.status_label)
    self.execute_button = QPushButton("Execute", self)
    self.execute_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    controls_layout.addWidget(self.execute_button)

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

    self.active = False # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
    self._status = Status.inactive
    #self.setIcon(self.icons[IconType.inactive])

    # TODO Connect button and finish copying the code
#    self.execute_button.connect(self.start)
#    self.execute_button.connect(self.stop)

    self.mutex_recovery = QMutex()
    self.mutex_status = QMutex()

    # createWorker()

    self.setLayout(layout)
    self.resize(layout.sizeHint())

  def onResize(self, event):
    pass

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

    self.thread_pool = QThreadPool(self)
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

    # TODO Fix UI
    _icons = IconType.loadIcons(name)
    self.icons = _icons[0]

    self.args = args
    self.timeout = timeout
    self.name = name
    self.setObjectName(name)

    self.layout = QVBoxLayout(self)
    self.layout.setObjectName(name+'_layout')

    controls_layout = QHBoxLayout()
    spacer = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
    controls_layout.addItem(spacer)

    self.status_label = QLabel(self)
    self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    self.status_label.setScaledContents(True);
    self.status_label.setToolTip('Service call "' + self.args + '" inactive')
    self.status_label.setPixmap(self.icons[IconType.inactive].pixmap(self.icons[IconType.inactive].availableSizes()[0])) # Convert icon to pixmap: http://stackoverflow.com/a/27057295/1559401
    controls_layout.addWidget(self.status_label)
    self.service_caller = QPushButton("Call", self)
    self.service_caller.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    controls_layout.addWidget(self.service_caller)

    controls_layout.addItem(spacer)
    self.layout.addLayout(controls_layout)

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
    self.layout.addLayout(info_layout)
    self.reply_msgL = QLabel('Reply message:', self)
    self.reply_msgL.setWordWrap(True)
    info_layout.addWidget(self.reply_msgL)
    self.layout.addLayout(info_layout)
    self.setWindowTitle('Service "' + self.args + '"')

    self.thread_pool = QThreadPool(self)
    self.service = ServiceRunnable(self.args, self.timeout)
    self.service.setAutoDelete(False)
    self.service.signals.srv_running.connect(self.block)
    self.service.signals.srv_status.connect(self.reply)
    self.service_caller.clicked.connect(self.call)

    self.disabled = False

    self.setLayout(self.layout)

  def onResize(self, event):
    # Resize icon of button
    # OR implement custom button and override paintEvent(event)
    self.service_caller.setIconSize(QSize(self.service_caller.width()/2, self.service_caller.height()/2))
    super.onResize(event)

# TODO Resize status label
#  def onResize(self, event):
#    '''
#    Called whenever a resize event is triggered on the widget. It resizes the icon (if present) inside the button
#    '''
#    if self.icons != None: self.service_caller.setIconSize(QSize(self.service_caller.width()/2, self.service_caller.height()/2))

##############################################################################################################################################
######################################################  SR2ToolbarButtonWithView  ############################################################
##############################################################################################################################################
class SR2ToolbarButtonWithView(IconToolButton):
  '''
  Part of a toolbar; opens a view
  '''
#  class SR2ViewSingleWidget(QWidget):
#    def __init__(self, name, widget, parent = None):
#      super(SR2ToolbarButtonWithView.SR2ViewSingleWidget, self).__init__(parent)
#      name = name + ' View' + (widget.objectName if widget and widget.objectName else '')
#      self.setObjectName(name)
#      self.setWindowTitle(name)
#
#      self.layout = QVBoxLayout(self)
#      widget.setParent(self)
#      self.layout.addWidget(widget)
#      self.setLayout(self.layout)

  class SR2View(QWidget):
    def __init__(self, name, yaml_button_list):
      super(SR2ToolbarButtonWithView.SR2View, self).__init__()

      name = name + ' View'
      self.setObjectName(name)
      self.setWindowTitle(name)

      rospy.loginfo('SR2: Creating view for %s', yaml_button_list)

      self.grid = QGridLayout(self)
      self.grid.setMargin(20)
      self.grid.setContentsMargins(5,5,5,5)

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
          self.grid.addWidget(self.buttons[idx], pos[0], pos[1])
          idx += 1
        except:
          break;

      self.setLayout(self.grid)

    def shutdown(self):
      pass

    def save_settings(self, plugin_settings, instance_settings):
      pass

    def restore_settings(self, plugin_settings, instance_settings):
      pass

  def __init__(self, name, yaml_entry_data, context, surpress_overlays=False, minimal=True):
    # Load icons
    icons = IconType.loadIcons(name, with_view=True)
    super(SR2ToolbarButtonWithView, self).__init__(name, icons=icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

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
          rospy.loginfo('SR2: Closed SR2MenuView "%s"', self.name)
        else:
          # If menu entry doesn't display a view, create it and display it
          self.toggled = True
          self.setIcon(self._icons[IconType.running])
          rospy.loginfo('SR2: Added SR2MenuView "%s"', self.name)
          self.view_widget = SR2ToolbarButtonWithView.SR2View(self.name, self.yaml_view_buttons)
          # FIXME Random crashes occur here whenever reopening the view but not always after the second try! Sometimes it takes multiple open->close operations to crash the whole application
          # FIXME Crash whenever closing the Dashboard and a view is openened!
          # NOTE Crash IS NOT due to the content of the the SR2View! I have replaced the generated buttons with simple QLabels() and the crash still occurs
          print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
          print('Created view at %s' % self.view_widget)
          self.context.add_widget(self.view_widget)
          print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
      except Exception as e:
        if not self.view_widget:
          self.setIcon(self._icons[IconType.error])
          rospy.logerr('SR2: Error during showing SR2MenuView : %s', e.message)
        self.toggled = False

#    with QMutexLocker(self.show_mutex):
#      try:
#        if self.toggled:
#          # If menu entry has a view, remove it
#          rospy.loginfo('SR2: Hiding SR2MenuView "%s"', self.name)
#          self.context.remove_widget(self.view_widget)
##          self.view_widget.close()
#          self.close()
##          self.view_widget = None
#          self.toggled = False
#          self.setIcon(self._icons[IconType.inactive])
#        else:
#          # If menu entry has a view, create it and display it
#          rospy.loginfo('SR2: Showing SR2MenuView "%s View"', self.name)
#          #self.view_widget = sr2mev.createWidget(self.name, self.yaml_view_buttons, self.icons)
#          self.view_widget = SR2ToolbarButtonWithView.SR2View(self.name, self.yaml_view_buttons)
##          if self._plugin_settings:
##            self.view_widget.restore_settings(self._plugin_settings,
##                                              self._instance_settings)
#

#          self.context.add_widget(self.view_widget)
#          rospy.loginfo('SR2: Added SR2MenuView "%s"', self.name)
#          self.toggled = True
#          self.setIcon(self.icons[IconType.running])
#      except Exception as e:
#        if not self.view_widget:
#          self.setIcon(self.icons[IconType.error])
#          rospy.logerr('SR2: Error during showing SR2MenuView : %s', e.message)
#        self.toggled = False

  def close(self):
    '''
    Unloads the plugin from memory
    '''
    if self.toggled:
      with QMutexLocker(self.close_mutex):
#        if self._plugin_settings:
#          self.view_widget.save_settings(self._plugin_settings,
#                                         self._instance_settings)
#        print('CLOSING VIEW')
        if self.view_widget:
#          print('View_widget is present!')
          self.view_widget.shutdown()
          self.view_widget.close()
#          self.context.remove_widget(self.view_widget)
          self.view_widget = None

  def save_settings(self, plugin_settings, instance_settings):
#    if self.toggled:
#      self.view_widget.save_settings(self._plugin_settings,
#                            self._instance_settings)
    pass
  def restore_settings(self, plugin_settings, instance_settings):
    pass