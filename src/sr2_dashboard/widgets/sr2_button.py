# Author: Aleksandar Vladimirov Atanasov
# Description: A SR2 buttons (for both toolbar and view widgets)




# TODO Finish rewriting all below and integrate with current code base




# PyQt
# QtGui modules
from python_qt_binding.QtGui import QToolButton, QPushButton, QWidget, QVBoxLayout, QHBoxLayout, QLabel
# QtCore modules
from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer, QThread, pyqtSlot, pyqtSignal, QThreadPool, QSize

import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
import rospkg
# ROS-related  modules: RQT Robot Dashboard
from rqt_robot_dashboard.util import IconHelper
#from rqt_robot_dashboard.widgets import ...

from sr2_monitor_object import SR2Worker, ProcStatus
from sr2_runnable_object import ServiceRunnable
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
    pkg, cmd, args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data)

    if not cmd: return None
    if (not cmd and not pkg and not args and not timeout): return None

    if 'type' in yaml_entry_data:
      # We have a toolbar entry
      if yaml_entry_data['type'] == 'view':
        return SR2ToolbarButtonWithView(context, name)
        pass
      elif yaml_entry_data['type'] == 'noview':
        # Toolbar entry without view
        if timeout > 0:
          # Toolbar entry without view; service call
          return SR2ButtonService(context, name, cmd, args, timeout)
        else:
          # Toolbar entry without view; external app, rosrun or roslaunch
          return SR2ButtonExtProcess(context, name, cmd, pkg, args)
          pass
      else: return None
    else:
      # We have a view entry
      if timeout > 0:
        # Service call
        return SR2ViewButtonService(context, name, cmd, args, timeout)
        pass
      else:
        # App, roslaunch or rosrun
        return SR2ViewButtonExtProcess(context, name, cmd, pkg, args)
        pass

##############################################################################################################################################
#########################################################  SR2ButtonExtProcess  ##############################################################
##############################################################################################################################################
class SR2ButtonExtProcess(QPushButton):
  '''
  Part of a toolbar or a view; launches an external application, provides status monitoring for that application and the ability to stop it
  '''

  statusSignal = pyqtSignal(int, str) # status, name
  startSignal = pyqtSignal()
  stopSignal = pyqtSignal()

  def __init__(self, name, converted_icons, cmd, pkg=None, args=None, converted_clicked_icons=None, surpress_overlays=False):
    super(SR2ButtonExtProcess, self).__init__()

    # TODO Replace the constructor with the simpler one taking context, name, cmd, pkg and args as arguments (along with surpress_overlays)
    self.setStyleSheet('QToolButton {border: none;}')
    self.name = name
    self.setObjectName(name)
    self.icons = converted_icons
    self.clicked_icons = converted_clicked_icons

    self.cmd = cmd
    self.args = args
    self.pkg = pkg

    self.active = False # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
    self.status = Status.inactive
    self.setIcon(self.icons[IconType.inactive])

    self.clicked.connect(self.start())
    self.clicked.connect(self.stop())

    self.mutex_recovery = Mutex()
    self.mutex_status = Mutex()

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
      self.status = Status.running
      self.setIcon(self.icons[IconType.running])
      self.startSignal.emit()

  @pyqtSlot()
  def stop(self):
    '''
    Attempt to stop the external process
    '''
    if self.active:
      rospy.loginfo('SR2: External process stopped')
      self.stopSignal.emit()

  @pyqtSlot(int)
  def status(self, status):
    '''
    Receive status from the worker based on the state of the external process this button represents
    '''
    lock = QMutexLocker(self.mutex_status)

    if status == ProcStatus.RUNNING:
      self.status = Status.running
      self.setIcon(self.icons[IconType.running])
    elif status in [ProcStatus.FINISHED, ProcStatus.INACTIVE]:
      self.status = Status.inactive
      self.setIcon(self.icons[IconType.inactive])
    elif status in [ProcStatus.FAILED_START, ProcStatus.FAILED_STOP]:
      self.status = Status.error
      self.setIcon(self.icons[IconType.error])

    if self.status != Status.running: self.active = False

    self.status_signal.emit(self.status, self.name) # Status is forwarded to the parent widget (view) if the button is part of a view (TODO or to the statusbar)

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
  pass

##############################################################################################################################################
######################################################  SR2ToolbarButtonService  #############################################################
##############################################################################################################################################
class SR2ButtonService(QPushButton):
  '''
  Part of a toolbar; initiates a service call and reports back once service has replied or timeout
  '''
  @pyqtSlot()
  def call(self):
    if not self.disabled:
      rospy.loginfo('SR2: Calling service %s from thread %d', self.args, int(QThread.currentThreadId()))
      self.thread_pool.start(self.service)

  @pyqtSlot(bool)
  def block(self, state):
    if state: self.setIcon(self._icons[IconType.running])
    self.disabled = state

  @pyqtSlot(int, str)
  def reply(self, status, msg):
    if status in [ServiceRunnable.CallStatus.SUCCESS_TRUE, ServiceRunnable.CallStatus.SUCCESS_FALSE]:
      self.setIcon(self._icons[IconType.inactive])
      rospy.loginfo('SR2: Calling service %s from thread %d successful. Service returned status %s with message "%s"', self.args, int(QThread.currentThreadId()), ('True' if not status else 'False'), msg)
    else:
      self.setIcon(self._icons[IconType.error])
      rospy.logerr('SR2: Calling service %s from thread %d failed due to "%s"', self.args, int(QThread.currentThreadId()), msg)

    self.tooltip = '<nobr>' + self.name + ' : "' + self.cmd + ' ' + self.args + '"</nobr><br/>Reply: ' + msg

  def __init__(self, context, yamlSR2MenuEntry, name='Default name'):
    self.setStyleSheet('QToolButton {border: none;}')

    # Load icons
    icons = IconType.loadIcons(name, True)
    self.icons = icons[0]
    super(SR2ButtonService, self).__init__(name, icons=icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    self.name = name
    self.context = context

    rospy.loginfo('SR2: Parsing button configuration', yamlSR2MenuEntry)
    self.cmd = ''   # Can only be rosservice call
    self.args = ''  # Args contains

    #Try each of the possible configurations: node, launch and service
    self.cmd, self.args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yamlSR2MenuEntry['menu_entry'])[1:] # Package is empty so we can exclude it
    #self.args = '/' + self.args # Example: rosservice call /trigger_srv
    rospy.loginfo('SR2: Found "%s /%s"' % (self.cmd, self.args))

    self.setIcon(self._icons[IconType.inactive])
    self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))
    self.tooltip = self.name + ' : "' + self.cmd + ' ' + self.args + '"<br/>Reply: --'
    self.setToolTip(self.tooltip)

    self.thread_pool = QThreadPool(self)
    self.service = ServiceRunnable(self.args, timeout)
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
      rospy.loginfo('SR2: Calling service %s from thread %d', self.args, int(QThread.currentThreadId()))
      self.thread_pool.start(self.service)

  @pyqtSlot(bool)
  def block(self, state):
    '''
    Disables button from futher interaction while service is being called (or until timeout occurs)
    '''
    if state:
      self.status_label.setPixmap(self._icons[IconType.running])
      self.service_caller.setText('Waiting for reply...')
      self.service_caller.setDisabled(True)
    self.disabled = state

  @pyqtSlot(int, str)
  def reply(self, status, msg):
    '''
    Based on the success status returned by the service call (SUCCESS_TRUE, SUCCESS_FALSE, FAILED)
    the button is enabled and changes its icon while the test of the status label displays information on how the service call went
    '''
    if status in [ServiceRunnable.CallStatus.SUCCESS_TRUE, ServiceRunnable.CallStatus.SUCCESS_FALSE]:
      self.status_label.setPixmap(self._icons[IconType.inactive])
      rospy.loginfo('SR2: Calling service %s from thread %d successful. Service returned status %s with message "%s"', self.args, int(QThread.currentThreadId()), ('True' if not status else 'False'), msg)
    else:
      self.status_label.setPixmap(self._icons[IconType.error])
      rospy.logerr('SR2: Calling service %s from thread %d failed due to "%s"', self.args, int(QThread.currentThreadId()), msg)

    self.service_caller.setDisabled(False)
    self.message.setText('<nobr>' + self.name + ' : "' + self.cmd + ' ' + self.args + '"</nobr><br/>Reply: ' + msg)

  def __init__(self, context, yamlSR2MenuEntry, name='Default name'):
    super(SR2ViewButtonService, self).__init__()

    self.cmd, self.args, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yamlSR2MenuEntry['menu_entry'])[1:] # Package is empty so we can exclude it
    if not(self.cmd and self.args):
      rospy.logerr('SR2: Service widget will be empty due to error in parsing')
      return

    layout = QVBoxLayout(self)
    layout_status_and_call = QHBoxLayout(self)

    self.status_label = QLabel(self)
    self.service_caller = QPushButton(self)
    layout_status_and_call.addWidget(self.status_label)
    layout_status_and_call.addWidget(self.service_caller)
    layout.addWidget(layout_status_and_call)

    self.message = QLabel(self)
    self.message.setText('---')
    self.message.setWordWrap(True)
    layout.addWidget(self.message)

    self.service_caller.setText('Call "' + self.args + '"')
    self.setWindowTitle('Service "' + self.args + '"')

    self.thread_pool = QThreadPool(self)
    self.service = ServiceRunnable(self.args, timeout)
    self.service.setAutoDelete(False)
    self.service.signals.srv_running.connect(self.block)
    self.service.signals.srv_status.connect(self.reply)
    self.service_caller.clicked.connect(self.call)

    self.disabled = False

    self.setLayout(layout)

  def onResize(self, event):
    '''
    Called whenever a resize event is triggered on the widget. It resizes the icon (if present) inside the button
    '''
    if self.icons != None: self.qbtn.setIconSize(QSize(self.qbtn.width()/2, self.qbtn.height()/2))

##############################################################################################################################################
######################################################  SR2ToolbarButtonWithView  ############################################################
##############################################################################################################################################
class SR2ToolbarButtonWithView(QToolButton):
  '''
  Part of a toolbar; opens a view
  '''
  def __init__(self, name, surpress_overlays=False, minimal=True):
    super(SR2ToolbarButtonWithView, self).__init__()

    self.setStyleSheet('QToolButton {border: none;}')