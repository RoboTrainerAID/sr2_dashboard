# Author: Aleksandar Vladimirov Atanasov
# Description: A SR2 buttons (for both toolbar and view widgets)




# TODO Finish rewriting all below and integrate with current code base




# PyQt
# QtGui modules
from python_qt_binding.QtGui import QToolButton, QPushButton, QWidget, QVBoxLayout, QLabel
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
    # Parse the yaml_entry_data and if possible return the respective widget
    return None

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
    if not self.cmd: return

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
      self.startSignal

  @pyqtSlot()
  def stop(self):
    if self.active:
      rospy.loginfo('SR2: External process stopped')
      self.active = False

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
######################################################  SR2ToolbarButtonService  #############################################################
##############################################################################################################################################
class SR2ToolbarButtonService(QPushButton):
  '''
  Part of a toolbar; initiates for a service call and reports back once service has replied or timeout
  '''
  def __init__(self, name, icons, clicked_icons=None, surpress_overlays=False, icon_paths=None):
    super(SR2ToolbarButtonService, self).__init__()

    self.setStyleSheet('QToolButton {border: none;}')


##############################################################################################################################################
######################################################  SR2ToolbarButtonWithView  ############################################################
##############################################################################################################################################
class SR2ToolbarButtonWithView(QToolButton):
  '''
  Part of a toolbar; opens a view
  '''
  def __init__(self, name, icons, clicked_icons=None, surpress_overlays=False, icon_paths=None, minimal=True):
    super(SR2ToolbarButtonWithView, self).__init__()

    self.setStyleSheet('QToolButton {border: none;}')


##############################################################################################################################################
##########################################################  SR2ViewButtonService  ############################################################
##############################################################################################################################################
class SR2ViewButtonService(QWidget):
  '''
  '''
  def __init__(self, name, icons, clicked_icons=None, surpress_overlays=False, icon_paths=None):
    super(SR2ViewButtonService, self).__init__()

    self.setStyleSheet('QToolButton {border: none;}')
    layout = QVBoxLayout(self)
    self.message = QLabel(self)

  def onResize(self, event):
    '''
    Called whenever a resize event is triggered on the widget. It resizes the icon (if present) inside the button
    '''
    if self.icons != None: self.qbtn.setIconSize(QSize(self.qbtn.width()/2, self.qbtn.height()/2))