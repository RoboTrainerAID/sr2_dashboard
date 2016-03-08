# Author: Aleksandar Vladimirov Atanasov
# Description: A SR2 menu entry is used to trigger a process or represent a module (a view of multiple controllable processes that is shown/hidden whenever the entry is interacted with)

# ROS-related  modules
import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
import rospkg
# ROS-related  modules: RQT Robot Dashboard
from rqt_robot_dashboard.icon_tool_button import IconToolButton
from rqt_robot_dashboard.util import IconHelper
#from rqt_robot_dashboard.widgets import ...

#import sys
import os
from yaml import YAMLError

# COB messages
#from cob_msgs.msg import ...

# PyQt
# QtGui modules
#from python_qt_binding.QtGui import ...
# QtCore modules
from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer, QThread, pyqtSlot, pyqtSignal, QThreadPool, QSize

# Widgets
from sr2_view import SR2MenuEntryViewWidget as sr2mev
from sr2_monitor_object import SR2Worker, ProcStatus
from sr2_runnable_object import ServiceRunnable
from ..misc.sr2_ros_entry_extraction import SR2PkgCmdExtractor as sr2pce

class IconType():
  inactive = 0
  running = 1
  error = 2

  @staticmethod
  def loadIcons(name, with_view=False):
    '''
    Loads predefined icons
    '''
    # Create paths
    icon_paths = [['sr2_dashboard', 'resources/images']]
    paths = []
    rp = rospkg.RosPack()
    for path in icon_paths:
        paths.append(os.path.join(rp.get_path(path[0]), path[1]))

    icon_helper = IconHelper(paths, name)

    icons = []
    icons_view = []
    res_icons = None
    converted_icons = []

    # Add icons
    if with_view:
      icons_view.append(['status/status_inactive.svg'])         # Inactive
      icons_view.append(['status/status_running.svg'])          # Active
      icons_view.append(['status/status_error.svg'])            # Failed
    else:
      icons.append(['control/menu/diagnostics_inactive.png'])   # Inactive view
      icons.append(['control/menu/diagnostics_running.png'])    # Active view
      icons.append(['control/menu/diagnostics_error.png'])      # Failed

    res_icons = list(icons_view if with_view else icons)
    converted_icons = icon_helper.set_icon_lists(icons_view if with_view else icons)

    return (converted_icons[0], res_icons)

class SR2MenuEntryWidget():

  @staticmethod
  def createWidget(SR2MenuEntryContext, yamlItemData, SR2MenuEntryName):
    '''
    Creates a SR2MenuEntryWidget or SR2MenuEntryWidgetWithView object depending on whether menu entry has a view or not attached to it
    :param SR2MenuEntryContext: context (currently unknown what this contains)
    :param yamlItemData: YAML node containing the data for the menu entry and its view (if present) in the form of a dictionary
    :param override_icons: if set to true default icons will be used (if available!) inside "$(find sr2_dashboard)/resources/images"
    :returns: SR2MenuEntryWidget or SR2MenuEntryWidgetWithView which can be added to a Dashboard
    '''
#    rospy.loginfo(yamlItemData) # TODO Remove this

    if SR2MenuEntryContext == None or yamlItemData == None: return None
    else:
      # Check if menu entry has a view or not and return repsective instance
      rospy.loginfo('SR2: Menu entry is of type "%s"' % yamlItemData['type'])
      if yamlItemData['type'] == 'view':
        return SR2MenuEntryWidgetWithView(SR2MenuEntryContext, yamlItemData, SR2MenuEntryName)
      elif yamlItemData['type'] == 'noview':
        if 'menu_entry' in yamlItemData:
          if 'service' not in yamlItemData['menu_entry']: return SR2MenuEntryWidgetNoView(SR2MenuEntryContext, yamlItemData, SR2MenuEntryName)
          else: return SR2MenuEntryWidgetNoViewService(SR2MenuEntryContext, yamlItemData, SR2MenuEntryName)
        return None
      return None

class SR2MenuEntryWidgetWithView(IconToolButton):
  '''
  Menu entry with a view which can be placed in the toolbar of the SR2 Dashboard.
  The view is a SR2MenuViewWidget, which is attached to the main view of the SR2 Dashboard
  '''
  def __init__(self, context, yamlSR2MenuEntry, name='Default name', minimal=True):
    self.withView = True if yamlSR2MenuEntry['type'] == 'view' else False
    try:
      self.buttons = yamlSR2MenuEntry['menu_entry']['buttons']
    except YAMLError:
      rospy.logwarn('SR2: Detected menu entry with view which does NOT contain any buttons. An empty view will be created instead')
      return None

    # Load icons
    icons = IconType.loadIcons(name, True)
    self.icons = icons[0]
    super(SR2MenuEntryWidgetWithView, self).__init__(name, icons=icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    self.view_widget = None
    self.name = name
    self.toggled = False
    self.setIcon(self._icons[IconType.inactive])
    self.clicked.connect(self.toggleDo)
    self.context = context

    self.setToolTip(self.name)
    self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))

    self.close_mutex = QMutex()
    self.show_mutex = QMutex()

  def toggleDo(self):
    '''
    Toggles the visibility of the view (if such exists) that is connected to the menu entry
    '''
    # Sadly the way the views in the dashboard work doesn't allow
    # for a view's components to emit feedback to the menu entry
    # since those are destroyed every  time the view is hidden thus
    # only the menu entry remains
    with QMutexLocker(self.show_mutex):
      try:
        if self.toggled:
          # If menu entry has a view, remove it
          rospy.loginfo('SR2: Hiding SR2MenuView "%s"', self.name)
          self.context.remove_widget(self.view_widget)
          self.close()
          self.toggled = False
          self.setIcon(self._icons[IconType.inactive])
        else:
          # If menu entry has a view, create it and display it
          rospy.loginfo('SR2: Showing SR2MenuView "%s View"', self.name)
          self.view_widget = sr2mev.createWidget(self.buttons, self.name, self._icons_view)
          self.context.add_widget(self.view_widget)
          rospy.loginfo('SR2: Added SR2MenuView "%s"', self.name)
          self.toggled = True
          self.setIcon(self._icons[IconType.running])
      except Exception as e:
        if not self.view_widget:
          rospy.logerr('SR2: Error during showing SR2MenuView : %s', e.message)
        self.toggled = False

  def close(self):
    '''
    Unloads the plugin from memory
    '''
    with QMutexLocker(self.close_mutex):
      if self.toggled:
        self.view_widget.shutdown()
        self.view_widget.close()
        self.view_widget = None

class SR2MenuEntryWidgetNoViewService(IconToolButton):
  '''
  Menu entry without a view which can be places in the toolbar of the SR2 Dashboard
  It acts the same way as a SR2ButtonWidget however stopping the triggered action is
  not possible. The action that is being executed is a ROS service call (Trigger.srv)
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

  def __init__(self, context, yamlSR2MenuEntry, name='Default name', minimal=True):
    # Load icons
    icons = IconType.loadIcons(name, True)
    self.icons = icons[0]
    super(SR2MenuEntryWidgetNoViewService, self).__init__(name, icons=icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    self.name = name
    self.setIcon(self._icons[IconType.inactive])
    self.context = context

    rospy.loginfo('SR2: Parsing button configuration', yamlSR2MenuEntry)
    self.cmd = ''   # Can only be rosservice call
    self.args = ''  # Args contains

    #Try each of the possible configurations: node, launch and service
    self.cmd, self.args, timeout = sr2pce.getRosPkgCmdData(yamlSR2MenuEntry['menu_entry'])[1:] # Package is empty so we can exclude it
    #self.args = '/' + self.args # Example: rosservice call /trigger_srv
    rospy.loginfo('SR2: Found "%s /%s"' % (self.cmd, self.args))

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

class SR2MenuEntryWidgetNoView(IconToolButton):
  '''
  Menu entry without a view which can be placed in the toolbar of the SR2 Dashboard
  It acts the same way as a SR2ButtonWidget
  '''

  checkRecoverySignal = pyqtSignal()
  startSignal = pyqtSignal()
  stopSignal = pyqtSignal()

  def __init__(self, context, yamlSR2MenuEntry, name='Default name', minimal=True):
    # Load icons
    icons = IconType.loadIcons(name, True)
    self.icons = icons[0]
    super(SR2MenuEntryWidgetNoView, self).__init__(name, icons=icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    self.name = name
    self.toggled = False
    self.setIcon(self._icons[IconType.inactive])
    self.clicked.connect(self.toggleDo) # self.show
    self.context = context

    rospy.loginfo('SR2: Parsing button configuration', yamlSR2MenuEntry)
    self.pkg = ''
    self.cmd = ''   # Can be rosrun/roslaunch/rosservice call
    self.args = ''  # Args is the actual standalone app/ ROS node/ ROS launch file/etc. we want to start

    #Try each of the possible configurations: node, launch and service
    self.pkg, self.cmd, self.args = sr2pce.getRosPkgCmdData(yamlSR2MenuEntry['menu_entry'])[:-1]
    rospy.loginfo('SR2: Found "%s %s %s"' % (self.cmd, self.pkg, self.args))

    self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))
    self.setToolTip(self.name + ' : "' + self.cmd + ' ' + self.pkg + ' ' + self.args + '"')

    # The package itself is considered as an argument so we attach it to args
    self.worker = SR2Worker(self.cmd, self.pkg, self.args)

    # Add timer, worker and thread
    self.timer = QTimer()
    self.timer.setInterval(1000)
    self.thread = QThread()

    # Delete worker and timer once thread has stopped
    self.thread.finished.connect(self.worker.deleteLater)
    self.thread.finished.connect(self.timer.deleteLater)

    # Connect worker's status signal to the status slot of the menu entry
    self.worker.statusSignal.connect(self.status)
    self.worker.recoverySignal.connect(self.recover)
    self.timer.timeout.connect(self.worker.status)
    self.thread.started.connect(self.timer.start)
    self.startSignal.connect(self.worker.start)
    self.stopSignal.connect(self.worker.stop)

    self.worker.moveToThread(self.thread)
    self.timer.moveToThread(self.thread)

    self.thread.start()

    self.status = ProcStatus.INACTIVE
    self.recovered = False
    self.checkRecoverySignal.connect(self.worker.checkRecoveryState)
    self.checkRecoverySignal.emit()

  def __del__(self):
    # Stop thread
    self.thread.quit()
    # Wait until thread has really stopped
    while not self.thread.isFinished(): pass

  def toggleDo(self):
    '''
    Toggles the visibility of the view (if such exists) that is connected to the menu entry
    '''
    # Sadly the way the views in the dashboard work doesn't allow
    # for a view's components to emit feedback to the menu entry
    # since those are destroyed every  time the view is hidden thus
    # only the menu entry remains
    try:
      if self.toggled and not self.recovered and self.status == ProcStatus.RUNNING:
        rospy.loginfo('SR2: Shutting down external process')
        self.close()
        self.toggled = False
        self.setIcon(self._icons[IconType.inactive])
        self.stopSignal.emit()  # Tell worker to stop external ROS process
      elif self.status != ProcStatus.RUNNING:
        self.recovered = False
        rospy.loginfo('SR2: Launching external process')
        self.toggled = True
        self.setIcon(self._icons[IconType.running])
        self.startSignal.emit() # Tell worker to start external ROS process
    except Exception as e:
      if not self.view_widget:
        rospy.logerr('SR2: Error during showing SR2MenuView : %s', e.message)
      self.toggled = False

  def close(self):
    '''
    Unloads the plugin from memory
    '''
    pass

  @pyqtSlot(int)
  def status(self, status):
    if status in [ProcStatus.INACTIVE, ProcStatus.FINISHED]: self.setIcon(self._icons[IconType.inactive])
    elif status in [ProcStatus.FAILED_START, ProcStatus.FAILED_STOP]: self.setIcon(self._icons[IconType.error])
    else: self.setIcon(self._icons[IconType.running])

    self.status = status

  @pyqtSlot(int)
  def recover(self, status):
    if status == ProcStatus.RUNNING:
      self.setIcon(self._icons[IconType.running])
      self.status = status
      #self.toggled = False
      self.recovered = True