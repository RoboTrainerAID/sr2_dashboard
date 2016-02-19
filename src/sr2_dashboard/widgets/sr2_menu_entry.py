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
from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer, QThread, pyqtSlot, pyqtSignal

# Widgets
from sr2_view import SR2MenuEntryViewWidget as sr2mev
from sr2_monitor_object import SR2Worker, ProcStatus
from ..misc.sr2_ros_entry_extraction import SR2PkgCmdExtractor as sr2pce

class IconType():
  inactive = 0
  running = 1
  error = 2

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
    rospy.loginfo(yamlItemData) # TODO Remove this

    if SR2MenuEntryContext == None or yamlItemData == None: return None
    else:
      # Check if menu entry has a view or not and return repsective instance
      rospy.loginfo('SR2: Menu entry is of type "%s"' % yamlItemData['type'])
      if yamlItemData['type'] == 'view': return SR2MenuEntryWidgetWithView(SR2MenuEntryContext, yamlItemData, SR2MenuEntryName)
      else: return SR2MenuEntryWidgetNoView(SR2MenuEntryContext, yamlItemData, SR2MenuEntryName)

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

    icons = []
    icons_view = []
    icons.append(['control/menu/diagnostics_inactive.png'])        # Inactive view
    icons.append(['control/menu/diagnostics_running.png'])  # Active view
    icons.append(['control/menu/diagnostics_error.png'])
    icons_view.append(['status/status_inactive.svg'])
    icons_view.append(['status/status_running.svg'])
    icons_view.append(['status/status_error.svg'])

    icon_paths = [['sr2_dashboard', 'resources/images']]

    super(SR2MenuEntryWidgetWithView, self).__init__(name, icons, icon_paths=icon_paths)

    # Create paths to icons
    paths = []
    rp = rospkg.RosPack()
    for path in icon_paths:
        paths.append(os.path.join(rp.get_path(path[0]), path[1]))
    self.icon_helper = IconHelper(paths, name)
    converted_icons = self.icon_helper.set_icon_lists(icons)
    self._icons = converted_icons[0]
    converted_icons_view = self.icon_helper.set_icon_lists(icons_view) if icons_view else None
    self._icons_view = converted_icons_view[0] if converted_icons_view else None

    self.view_widget = None
    self.name = name
    self.toggled = False
    self.setIcon(self._icons[IconType.inactive])
    self.clicked.connect(self.toggleDo)
    self.context = context

    self.setToolTip(self.name)

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
          raise
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

class SR2MenuEntryWidgetNoView(IconToolButton):
  '''
  Menu entry without a view which can be placed in the toolbar of the SR2 Dashboard
  It acts the same way as a SR2ButtonWidget
  '''

  startSignal = pyqtSignal()
  stopSignal = pyqtSignal()

  def __init__(self, context, yamlSR2MenuEntry, name='Default name', minimal=True):
    # Load icons
    icons = []
    icons.append(['status/status_inactive.svg'])
    icons.append(['status/status_running.svg'])
    icons.append(['status/status_error.svg'])

    icon_paths = [['sr2_dashboard', 'resources/images']]

    super(SR2MenuEntryWidgetNoView, self).__init__(name, icons, icon_paths=icon_paths)

    # Create paths to icons
    paths = []
    rp = rospkg.RosPack()
    for path in icon_paths:
        paths.append(os.path.join(rp.get_path(path[0]), path[1]))
    self.icon_helper = IconHelper(paths, name)
    converted_icons = self.icon_helper.set_icon_lists(icons)
    self._icons = converted_icons[0]

    self.name = name
    self.toggled = False
    self.setIcon(self._icons[IconType.inactive])
    self.clicked.connect(self.toggleDo) # self.show
    self.context = context

    self.setToolTip(self.name)

    # TODO Add SR2Worker to this menu entry along with all related to it
    # Parse command and arguments and pass these to the constructor of the worker
    #self.worker = SR2Worker(...)
    # Add slots for changing the UI based on feedback from worker
    # Add signals for starting/stoping of worker and triggering status report
    # Add timer, create the connections (slots,signals) etc.

    rospy.loginfo('SR2: Parsing button configuration', yamlSR2MenuEntry)
    self.pkg = ''
    self.cmd = ''   # Can be rosrun/roslaunch/rosservice call
    self.args = []  # Args is the actual ROS node/launch file/etc. we want to start

    #Try each of the possible configurations: node, launch and service
    self.pkg, self.cmd, self.args = sr2pce.getRosPkgCmdData(yamlSR2MenuEntry['menu_entry'])

    rospy.loginfo('SR2: Found "%s %s %s"' % (self.cmd, self.pkg, self.args))

    # The package itself is considered as an argument so we attach it to args
    self.worker = SR2Worker(self.cmd, self.pkg, self.args)

    # Add timer, worker and thread
    self.timer = QTimer()
    self.timer.setInterval(10)
    self.thread = QThread()

    # Delete worker and timer once thread has stopped
    self.thread.finished.connect(self.worker.deleteLater)
    self.thread.finished.connect(self.timer.deleteLater)

    # Connect worker's status signal to the status slot of the menu entry
    self.worker.statusSignal.connect(self.status)
    self.timer.timeout.connect(self.worker.status)
    self.thread.started.connect(self.timer.start)
    self.startSignal.connect(self.worker.start)
    self.stopSignal.connect(self.worker.stop)

    self.worker.moveToThread(self.thread)
    self.timer.moveToThread(self.thread)

    self.thread.start()

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
      if self.toggled:
        rospy.loginfo('SR2: Shutting down external process')
        self.close()
        self.toggled = False
        self.setIcon(self._icons[IconType.inactive])
        self.stopSignal.emit()  # Tell worker to stop external ROS process
      else:
        rospy.loginfo('SR2: Launching external process')
        self.toggled = True
        self.setIcon(self._icons[IconType.running])
        self.startSignal.emit() # Tell worker to start external ROS process
    except Exception as e:
      if not self.view_widget:
        rospy.logerr('SR2: Error during showing SR2MenuView : %s', e.message)
        raise
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