# Author: Aleksandar Vladimirov Atanasov
# Description: The SR2 Dashboard is used for controlling the SR2 service robot platform and view data about the state of the system

# ROS
import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
# RQT Robot Dashboard
from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
from rqt_robot_dashboard.icon_tool_button import IconToolButton
from .misc.sr2_ros_entry_extraction import SR2PkgCmdExtractor, IconType
# RQT Robot Dashboard widgets
#from rqt_robot_dashboard.widgets import ...

# RQT Plugins
from rqt_pose_view.pose_view_widget import PoseViewWidget

# Python
from yaml import YAMLError

# COB messages
#from cob_msgs.msg import ...

# PyQt
# QtGui modules
from python_qt_binding.QtGui import QStatusBar, QToolBar

# QtCore modules
from python_qt_binding.QtCore import QMutex, QMutexLocker, QSize, pyqtSlot, pyqtSignal

# SR2 widgets
#from widgets.sr2_menu_entry import SR2MenuEntryWidget as sr2me #### OLD VERSION
from widgets.sr2_button import SR2Button as sr2b # NEW VERSION using sr2_button.py


#########################################################################################################
#############################TEMPLATE WIDGETS FOR THE SR2 DASHBOARD######################################
#########################################################################################################
'''
The following code demonstrates how to create the basic widget (menu entry + main view entry).
Note that entries for the dashboard have to have unique object names (see self.setObjectName())
'''
'''
class TestWidget(QWidget):
    def __init__(self):
        super(TestWidget, self).__init__()
        obj_name = 'Test Widget'
        self.setObjectName(obj_name)
        self.setWindowTitle(obj_name)
        layout = QVBoxLayout()
        button = QPushButton('Click me!', self)
        button.clicked.connect(self.print_message)
        layout.addWidget(button)
        self.setLayout(layout)

    def print_message(self):
        rospy.loginfo('Hello!')

    def shutdown(self):
        rospy.loginfo('Bye!')

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

class TestWidgetSR2MenuEntry(IconToolButton):

    def __init__(self, context, name='Test Widget Menu Entry', icons=None, icon_paths=None, minimal=True): # TODO: what is the minimal for?!
        if icons == None:
          icons = []
          icons.append(['emergency_stop/emergency_stop_off.svg'])
          icons.append(['emergency_stop/emergency_stop_on.svg'])

        #icon_paths = (icon_paths if icon_paths else []) + [['rqt_robot_dashboard', 'images']]
        icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]

        super(TestWidgetSR2MenuEntry, self).__init__(name, icons, icon_paths=icon_paths)
        paths = []
        rp = rospkg.RosPack()
        for path in icon_paths:
            paths.append(os.path.join(rp.get_path(path[0]), path[1]))
        self._icon_helper = IconHelper(paths, name)
        converted_icons = self._icon_helper.set_icon_lists(icons)
        self._widget = None
        self._icons = converted_icons[0]
        self._name = name
        self._visible = False
        self.setIcon(self._icons[0])
        self.clicked.connect(self._show)
        self.context = context

        from python_qt_binding.QtCore import QMutex
        self._close_mutex = QMutex()
        self._show_mutex = QMutex()

    def _show(self):
        from python_qt_binding.QtCore import QMutexLocker
        with QMutexLocker(self._show_mutex):
            try:
                if self._visible:
                    rospy.loginfo('Showing TestWidget')
                    self.context.remove_widget(self._widget)
                    self._close()
                    self._visible = False
                    self.setIcon(self._icons[0])
                else:
                    rospy.loginfo('Hiding TestWidget')
                    self._widget = TestWidget()
                    self.context.add_widget(self._widget)
                    rospy.loginfo('Added TestWidget')
                    self._visible = True
                    self.setIcon(self._icons[1])
            except Exception as e:
                rospy.logerr('Error during showing TestWidget : %s', e.message)
                if self._widget == False:
                    rospy.logerr('Dang it!')
                    raise
                self._visible = False
                #self._test_widget_show()

    def _close(self):
        from python_qt_binding.QtCore import QMutexLocker
        with QMutexLocker(self._close_mutex):
            if self.visible:
                self._widget.shutdown()
                self._widget.close()
                self._widget = None
'''
#########################################################################################################
#########################################################################################################
#########################################################################################################

# WORK IN PROGRESS
# Integrate the SR2ButtonWidgetFactory, SR2GridGenerator and SR2LayoutCreator classes here


# TODO
# All buttons have to receive feedback of whether the executed process was successful or not (use kill -0 PID to check every N seconds if roslaunch process is running)

class SR2Dashboard(Dashboard):
  '''
  Contains a CoB/PR2-like dashboard with functionality that is used for controlling and monitoring the SR2 platform
  '''
  __block_override_reset = pyqtSignal(bool) # All entities that support init block override are blocked initially. This signal is emitted if Init entry in YAML config file is not present in order to unblock all entities
  def setup(self, context):
    rospy.loginfo('SR2: Starting dashboard')
    self.name = 'SR2Dashboard'
    self.context = context
    self.yFile = None
    self.widgets = []
    self.status_bar = QStatusBar()
    self.status_bar.showMessage('SR2 status: Welcome to the SR2 Dashboard')
    # TODO Add full support for the statusbar
    self.status_bar.setObjectName('Statusbar')
    self.status_bar.setMaximumHeight(30)
    self.context.add_widget(self.status_bar)
#    self.toolbar2 = QToolBar()
#    self.toolbar2.setObjectName('Toolbar') # The toolbar automatically added to the SR2 Dashboard has the name 'Dashboard'
#    self.context.add_toolbar(self.toolbar2)
    # http://docs.ros.org/jade/api/qt_gui/html/classqt__gui_1_1plugin__context_1_1PluginContext.html
    # Use context.add_widget(...) and context.add_widget(...) to add widgets to dashboard (set objectname parameter to unique value for each!)
    # Use context.remove_widget(...) to remove them

    # Check if configuration is uploaded to the parameter server
    if not rospy.has_param('/sr2_dashboard/dashboard/menus'):
      rospy.logfatal('SR2: Unable to find "/sr2_dashboard/dashboard/menus" on parameter server. Please make sure that you have executed "rosparam load your_config.yaml" before launching the SR2 Dashboard')
      return

    # Retrieve all menus from YAML file
    self._yMenus = rospy.get_param('/sr2_dashboard/dashboard/menus')

    if len(self._yMenus) > 0: rospy.logdebug('SR2: Found %d menus' % len(self._yMenus))
    else:
      rospy.logerr('SR2: No menus found! Dashboard will be empty')
      return

    self._yInit = None
    if not rospy.has_param('/sr2_dashboard/dashboard/init'):
      rospy.logwarn('SR2: Init YAML node not found!')
    else:
      self._yInit = rospy.get_param('/sr2_dashboard/dashboard/init')

    # Populate the self.widgets list
    self.generate_widgets()

  def generate_widgets(self):
    '''
    Populates the SR2Dashboard with widgets using the configuration stored inside the YAML file
    '''
    self.init = None
    # Add default widgets
    if self._yInit:
      self.init = sr2b.createButton(self.context, self._yInit, self._yInit['name'], init=True)
    self.monitor = MonitorDashWidget(self.context)
#    self.console = ConsoleDashWidget(self.context, minimal=False)
    self.pose_view = SR2PoseView('Pose View', self.context, minimal=False)

    if self.init:
      try: self.init.block_override.connect(self.pose_view.block_override)
      except:
        self.__block_override_reset.connect(self.pose_view.block_override)
        self.__block_override_reset.emit(False)
    else:
      self.__block_override_reset.connect(self.pose_view.block_override)
      self.__block_override_reset.emit(False)

#    try: self.widgets.append([self.init, self.monitor, self.console, self.pose_view])
#    except: self.widgets.append([self.monitor, self.console, self.pose_view])
    if self.init: self.widgets.append([self.init, self.monitor, self.pose_view])
    else: self.widgets.append([self.monitor, self.pose_view])

    try:
      # Iterate through all menus
      for menuIdx in range(0,len(self._yMenus)):
        rospy.logdebug('SR2: Found menu with %d entries' % len(self._yMenus[menuIdx]['modules']))
        widget_curr_menu = [] # Contains all SR2MenuEntry objects for the currently parsed menu X

        # Iterate through all modules of the current menu
        for menu_entryIdx in range(0, len(self._yMenus[menuIdx]['modules'])):
          rospy.logdebug('SR2: Found menu entry "%s"' % self._yMenus[menuIdx]['modules'][menu_entryIdx]['name'])

          # Create menu entry
          # OLD VERSION
          #entry = sr2me.createWidget(self.context, self._yMenus[menuIdx]['modules'][menu_entryIdx], self._yMenus[menuIdx]['modules'][menu_entryIdx]['name'])

          # Connect self.init signals to menu entries' slots
          entry = sr2b.createButton(self.context, self._yMenus[menuIdx]['modules'][menu_entryIdx], self._yMenus[menuIdx]['modules'][menu_entryIdx]['name'])
          # Append menu entry to menubar
          if entry:
            if self.init:
              try:
                self.init.block_override.connect(entry.block_override)
              except:
                self.__block_override_reset.connect(entry.block_override)
                self.__block_override_reset.emit(False)
            else:
              try:
                self.__block_override_reset.connect(entry.block_override)
                self.__block_override_reset.emit(False)
              except:
                pass
            widget_curr_menu.append(entry)

        # Append the complete menu X to the list of widgets that are loaded when self.get_widgets() is called
        self.widgets.append(widget_curr_menu)

    except YAMLError as exc:
      rospy.logerr('SR2: error while loading YAML file.\nFull error message: %s', exc)

  def get_widgets(self):
    '''
    Retrieves the widgets that will populate the menubar
    It can contain various QWidgets however it is advised to use IconToolButton, QLabel
    or other predefined or custom simplistic QWidgets in order to keep the menu compact.
    Complex QWidgets are to be displayed in the view of the respective menu entry (if available)
    '''
    return self.widgets

  def shutdown_dashboard(self):
    rospy.loginfo('SR2: Shutting down the SR2 Dashboard')
    # unregister all subscribers
    pass

  # TODO See what these two functions below are used for and how
  def save_settings(self, plugin_settings, instance_settings):
    pass

  def restore_settings(self, plugin_settings, instance_settings):
    pass

# Ported widget from RQT plugins
class SR2PoseView(IconToolButton):
  def __init__(self, name, context, minimal=True):
    icons = IconType.loadIcons(name, with_view=True)
    super(SR2PoseView, self).__init__(name, icons=icons[1], icon_paths=[['sr2_dashboard', 'resources/images']])

    self.context = context
    self.icons = icons[0]
    self.setStyleSheet('QToolButton {border: none;}')

    self.setFixedSize(self.icons[0].actualSize(QSize(50, 30)))
    self.setIcon(self.icons[IconType.inactive])
    self.pose_view = None # Contains the instance of PoseViewWidget
    self.close_mutex = QMutex()
    self.toggled = False

    self.clicked.connect(self.toggleView)

    self.init_block_enabled = True

  @pyqtSlot(bool)
  def block_override(self, block_override_flag):
    self.init_block_enabled = block_override_flag

  def toggleView(self):
    if self.init_block_enabled:
      rospy.logerr('SR2: Init ext.process is not running. Unable to control ext.process connected to this button')
      return

    if self.pose_view is None:
      print('PoseView')
      self.pose_view = PoseViewWidget(None) # Curse Plugin argument for the constructor...
    try:
      if self.toggled:
        rospy.logdebug('PoseView hide')
        self.context.remove_widget(self.pose_view)
        self.close()
        self.toggled = False
      else:
        rospy.logdebug('PoseView show')
        self.context.add_widget(self.pose_view)
        self.toggled = True
    except Exception:
      self.toggled = False
      self.toggleView()

  def close(self):
    if self.toggled:
      with QMutexLocker(self.close_mutex):
        if self.pose_view:
          self.pose_view.shutdown_plugin()
          self.pose_view = None

  def save_settings(self, plugin_settings, instance_settings):
    self.pose_view.save_settings(plugin_settings,
                            instance_settings)
  def restore_settings(self, plugin_settings, instance_settings):
    self.pose_view.restore_settings(plugin_settings, instance_settings)

#  def update_emergency_stop_state_callback(self, state):
#    self._emergencystop.update_state(state.emergency_button_stop)