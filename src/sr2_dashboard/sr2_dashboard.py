# Author: Aleksandar Vladimirov Atanasov
# Description: The SR2 Dashboard is used for controlling the SR2 service robot platform and view data about the state of the system

# ROS
import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
# RQT Robot Dashboard
from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
# RQT Robot Dashboard widgets
#from rqt_robot_dashboard.widgets import ...

# Python
from yaml import YAMLError

# COB messages
#from cob_msgs.msg import ...

# PyQt
# QtGui modules
#from python_qt_binding.QtGui import ...

# QtCore modules
#from python_qt_binding.QtCore import ...

# SR2 widgets
from widgets.sr2_menu_entry import SR2MenuEntryWidget as sr2me

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




# STRUCTURE
#class SR2DashboardItemFactory:
#
#  class SR2MenuEntryWidget(IconToolButton):
#    '''
#    Part of a toolbar in the SR2 Dashboard
#    There are two types of SR2MenuEntryWidgets:
#      - without SR2MenuViewWidget - this type acts in the same way as SR2ButtonWidget
#      - with SR2MenuViewWidget - here a SR2MenuViewWidget is attached
#    '''
#    def __init__(self, yaml_menu_entry, override_icons, module_name):
#
#      pass
#
#
#  class SR2MenuViewWidget(QWidget):
#    '''
#    Shows/hides a collection of SR2ButtonWidgets whenever a corresponding SR2MenuEntryWidget is clicked
#    It is meant to provide a module of SR2ButtonWidgets with similar functionality
#    '''
#    pass
#
#  class SR2ButtonWidget(QWidget):
#    '''
#    Contains a single button for starting/stopping an external process (roslaunch, rosrun or rosservice call (only for Trigger.srv!))
#    It is part of a SR2MenuViewWidget
#    '''
#    pass
#
#  @staticmethod
#  def createMenuEntry(context, yaml_menu_entry, module_name, override_icons=False):
#    return SR2DashboardItemFactory.SR2MenuEntry(context, yaml_menu_entry, override_icons, module_name) if yaml_menu_entry else None

class SR2Dashboard(Dashboard):
  '''
  Contains a CoB/PR2-like dashboard with functionality that is used for controlling and monitoring the SR2 platform
  '''
  def setup(self, context):
    rospy.loginfo('SR2: Starting dashboard')
    self.name = 'SR2Dashboard'
    self.context = context
    self.yFile = None
    self.widgets = []

    # Check if configuration is uploaded to the parameter server
    if not rospy.has_param('/sr2_dashboard/dashboard/menus'):
      rospy.logfatal('SR2: Unable to find "/sr2_dashboard/dashboard/menus" on parameter server. Please make sure that you have executed "rosparam load your_config.yaml" before launching the SR2 Dashboard')
      return

    # Retrieve all menus from YAML file
    self._yMenus = rospy.get_param('/sr2_dashboard/dashboard/menus')


    if len(self._yMenus) > 0: rospy.loginfo('SR2: Found %d menus' % len(self._yMenus))
    else:
      rospy.loginfo('SR2: No menus found! Dashboard will be empty')
      return

    # Populate the self.widgets list
    self.generate_widgets()

  def generate_widgets(self):
    '''
    Populates the SR2Dashboard with widgets using the configuration stored inside the YAML file
    '''
    # Add default widgets
    # TODO Find a way to convert rqt_graph, rqt_tf_tree to QWidget
    self.monitor = MonitorDashWidget(self.context)
    self.console = ConsoleDashWidget(self.context, minimal=False)
    self.widgets.append([self.monitor, self.console])
    try:
      # Iterate through all menus
      for menuIdx in range(0,len(self._yMenus)):
        rospy.loginfo('SR2: Found menu with %d entries' % len(self._yMenus[menuIdx]['modules']))
        widget_curr_menu = [] # Contains all SR2MenuEntry objects for the currently parsed menu X

        # Iterate through all modules of the current menu
        for menu_entryIdx in range(0, len(self._yMenus[menuIdx]['modules'])):
          rospy.loginfo('SR2: Found menu entry "%s"' % self._yMenus[menuIdx]['modules'][menu_entryIdx]['name'])

          # Create menu entry
          entry = sr2me.createWidget(self.context, self._yMenus[menuIdx]['modules'][menu_entryIdx], self._yMenus[menuIdx]['modules'][menu_entryIdx]['name'])
          # Append menu entry to menubar
          if entry: widget_curr_menu.append(entry)

        # Append the complete menu X to the list of widgets that are loaded when self.get_widgets() is called
        self.widgets.append(widget_curr_menu)

    except YAMLError as exc:
      rospy.loginfo('SR2: error while loading YAML file.')
      rospy.loginfo('SR2: full message: \n"%s"' % exc)

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

#  def update_emergency_stop_state_callback(self, state):
#    self._emergencystop.update_state(state.emergency_button_stop)