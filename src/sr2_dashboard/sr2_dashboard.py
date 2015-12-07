# Author: Aleksandar Vladimirov Atanasov
# Description: The SR2 Dashboard is used for controlling the SR2 service robot platform and view data about the state of the system

import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
import os    
import rospkg

# COB messages
# ...
#from cob_msgs.msg import ...

# RQT Robot Dashboard
from rqt_robot_dashboard.dashboard import Dashboard
from icon_tool_button import IconToolButton
from rqt_robot_dashboard.util import IconHelper
#from rqt_robot_dashboard.widgets import ...

# PyQt
#from python_qt_binding.QtCore import ...
from PyQt4.QtGui import QWidget, QPushButton, QVBoxLayout

# Widgets
# ...
#from .sr2_emergency_stop_widget import SR2EmergencyStopWidget
#from sr2_display_main_view import SR2MainViewWidget
#from yaml_gui import SR2LayoutCreator

# TODO:
# - Add the yaml_gui.py properly to the ROS infrastructure. Right now there is an issue with the paths,
#   since they need to be resolved by rospkg.RosPack, which is not included in the yaml_gui.py

import yaml_gui as yg

class SR2ControlMenu(IconToolButton):
    def __init__(self, context, name='SR2 Control Menu', icons=None, icon_paths=None, minimal=True):
        if icons == None:
          icons = []
          icons.append(['control/menu/diagnostics.png'])
          icons.append(['control/menu/diagnostics-click.png'])
        icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]
        
        super(SR2ControlMenu, self).__init__('SR2 Control Menu', icons, icon_paths=icon_paths)
        paths = []
        self.rp = rospkg.RosPack()
        for path in icon_paths:
            paths.append(os.path.join(self.rp.get_path(path[0]), path[1]))
        self._icon_helper = IconHelper(paths, name)
        converted_icons = self._icon_helper.set_icon_lists(icons)
        self._control_view = None
        self._icons = converted_icons[0]
        self._name = name
        self._control_view_visible = False
        self.setIcon(self._icons[0])
        self.clicked.connect(self._control_view_show)
        self.context = context

        from python_qt_binding.QtCore import QMutex
        self._close_mutex = QMutex()
        self._show_mutex = QMutex()
        
    def _control_view_show(self):
        from python_qt_binding.QtCore import QMutexLocker
        from yaml import YAMLError
        with QMutexLocker(self._show_mutex):
            try:
                if self._control_view_visible:
                    rospy.loginfo('Activate SR2 Control View')
                    self.context.remove_widget(self._control_view)
                    self._control_view_close()
                    self._control_view_visible = False
                    self.setIcon(self._icons[0])
                else:
                    rospy.loginfo('Deactivate SR2 Control View')
                    #self._control_view = SR2ControlView(self.rp.get_path('sr2_dashboard') + '/resources/config/config1.yaml', 'Launches')
                    self._control_view = SR2ControlView(self.rp.get_path('sr2_dashboard') + '/resources/config/configDenis.yaml', 'Launches')
a                   #self._control_view = SR2ControlView(self.rp.get_path('sr2_dashboard') + '/resources/config/config1.yaml', 'Services')
                    self.context.add_widget(self._control_view)
                    rospy.loginfo('Added SR2 Control View to SR2 Dashboard')
                    self._control_view_visible = True
                    self.setIcon(self._icons[1])
            except (KeyError, YAMLError, Exception, IOError, yg.MissingComponents, yg.SR2YamlLoaderFailureException, yg.UnknownModuleException) as e:
                rospy.logerr('Error during showing SR2 Control View : %s', e.message)
                if self._control_view == False:
                    raise
                self._control_view_visible = False
                ##self._control_view_show()
            
    def _control_view_close(self):
        from python_qt_binding.QtCore import QMutexLocker
        with QMutexLocker(self._close_mutex):
            if self._control_view_visible:
                self._control_view.shutdown()
                self._control_view.close()
                self._control_view = None


class SR2ControlView(QWidget):
    
    def __init__(self, yamlConfigFile, moduleName):
        super(SR2ControlView, self).__init__()
        obj_name = 'SR2 Control View ' + moduleName
        self.setObjectName(obj_name)
        self.setWindowTitle(obj_name)
        """layout = QVBoxLayout()
        button = QPushButton('Do something meaningful', self)
        button.clicked.connect(self.print_message)
        layout.addWidget(button)
        self.setLayout(layout)"""
        try:
            self.sr2yl = yg.SR2YamlLoader(yamlConfigFile)
        except IOError as e:
            raise e
        rospy.loginfo('Found YAML configuration file')

        if self.sr2yl == None:
            raise yg.SR2YamlLoaderFailureException('SR2YamlLoader failed to load the given configuration file')
        rospy.loginfo('Opening YAML configuration file completed')
        # Parse the configuration file
        self.sr2yl.parse_config_file()
        rospy.loginfo('Parsing YAML configuration file completed')
        
        # Extract the defined components from the configuration file
        self.components = self.sr2yl.getComponents()
        if self.components == []:
            raise yg.MissingComponents('No components found in the given configuration file')
        rospy.loginfo('Populating components structure completed')
            
        if moduleName not in self.components[1]:
            firstModule = True
            availableModules = ' '
            for module in self.components[1]:
                if firstModule:
                    firstModule = False
                    availableModules = availableModules + module
                else:
                    availableModules = availableModules + ', ' + module
                
            raise yg.UnknownModuleException('Unknown module \"' + moduleName + '\". Choose one of the following:' + availableModules)
            
        rospy.loginfo('Found given module in YAML configuration file')
        # In case we have multiple (>1) ButtonWidgets a "Stop all" button is added for terminating all processes started via those ButtonWidgets
        if len(self.components[1][moduleName]) > 1:
            (self.rows, self.cols) = yg.SR2GridGenerator.get_dim(len(self.components[1][moduleName])+1) # Add button "Stop all" at the end
        else:
            (self.rows, self.cols) = yg.SR2GridGenerator.get_dim(len(self.components[1][moduleName]))
        positions = []
        for r in range(0, self.rows):
            for c in range(0, self.cols):
                positions.append([r, c])
                
        from PyQt4.QtGui import QGridLayout
        self.grid = QGridLayout(self)
        self.grid.setMargin(20)
        self.grid.setContentsMargins(5,5,5,5)
        self.setLayout(self.grid)
        rospy.loginfo('Generating grid completed')
        
        componentIdx = 0
        for (buttonWidget, position) in (zip(self.components[1][moduleName], positions[:-1]) if len(self.components[1][moduleName]) > 1 else zip(self.components[1][moduleName], positions)):
            if componentIdx < len(self.components[1][moduleName]):
                rospy.loginfo('Adding button \"%s\" at [row: %s |col: %s]', buttonWidget['caption'], str(position[0]), str(position[1]))
                # caption, rospkg, roscommand, target, icon
                bW = yg.SR2ButtonWidgetFactory.createButtonWidget(buttonWidget)
                if bW != None:
                    self.grid.addWidget(bW, position[0], position[1])
        
        # FIXME: Currently the "STOP ALL" button is not working properly
        if len(self.components[1][moduleName]) > 1:
            rospy.loginfo('Adding "Stop all" button')
            def stopAll(self, layout):
                #from PyQt4.QtCore import QObject, pyqtSignal
                #class stopEmitter(QObject):
                #    stopSig = pyqtSignal()
                #self.emitter = stopEmitter()
                
                widgets = (layout.itemAt(i).widget() for i in range(layout.count()))
                for widget in widgets:
                    if isinstance(widget, yg.SR2ButtonWidgetFactory.ButtonWidget):
                        rospy.loginfo('Trying to terminate process started by "' + widget.caption + '"')
                        #widget.toggleProcess(False) # terminate the process bound to the current widget if a PID file is detected
                        widget.broadcastedStop()
                        # FIXME: for some reason the button behaviour cannot be controlled decently from here
                        # The toggling works partially - caption changes, process (if PID file present) is terminated but the ButtonWidget's state actually remains checked
                        # Note: signals are a bad idea here since specifying the receiver is against the concept of signals and slots in Qt...I think :D
            stopAllButton = QPushButton('Stop all (NOT WORKING)')
            from PyQt4.QtGui import QSizePolicy
            stopAllButton.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            stopAllButton.clicked.connect(lambda: stopAll(self, self.grid))
            self.grid.addWidget(stopAllButton, positions[-1][0], positions[-1][1])
            
        
        rospy.loginfo('Adding components to grid completed')
    
    def shutdown(self):
        rospy.loginfo('Closing SR2 Control View.')
        rospy.loginfo('Note: started processes will still remain active even when this widget is closed.\n' +
                      'In order to terminate those re-open the SR2 Control View and terminate them one by\n' +
                      'one or using the "Stop all" button')
        
        
    def save_settings(self, plugin_settings, instance_settings):
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        pass
    
#########################################################################################################
#############################TEMPLATE WIDGETS FOR THE SR2 DASHBOARD######################################
#########################################################################################################
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

class TestWidgetMenuEntry(IconToolButton):
    
    def __init__(self, context, name='Test Widget Menu Entry', icons=None, icon_paths=None, minimal=True): # TODO: what is the minimal for?!
        if icons == None:
          icons = []
          icons.append(['emergency_stop/emergency_stop_off.svg'])
          icons.append(['emergency_stop/emergency_stop_on.svg'])
        
        #icon_paths = (icon_paths if icon_paths else []) + [['rqt_robot_dashboard', 'images']]
        icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]
        
        super(TestWidgetMenuEntry, self).__init__('Test Widget Menu Entry', icons, icon_paths=icon_paths)
        paths = []
        rp = rospkg.RosPack()
        for path in icon_paths:
            paths.append(os.path.join(rp.get_path(path[0]), path[1]))
        self._icon_helper = IconHelper(paths, name)
        converted_icons = self._icon_helper.set_icon_lists(icons)
        self._test_widget = None
        self._icons = converted_icons[0]
        self._name = name
        self._test_widget_visible = False
        self.setIcon(self._icons[0])
        self.clicked.connect(self._test_widget_show)
        self.context = context

        from python_qt_binding.QtCore import QMutex
        self._close_mutex = QMutex()
        self._show_mutex = QMutex()
        
    def _test_widget_show(self):
        from python_qt_binding.QtCore import QMutexLocker
        with QMutexLocker(self._show_mutex):
            try:
                if self._test_widget_visible:
                    rospy.loginfo('Showing TestWidget')
                    self.context.remove_widget(self._test_widget)
                    self._test_widget_close()
                    self._test_widget_visible = False
                    self.setIcon(self._icons[0])
                else:
                    rospy.loginfo('Hiding TestWidget')
                    self._test_widget = TestWidget()
                    self.context.add_widget(self._test_widget)
                    rospy.loginfo('Added TestWidget')
                    self._test_widget_visible = True
                    self.setIcon(self._icons[1])
            except Exception as e:
                rospy.logerr('Error during showing TestWidget : %s', e.message)
                if self._test_widget == False:
                    rospy.logerr('Dang it!')
                    raise
                self._test_widget_visible = False
                #self._test_widget_show()
            
    def _test_widget_close(self):
        from python_qt_binding.QtCore import QMutexLocker
        with QMutexLocker(self._close_mutex):
            if self._test_widget_visible:
                self._test_widget.shutdown()
                self._test_widget.close()
                self._test_widget = None
#########################################################################################################
#########################################################################################################
#########################################################################################################
class SR2Dashboard(Dashboard):

  def setup(self, context):
      self.name = 'SR2Dashboard'
      self.context = context
      self._test_widget_menu_entry = TestWidgetMenuEntry(self.context)
      self._control_view = SR2ControlMenu(self.context)

    # declare subscribers and publishers
    #self._emergencystop = SR2EmergencyStopWidget(self.context)
#    self._mainview = SR2MainViewWidget(self.context)
    #self._estopSub = rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.update_emergency_stop_state_callback)
#    self._sr2layout = SR2LayoutCreator('config1.yaml', 'Launches')

  def get_widgets(self):
      """ Retrieves the widgets that will populate the menu (by default: top toolbar)
      It can contain various QWidgets however it is advised to use IconToolButton, QLabel
      or other predefined or custom simplistic QWidgets in order to keep the menu compact.
      Complex QWidgets are to be displayed in the main view
      """
    #return [[self._emergencystop, self._mainview]]
 #     return [[self._mainview, self._sr2layout]]
      return [[self._test_widget_menu_entry], [self._control_view]]

  def shutdown_dashboard(self):
      rospy.loginfo('Shutting down the SR2 Dashboard')
    # unregister all subscribers
#    self._estopSub.unregister()
#    rospy.loginfo("Emergency stop subsciber unregistered")
      pass

  def save_settings(self, plugin_settings, instance_settings):
      pass

  def restore_settings(self, plugin_settings, instance_settings):
      pass

#  def update_emergency_stop_state_callback(self, state):
#    self._emergencystop.update_state(state.emergency_button_stop)
