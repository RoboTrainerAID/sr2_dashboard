# Author: Aleksandar Vladimirov Atanasov
# Description: The SR2 Dashboard is used for controlling the SR2 service robot platform and view data about the state of the system

# ROS-related  modules
import roslib
roslib.load_manifest('sr2_dashboard')
import rospy 
import rospkg
# ROS-related  modules: RQT Robot Dashboard
from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.icon_tool_button import IconToolButton
from rqt_robot_dashboard.util import IconHelper
#from rqt_robot_dashboard.widgets import ...

#import sys
import os
from os import remove, kill, mkdir
from os.path import isfile, exists
from signal import SIGINT
from exceptions import IOError, KeyError, TypeError
from yaml import YAMLError

# COB messages
#from cob_msgs.msg import ...

# PyQt
# QtGui modules
from python_qt_binding.QtGui import QSizePolicy
from python_qt_binding.QtGui import QVBoxLayout, QHBoxLayout, QGridLayout
from python_qt_binding.QtGui import QWidget, QPushButton, QToolButton
from python_qt_binding.QtGui import QIcon
# QtCore modules
from python_qt_binding.QtCore import QProcess, QSize
from python_qt_binding.QtCore import Qt
#ToolButtonTextUnderIcon, ToolButtonTextOnly, ToolButtonIconOnly

# Widgets
# ...
#from .sr2_emergency_stop_widget import SR2EmergencyStopWidget
#from sr2_display_main_view import SR2MainViewWidget
#from yaml_gui import SR2LayoutCreator

# TODO:
# - Add the yaml_gui.py properly to the ROS infrastructure. Right now there is an issue with the paths,
#   since they need to be resolved by rospkg.RosPack, which is not included in the yaml_gui.py
# - Change the YAML configuration file. Do not use KEY "button" but simply add a list where each element starts with a string (the name of the button) and after that all button's properties

#import yaml_gui as yg

#class SR2ControlMenu(IconToolButton):
#    def __init__(self, context, name='SR2 Control Menu', icons=None, icon_paths=None, minimal=True):
#        if icons == None:
#          icons = []
#          icons.append(['control/menu/diagnostics.png'])
#          icons.append(['control/menu/diagnostics-click.png'])
#        icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]
#        
#        super(SR2ControlMenu, self).__init__('SR2 Control Menu', icons, icon_paths=icon_paths)
#        paths = []
#        self.rp = rospkg.RosPack()
#        for path in icon_paths:
#            paths.append(os.path.join(self.rp.get_path(path[0]), path[1]))
#        self._icon_helper = IconHelper(paths, name)
#        converted_icons = self._icon_helper.set_icon_lists(icons)
#        self._control_view = None
#        self._icons = converted_icons[0]
#        self._name = name
#        self._control_view_visible = False
#        self.setIcon(self._icons[0])
#        self.clicked.connect(self._control_view_show)
#        self.context = context
#
#        from python_qt_binding.QtCore import QMutex
#        self._close_mutex = QMutex()
#        self._show_mutex = QMutex()
#        
#    def _control_view_show(self):
#        from python_qt_binding.QtCore import QMutexLocker
#        from yaml import YAMLError
#        with QMutexLocker(self._show_mutex):
#            try:
#                if self._control_view_visible:
#                    rospy.loginfo('Activate SR2 Control View')
#                    self.context.remove_widget(self._control_view)
#                    self._control_view_close()
#                    self._control_view_visible = False
#                    self.setIcon(self._icons[0])
#                else:
#                    rospy.loginfo('Deactivate SR2 Control View')
#                    self._control_view = SR2ControlView(self.rp.get_path('sr2_dashboard') + '/resources/config/config1.yaml', 'Launches')
#                    #self._control_view = SR2ControlView(self.rp.get_path('sr2_dashboard') + '/resources/config/configDenis.yaml', 'Launches')
#                    #self._control_view = SR2ControlView(self.rp.get_path('sr2_dashboard') + '/resources/config/config1.yaml', 'Services')
#                    self.context.add_widget(self._control_view)
#                    rospy.loginfo('Added SR2 Control View to SR2 Dashboard')
#                    self._control_view_visible = True
#                    self.setIcon(self._icons[1])
#            except (KeyError, YAMLError, Exception, IOError, yg.MissingComponents, yg.SR2YamlLoaderFailureException, yg.UnknownModuleException) as e:
#                rospy.logerr('Error during showing SR2 Control View : %s', e.message)
#                if self._control_view == False:
#                    raise
#                self._control_view_visible = False
#                ##self._control_view_show()
#            
#    def _control_view_close(self):
#        from python_qt_binding.QtCore import QMutexLocker
#        with QMutexLocker(self._close_mutex):
#            if self._control_view_visible:
#                self._control_view.shutdown()
#                self._control_view.close()
#                self._control_view = None


#class SR2ControlView(QWidget):
#    
#    def __init__(self, yamlConfigFile, moduleName):
#        super(SR2ControlView, self).__init__()
#        obj_name = 'SR2 Control View ' + moduleName
#        self.setObjectName(obj_name)
#        self.setWindowTitle(obj_name)
#        """layout = QVBoxLayout()
#        button = QPushButton('Do something meaningful', self)
#        button.clicked.connect(self.print_message)
#        layout.addWidget(button)
#        self.setLayout(layout)"""
#        try:
#            self.sr2yl = yg.SR2YamlLoader(yamlConfigFile)
#        except IOError as e:
#            raise e
#        rospy.loginfo('Found YAML configuration file')
#
#        if self.sr2yl == None:
#            raise yg.SR2YamlLoaderFailureException('SR2YamlLoader failed to load the given configuration file')
#        rospy.loginfo('Opening YAML configuration file completed')
#        # Parse the configuration file
#        self.sr2yl.parse_config_file()
#        rospy.loginfo('Parsing YAML configuration file completed')
#        
#        # Extract the defined components from the configuration file
#        self.components = self.sr2yl.getComponents()
#        if self.components == []:
#            raise yg.MissingComponents('No components found in the given configuration file')
#        rospy.loginfo('Populating components structure completed')
#            
#        if moduleName not in self.components[1]:
#            firstModule = True
#            availableModules = ' '
#            for module in self.components[1]:
#                if firstModule:
#                    firstModule = False
#                    availableModules = availableModules + module
#                else:
#                    availableModules = availableModules + ', ' + module
#                
#            raise yg.UnknownModuleException('Unknown module \"' + moduleName + '\". Choose one of the following:' + availableModules)
#            
#        rospy.loginfo('Found given module in YAML configuration file')
#        # In case we have multiple (>1) ButtonWidgets a "Stop all" button is added for terminating all processes started via those ButtonWidgets
#        if len(self.components[1][moduleName]) > 1:
#            (self.rows, self.cols) = yg.SR2GridGenerator.get_dim(len(self.components[1][moduleName])+1) # Add button "Stop all" at the end
#        else:
#            (self.rows, self.cols) = yg.SR2GridGenerator.get_dim(len(self.components[1][moduleName]))
#        positions = []
#        for r in range(0, self.rows):
#            for c in range(0, self.cols):
#                positions.append([r, c])
#                
#        from PyQt4.QtGui import QGridLayout
#        self.grid = QGridLayout(self)
#        self.grid.setMargin(20)
#        self.grid.setContentsMargins(5,5,5,5)
#        self.setLayout(self.grid)
#        rospy.loginfo('Generating grid completed')
#        
#        componentIdx = 0
#        for (buttonWidget, position) in (zip(self.components[1][moduleName], positions[:-1]) if len(self.components[1][moduleName]) > 1 else zip(self.components[1][moduleName], positions)):
#            if componentIdx < len(self.components[1][moduleName]):
#                rospy.loginfo('Adding button \"%s\" at [row: %s |col: %s]', buttonWidget['caption'], str(position[0]), str(position[1]))
#                # caption, rospkg, roscommand, target, icon
#                bW = yg.SR2ButtonWidgetFactory.createButtonWidget(buttonWidget)
#                if bW != None:
#                    self.grid.addWidget(bW, position[0], position[1])
#        
#        # FIXME: Currently the "STOP ALL" button is not working properly
#        if len(self.components[1][moduleName]) > 1:
#            rospy.loginfo('Adding "Stop all" button')
#            def stopAll(self, layout):
#                #from PyQt4.QtCore import QObject, pyqtSignal
#                #class stopEmitter(QObject):
#                #    stopSig = pyqtSignal()
#                #self.emitter = stopEmitter()
#                
#                widgets = (layout.itemAt(i).widget() for i in range(layout.count()))
#                for widget in widgets:
#                    if isinstance(widget, yg.SR2ButtonWidgetFactory.ButtonWidget):
#                        rospy.loginfo('Trying to terminate process started by "' + widget.caption + '"')
#                        #widget.toggleProcess(False) # terminate the process bound to the current widget if a PID file is detected
#                        widget.broadcastedStop()
#                        # FIXME: for some reason the button behaviour cannot be controlled decently from here
#                        # The toggling works partially - caption changes, process (if PID file present) is terminated but the ButtonWidget's state actually remains checked
#                        # Note: signals are a bad idea here since specifying the receiver is against the concept of signals and slots in Qt...I think :D
#            stopAllButton = QPushButton('Stop all (NOT WORKING)')
#            from PyQt4.QtGui import QSizePolicy
#            stopAllButton.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
#            stopAllButton.clicked.connect(lambda: stopAll(self, self.grid))
#            self.grid.addWidget(stopAllButton, positions[-1][0], positions[-1][1])
#            
#        
#        rospy.loginfo('Adding components to grid completed')
#    
#    def shutdown(self):
#        rospy.loginfo('Closing SR2 Control View.')
#        rospy.loginfo('Note: started processes will still remain active even when this widget is closed.\n' +
#                      'In order to terminate those re-open the SR2 Control View and terminate them one by\n' +
#                      'one or using the "Stop all" button')
#        
#        
#    def save_settings(self, plugin_settings, instance_settings):
#        pass
#    
#    def restore_settings(self, plugin_settings, instance_settings):
#        pass
    
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

#########################################################################################################
#########################################################################################################
#########################################################################################################

# WORK IN PROGRESS
# Integrate the SR2ButtonWidgetFactory, SR2GridGenerator and SR2LayoutCreator classes here       


# TODO
# All buttons have to receive feedback of whether the executed process was successful or not
class SR2DashboardItemFactory:
    """
    Generates SR2 dashboard items - menu entries and (if available) the corresponding menu views (one view per entry!)
    """
    
    class SR2MenuView(QWidget):
        """
        A menu view is displayed in the dashboard's central main area and is connected to a menu entry.
        It contains a grid layout with ButtonWidget components for a single view
        
        WARNING: Supersedes SR2LayoutCreator

        (O = make floating, X = close sub-widget)
        -------------------------------------------------------
        | menu 1 [menus entries @  @  @]         ...          | ----- TOOLBAR: contains multiple menus each with one or multiple menu entries
        -------------------------------------------------------
        | menu view 1 O X | menu view 2  O X|      ...    O X |
        |                 |                 |                 |
        | b1    b2     b3 | b1    b2     b3 |                 |
        | b4    b4     b5 | b4    b4     b5 |      ...        | ----- MAIN VIEW: a menu entry can also have a view which is shown in this area
        |                 |                 |                 |
        |       ...       |       ...       |                 |
        |                 |                 |                 |
        -------------------------------------------------------
        """
        
        class SR2ButtonWidgetFactory:
            
            class ButtonWidget(QWidget):
                # TODO CHECK IF THIS WORKS PROPERLY AND ALTER IT SO THAT IT FITS THE NEW PARSING METHOD
                """
                A widget with a button controlling a single process (in case of roslaunch it controlls
                indirectly all processes that were spawned by that command due to the nature of roslaunch)
                """
                
                @staticmethod
                def check_pid(pid):
                    ''' Checks if a process with a given PID is running or not'''
                    try:
                        kill(pid, 0)
                        return True
                    except OSError:
                        return False
                
                def onResize(self, event):
                    ''' Called whenever a resize event is triggered on the widget. It resizes the icon (if present) inside the button'''
                    if self.icons != None:
                        self.qbtn.setIconSize(QSize(self.qbtn.width()/2, self.qbtn.height()/2))
                    
                    #if self.caption != None:
                    #  font = self.font()yamlButtonList[ybutton]
                    #  font.setPixelSize(self.qbtn.height()/4)
                    #  self.setFont(font)
                                
                def __init__(self, pkg, cmd, args, captions, icons=None):
                    super(SR2DashboardItemFactory.SR2MenuView.SR2ButtonWidgetFactory.ButtonWidget, self).__init__()
                    QWidget.__init__(self)
                    
                    print("Generating button")
                    self.captions = captions
                    self.icons = icons
                    self.command = cmd
                    self.args = args
                    self.status = False
                    self.pkg = pkg
                    self.pid = 0
                    
                    # A PID file is used for storage of the detached process
                    # In case the UI crashes or is closed intentionally this
                    # file is used to restore the UI's state and reconnect it
                    # to the detached processes so that these can be controlled
                    # The format of a PID file for now is as follows:
                    # rosrun:       rospkg + '_' + 'rosrun' + '_' + 'nodeName' + '.pid'
                    # roslaunch:    rospkg + '_' + 'roslaunch' + '_' + 'launchFile1' + '_' + 'launchFile2' + '_' + ... + '.pid'
                    # PID files are written to the .pid folder inside the folder where the executable is (in order to avoid the "Woops, I deleted this PID file by accident..." error)
                    if not exists('.pid'):
                        print("Creating hidden folder '.pid' for storing PID files")
                        mkdir('.pid')
                    
                    # Only in case we have multiple launch files (determined by the spaces between the names of the files contained in the self.args string) we remove the spaces
#                    self.pidFilePath =  '.pid/'
#                                        + self.pkg
#                                        + '_'
#                                        + self.command
#                                        + '_'
#                                        + (self.args if ' ' in self.args == True else "".join(self.args.split()))
                    self.pidFilePath =  '.pid/%s_%s_%s.pid' % (self.pkg, self.command, self.args if ' ' in self.args == True else "".join(self.args.split()))
                    print('PID file: "' + self.pidFilePath + '"')
                    
                    if isfile(self.pidFilePath):
                        with open(self.pidFilePath) as pidF:
                            self.pid = int(pidF.readline())
                            print('Found "' + self.pidFilePath + '". Restoring connection to detached process with PID' + str(self.pid))
                            self.status = True
                    else:
                        print('Warning: No "' + self.pidFilePath + '" detected. If you have started the detached process, closed the UI and deleted this file, the application will be unable to restore its state and the external process will be orphaned!')
                    
                    # TODO Rewrite where necessary initUI and toggleProcess in order to included the three possible states for a button: default(=not pressed), pressed and warning (icon and caption for each of those states is provided by the YAML config file!)
                    self.initUi()
        
                def initUi(self):
                    '''
                    Creates a simply UI for the widget with a single button in it
                    For the functionality behind the button see toggleProcess()
                    '''
                    self.hbox = QHBoxLayout()
        #            if len(self.caption) != 0:
        #                self.qbtn = QPushButton('Start \"' + self.caption + '\"', self)
        #            else:
        #                self.qbtn = QPushButton(self)
        #            if self.icon != None:
        #                self.qbtn.setIcon(QIcon(self.icon))
                        
                    # Sometimes Qt is full of ****. Why is it so hard to place an icon in the center of a button
                    # and the text aligned at the bottom of that button?!?!?!?!?!?!
                    # Result below looks bad but not as bad as using the default QPushbutton where text is aligned to the right
                    # of the icon.  Resizing text accordingly is also hard
                    # These two things are even more difficult to do when working with a grid layout!
                    self.qbtn = QToolButton(self)
                    self.qbtn.setText(self.captions[0])
                    self.qbtn.setIcon(QIcon(self.icons[0]))
                    self.qbtn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
#                    self.qbtn.setStyleSheet('''
#                    QToolButton {
#                        background: url(flaticon_Freepik_delete30.svg) top center no-repeat;
#                        padding-top: 32px;
#                    }
#                    ''')
#                
                        
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
            
                def broadcastedStop(self):
                    rospy.loginfo("Stop signal")
                    self.toggleProcess(False)
                    
                def toggleProcess(self, val):
                    ''' Controls a process (on/off states only) that is controlled by the button calling it
                        It also stores the PID of the process inside a special PID file (*.pid), which is used
                        in the recovery mechanism of the UI in a case of an UI crash or deliberate termination'''
                    # If button pressed
                    if val:
                        print('SR2: Starting process')
                		# Note: when roslaunch is terminated all processes spawned by it are also terminated
                		# thus even if roscore has been started by the roslaunch process it will too be stopped :)
        #        		self.status, self.pid = QProcess.startDetached('roslaunch', ['lt', 'talker.launch'], '.') #(self.command, self.args, '.')
                        self.status, self.pid = QProcess.startDetached(self.command, self.args, '.')
                        if self.status:
                            # Change to pressed state
                            print('SR2: PID:' + str(self.pid))
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
                            rospy.logerr('SR2: Failed to start process')
                    else:
                        print('SR2: Stopping process')
                        if self.status:
                            # kill takes a very short amount of time hence we can call it from inside the main thread without freezing the UI
                            self.success = None
                            # WARNING: ROS documentation states that "rosnode kill" is not guaranteed to succeed
                            # especially when the node that is to be killed has its "respawn" property on hence
                            # we use the kill command with SIGINT signal for both cases - roslaunch and rosrun
                            
                            # OLD VERSION
                            '''if self.command == 'rosrun':
                                self.success = subprocess.call(['rosnode', 'kill', self.args[-1]]) # the last element of the list with args (only when a node is started with rosrun!) is the name of the node
                                # Dang it, "rosnode kill" sucks!
                            else:
                                if SR2ButtonWidgetFactory.ButtonWidget.check_pid(self.pid):
                                    self.success = kill(self.pid, SIGINT)
                                else:
                                    print 'Error: No process with PID ' + str(self.pid) + ' detected'
                                    if isfile(self.pidFilePath):
                                        remove(self.pidFilePath)'''
                            if SR2DashboardItemFactory.SR2MenuView.SR2ButtonWidgetFactory.ButtonWidget.check_pid(self.pid):
                                self.success = kill(self.pid, SIGINT)
                            else:
                                rospy.logerr('SR2: No process with PID ' + str(self.pid) + ' detected')
                                if isfile(self.pidFilePath):
                                    remove(self.pidFilePath)
                			
                            # NOTE: using scripts (rosrun pkg_name script.py) and not launching ROS-confrom nodes creates
                            # nodes like "talker_121314_12121414", which are impossible to distinguish without too much
                            # fuss and make it really difficult to use 'rosnode kill' hence the requirement to start only
                            # nodes that have a simple, distinguishable name so that rosnode kill can be used or use roslaunch
                            # and launch files to give proper names
                            # == 0 : for subprocess.call() return value | == None : for os.kill() return value (None -> kill was successful)
                            if self.success == 0 or self.success == None:
                                rospy.loginfo("SR2: Process stopped!")
                                self.status = False
                                self.pid = 0
                                if isfile(self.pidFilePath):
                                    remove(self.pidFilePath)
                                self.qbtn.setText(self.captions[0])			
                                self.qbtn.setIcon(QIcon(self.icons[0]))
                            else:
                                self.qbtn.setChecked(True)
                                self.qbtn.setText(self.captions[2])			
                                self.qbtn.setIcon(QIcon(self.icons[2]))
                                rospy.logerr('SR2: Failed to stop process')                                
            
                @staticmethod
                def createButtonWidget(yamlButtonConfig):
                    """
                    Creates a QWidget with a button inside that is used to start/stop a given process
                    A button configuration is a dictionary with following keys:
                    - caption: the string that will be used to label the button
                    - icon (optional): a valid path to an image file (SVG, PNG etc. - all supported format by Qt)
                    - rospkg:  a valid ROS package
                    - command:    the type of command for 'target'; can be 'roslaunch' or 'rosrun'
                    - target:  the target to be executed:
                        - for a 'roslaunch' command: a list of valid ROS launch files within the selected 'rospkg'
                          ['file1.launch', 'file2.launch', ...]
                        - for a 'rosrun' command: name of a valid ROS node
                    - action: reference to a function based on rospkg, command and target; it is the functionality that the
                              created button will offer once inserted in the GUI
                    """
                    
                    print('SR2: parsing button configuration', yamlButtonConfig)
                    assert yamlButtonConfig != None, "Empty button configuration"
                    
                    pkg = ''
                    cmd = ''
                    args = ''
                    # Try each of the possible configurations: node, launch and service
                    # TODO Check if launch files etc exist or always launch the chosen command and let it fail if these are not present (let ROS handle the search for the files)
                    try:
                        pkg = yamlButtonConfig['package']
                        rospy.loginfo('Using package %s' % pkg)
                        try:
                            args = yamlButtonConfig['node']
                            rospy.loginfo('Nodes detected. Will use "rosrun"')
                            cmd = 'rosrun'
                            
                        except KeyError:
                            try:
                                _args = yamlButtonConfig['launch']
                                rospy.loginfo('Launch file(s) detected. Will use "roslaunch"')
                                cmd = 'roslaunch'
                                
                                if isinstance(_args, list):
                                    for i in range(0, len(_args)):
                                        args = args + ' ' + _args[i]
                                    args = args.lstrip(' ')
                                    rospy.loginfo('Multiple launch files detected: "%s"' % args)
                                else:
                                    args = _args
                                    rospy.loginfo('Single launch file detected: "%s"' % args)
                                
                            except KeyError:
                                try:
                                    args = yamlButtonConfig['serivce']
                                    rospy.loginfo('Service deteceted. Will use "rosservice call"')
                                    cmd = 'rosservice call'

                                except KeyError as exc:
                                    rospy.logerror('Button does not contain data that can be executed by the supported ROS tools. Add node, launch or service to the button"s description')
                                    raise exc   
                                    
                        captions = []
                        captions.append(yamlButtonConfig['captions']['default'])
                        captions.append(yamlButtonConfig['captions']['pressed'])
                        captions.append(yamlButtonConfig['captions']['warning'])
                        
                        #icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]
                        icons = []
#                        for path in icon_paths:
#                            paths.append(os.path.join(rp.get_path(path[0]), path[1]))
#                        _icon_helper = IconHelper(paths, name)
#                        converted_icons = self._icon_helper.set_icon_lists(icons)
#                        _icons = converted_icons[0]
                        # TODO Use the IconHelper to populate the icons list using the names of the 
                        icons.append(yamlButtonConfig['icons']['default'])
                        icons.append(yamlButtonConfig['icons']['pressed'])
                        icons.append(yamlButtonConfig['icons']['warning'])
                            
                    except KeyError as exc:
                        rospy.loginfo('SR2: error while loading YAML file.')
                        rospy.loginfo('SR2: full message: \n"%s"' % exc)
                        return None

                    
                    #return SR2DashboardItemFactory.SR2MenuView.SR2ButtonWidgetFactory.ButtonWidget(yamlButtonConfig['caption'], yamlButtonConfig['rospkg'], yamlButtonConfig['roscommand'], yamlButtonConfig['target'], yamlButtonConfig['icon'])
                    return SR2DashboardItemFactory.SR2MenuView.SR2ButtonWidgetFactory.ButtonWidget(pkg, cmd, args, captions, icons)
                    
        
        class SR2GridGenerator:
            """
            Returns the dimensions of a grid layout for the distribution of elements of a given list
            """
            @staticmethod
            def isqrt(n):
                """
                Returns the integer square root of a given number using Newton's method (so called integer square root)
                """
                x = n
                y = (x + 1) // 2
                while y < x:
                    x = y
                    y = (x + n // x) // 2
                return x
            
            @staticmethod
            def get_dim(list_len):
                """
                Splits a list in a square-matrix like structure. Used for creation of grid layouts
                Note: the dimensions expand horizontally first and then vertically
                """
                col = SR2DashboardItemFactory.SR2MenuView.SR2GridGenerator.isqrt(list_len)
            
                # special cases
                # 1x1 for a list with a single element
                if list_len == 1:
                    return (1, 1)
                # empty list
                if list_len == 0:
                    return (0, 0)
                # perfect sqrt numbers produce a square matrix without empty cells
                if col**2 == list_len:
                    return (col, col)
                # all the rest
                # following is suitable for displays with height < width
                # it can easily be reversed so that it can handle height > width
                elif col**2 < list_len <= col*(col+1):
                    return (col, col+1)
                elif col*(col+1) < list_len <= (col+1)**2:
                    return (col+1, col+1)
                            
        def __init__(self, yamlButtonList, override_icons, name='Default name'):
            """
            Initializes a SR2MenuView object
            
            :param yamlButtonList: A list of SR2 YAML buttons
            :param override_icons: :param override_icons: if set to true default icons will be used (if available!) inside "$(find sr2_dashboard)/resources/images" 
            :param name: Unique string that is used to identify the SR2MenuView object
            """
            super(SR2DashboardItemFactory.SR2MenuView, self).__init__()
            obj_name = name
            self.setObjectName(obj_name)
            self.setWindowTitle(obj_name)
            
            rospy.loginfo('SR2: Creating view for menu entry "%s"' % name)

            grid = QGridLayout()
            grid.setMargin(20)
            grid.setContentsMargins(5,5,5,5)
            
            if yamlButtonList == None:  
                rospy.logwarn('SR2: No data to populate view. View will be empty')
                self.setLayout(grid)
                return;
                
            # Get dimensions of grid based on number of buttons
            (self.rows, self.cols) = SR2DashboardItemFactory.SR2MenuView.SR2GridGenerator.get_dim(len(yamlButtonList))
            rospy.loginfo('SR2: View contains %d buttons which will be distributed on a %d by %d (rows by columns) grid layout' % (len(yamlButtonList), self.rows, self.cols))
                
            # PARSE buttons
            self.buttons = []
            for ybutton in range(0, len(yamlButtonList)):
                # Use createButtonWidget(...) (from yaml_gui.py) to generate each button
                # and then append it to the the list with buttons
                
                #rospy.loginfo('SR2: Parsing button: %s' % yamlButtonList[ybutton])
                button = SR2DashboardItemFactory.SR2MenuView.SR2ButtonWidgetFactory.ButtonWidget.createButtonWidget(yamlButtonList[ybutton])
                self.buttons.append(button)
                pass
            
            button = QPushButton('SR2: Click me!', self)
            button.clicked.connect(self.print_message)
            grid.addWidget(button)
            self.setLayout(grid)
            
        def print_message(self):
            rospy.loginfo('SR2: Hello!')
        
        def shutdown(self):
            rospy.loginfo('SR2: Bye!')
            
        def save_settings(self, plugin_settings, instance_settings):
            pass
        
        def restore_settings(self, plugin_settings, instance_settings):
            pass
        
    class SR2MenuEntry(IconToolButton):
        """
        A menu entry is displayed in the dashboard's toolbar area. It may contain a SR2MenuView widget, the visibility of which is toggled by pressing the menu entry
        """

        from enum import Enum
        class IconType(Enum):
            default = 0
            pressed = 1
            warning = 2
        
        def __init__(self, context, yamlSR2MenuEntry, override_icons, name='Default name', minimal=True):
            icons = None
            icon_paths=None
            self.override_icons = override_icons
            self.withView = True if yamlSR2MenuEntry['type'] == 'view' else False
            self.buttons = None
            try:
                if self.withView:
                    self.buttons = yamlSR2MenuEntry['menu_entry']['buttons']
            except YAMLError:
                rospy.logwarn('SR2: detected menu entry with view which does NOT contain any buttons. An empty view will be created instead')
                return None  

            if not override_icons:
                if len(yamlSR2MenuEntry['menu_entry']['icons']) == 0: icons = None
                else: icons = []
                
                # DO NOT load this with a loop since it's a dictionary which means that order may be different upon each execution of the code => referencing the icons by index would be impossible
                # TODO Possibly put lines below in a try-except block?
                icons.append([yamlSR2MenuEntry['menu_entry']['icons']['default']])
                icons.append([yamlSR2MenuEntry['menu_entry']['icons']['pressed']])
                icons.append([yamlSR2MenuEntry['menu_entry']['icons']['warning']])
            
            if icons == None and override_icons:
              icons = []
              
              rospy.loginfo('SR2: Menu entry is of type "%s"' % yamlSR2MenuEntry['type'])
              if self.withView:
                  icons.append(['control/menu/diagnostics.png'])
                  icons.append(['control/menu/diagnostics-click.png'])
              else:
                  icons.append(['status/status_normal.svg'])
                  icons.append(['status/status_normal_pressed.svg'])
              icons.append(['status/status_warning.svg'])

            #icon_paths = (icon_paths if icon_paths else []) + [['rqt_robot_dashboard', 'images']]
            icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]
            
            super(SR2DashboardItemFactory.SR2MenuEntry, self).__init__(name, icons, icon_paths=icon_paths)
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
            self.setIcon(self._icons[SR2DashboardItemFactory.SR2MenuEntry.IconType.default.value])
            self.clicked.connect(self._show)
            self.context = context
    
            from python_qt_binding.QtCore import QMutex
            self._close_mutex = QMutex()
            self._show_mutex = QMutex()
            
        def _show(self):
            """
            Toggles the visibility of the view (if such exists) that is connected to the menu entry
            """
            from python_qt_binding.QtCore import QMutexLocker
            with QMutexLocker(self._show_mutex):
                try:
                    if self._visible:
                        if self.withView:
                            # If menu entry has a view, remove it
                            rospy.loginfo('SR2: Hiding SR2MenuView "%s"', self._name)
                            self.context.remove_widget(self._widget)
                        self._close()
                        self._visible = False
                        self.setIcon(self._icons[SR2DashboardItemFactory.SR2MenuEntry.IconType.default.value])
                        #self.setIcon(self._icons[2])
                    else:
                        if self.withView:
                            # If menu entry has a view, create it and display it
                            rospy.loginfo('SR2: Showing SR2MenuView "%s View"', self._name)
                            self._widget = SR2DashboardItemFactory.SR2MenuView(self.buttons, self.override_icons, self._name)
                            self.context.add_widget(self._widget)
                            rospy.loginfo('SR2: Added SR2MenuView "%s"', self._name)
                        self._visible = True
                        self.setIcon(self._icons[SR2DashboardItemFactory.SR2MenuEntry.IconType.pressed.value])
                except Exception as e:
                    if self.withView and self._widget == False:
                        rospy.logerr('SR2: Error during showing SR2MenuView : %s', e.message)
                        raise
                    self._visible = False
                    #self._test_widget_show()
                
        def _close(self):
            """
            Unloads the plugin from memory
            """
            from python_qt_binding.QtCore import QMutexLocker
            with QMutexLocker(self._close_mutex):
                if self._visible:
                    if self.withView:
                        self._widget.shutdown()
                        self._widget.close()
                        self._widget = None
    
    @staticmethod
    def createDashboardItem(SR2MenuEntryContext, yamlItemData, override_icons, SR2MenuEntryName):    # Pass the node that contains the data for the dashboard item (menu entry + (if present) menu view)
        """
        Creates a SR2MenuEntry object with (if possible) an attached SR2MenuView widget that is shown/hidden upon interacting with the SR2MenuEntry object
        
        :param SR2MenuEntryContext: context (currently unknown what this contains)
        :param yamlItemData: YAML node containing the data for the menu entry and its view (if present) in the form of a dictionary
        :param override_icons: if set to true default icons will be used (if available!) inside "$(find sr2_dashboard)/resources/images"
        :returns: SR2MenuEntry which can be added to a Dashboard
        """
#        print('************************************')
#        print(yamlItemData)
#        print('************************************')
      
        if SR2MenuEntryContext == None or yamlItemData == None:
            pass
        else:
            return SR2DashboardItemFactory.SR2MenuEntry(SR2MenuEntryContext, yamlItemData, override_icons, SR2MenuEntryName)
    
#########################################################################################################
#########################################################################################################
#########################################################################################################
class SR2Dashboard(Dashboard):

  def setup(self, context):
      print('SR2: starting dashboard')
      self.name = 'SR2Dashboard'
      self.context = context
      self.yFile = None
      self.widgets = []
      
      # Check if configuration is uploaded to the parameter server
      if not rospy.has_param('/sr2_dashobard/dashboard/menus'):
          return
          
      self._yMenus = rospy.get_param('/sr2_dashobard/dashboard/menus')
      
      #if rospy.has_param('/)
      #self.override_icons = rospy.get_param
      self.override_icons = False
      if rospy.has_param('/sr2_dashobard/dashboard/override_icons'):
          self.override_icons = rospy.get_param('/sr2_dashobard/dashboard/override_icons')
          
      if self.override_icons:
          rospy.loginfo('SR2: Icons will be overriden')
      else:
          rospy.loginfo('SR2: Icons will be extracted from configuration file')
      

      if len(self._yMenus) > 0:
          rospy.loginfo('SR2: Found %d menus' % len(self._yMenus))
      else:
          rospy.loginfo('SR2: No menus found! Dashboard will be empty')
          return

      # Populate the self.widgets list
      self._generate_widgets()
      
#      self.menu_entry_test_1 = SR2DashboardItemFactory.createDashboardItem(context, '', 'Menu Entry 1') #TestWidgetSR2MenuEntry(self.context)
#      self.menu_entry_test_2 = SR2DashboardItemFactory.createDashboardItem(context, '', 'Menu Entry 2')
#      self.menu_entry_test_3 = SR2DashboardItemFactory.createDashboardItem(context, '', 'Menu Entry 3')
#      self.menu_entry_test_4 = SR2DashboardItemFactory.createDashboardItem(context, '', 'Menu Entry 4')
#      self.menu_entry_test_5 = SR2DashboardItemFactory.createDashboardItem(context, '', 'Menu Entry 5')
#      self.menu_entry_test_6 = SR2DashboardItemFactory.createDashboardItem(context, '', 'Menu Entry A')
#      self._control_view = SR2ControlMenu(self.context)

    # declare subscribers and publishers
    #self._emergencystop = SR2EmergencyStopWidget(self.context)
#    self._mainview = SR2MainViewWidget(self.context)
    #self._estopSub = rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.update_emergency_stop_state_callback)
#    self._sr2layout = SR2LayoutCreator('config1.yaml', 'Launches')

  def _generate_widgets(self):
      try:
          # Iterate through all menus
          for menuIdx in range(0,len(self._yMenus)):
              rospy.loginfo('Found menu with %d entries' % len(self._yMenus[menuIdx]['modules']))
              widget_curr_menu = [] # Contains all SR2MenuEntry objects for the currently parsed menu X
              
              # Iterate through all menu entries of the current menu
              for menu_entryIdx in range(0, len(self._yMenus[menuIdx]['modules'])):
                  rospy.loginfo('Found menu entry "%s"' % self._yMenus[menuIdx]['modules'][menu_entryIdx]['name'])
                  
                  # TEST
                  #entry = SR2DashboardItemFactory.createDashboardItem(context, '', 'Menu Entry_' + str(menuIdx) + '_' + str(menu_entryIdx))
                  entry = SR2DashboardItemFactory.createDashboardItem(self.context, self._yMenus[menuIdx]['modules'][menu_entryIdx], self.override_icons, self._yMenus[menuIdx]['modules'][menu_entryIdx]['name'])
                  widget_curr_menu.append(entry)
                  
              
              #menu = SR2DashboardItemFactory.createDashboardItem(context, self._yMenus[menuIdx], self._yMenus[menuIdx]['name'])
              #widget_curr_menu.append(R2DashboardItemFactory.createDashboardItem(context, self._yMenus[menuIdx], self._yMenus[menuIdx]['name'])
              
              # Append the complete menu X to the list of widgets that are loaded when self.get_widgets() is called
              self.widgets.append(widget_curr_menu)
              
      except YAMLError as exc:
          rospy.loginfo('SR2: error while loading YAML file.')
          rospy.loginfo('SR2: full message: \n"%s"' % exc)
          return None

  def get_widgets(self):
      """
      Retrieves the widgets that will populate the menu (by default: top toolbar)
      It can contain various QWidgets however it is advised to use IconToolButton, QLabel
      or other predefined or custom simplistic QWidgets in order to keep the menu compact.
      Complex QWidgets are to be displayed in the main view
      """
      return self.widgets
#      return [
#          [self.menu_entry_test_1, self.menu_entry_test_2, self.menu_entry_test_3],
#          [self.menu_entry_test_4, self.menu_entry_test_5],
#          [self._control_view, self.menu_entry_test_6]
#      ]

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
