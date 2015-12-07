#!/usr/bin/env python2.7
"""
Created on Thu Jul 23 10:21:47 2015

@author: atanasov2
"""

"""
TODO:
    - Add the ability for multiple SR2LayoutCreator to use the same SR2YamlLoader (pass by reference)
      This will improve the performance since a YAML file will be processed only once and each SR2LayoutCreator
      will then extract only the information it requires without the need to parse the file again
    - Better error checking - look at a more efficient way to propagate errors
    - Add automated creation for complete SR2 Dashboard widget
"""

from os.path import isfile
from exceptions import IOError, KeyError, TypeError
from yaml import YAMLError, load

# Exceptions
class UnknownModuleException(Exception):
    pass

class UnknownCommandException(Exception):
    pass

class TooManyCommands(Exception):
    pass

class MissingCommand(Exception):
    pass

class MissingLaunchFiles(Exception):
    pass

class SR2YamlLoaderFailureException(Exception):
    pass

class MissingComponents(Exception):
    pass

class SR2YamlLoader:
    ''' Converts a SR2Dashboard valid configuration file (YAML) into a format,
    which can then be used for generating the UI of the SR2Dashboard. It does NOT
    create any real UI elements. This is done the SR2UiGenerator'''

    def __new__(cls, yamlConfigFile):
        
        if not isfile(yamlConfigFile):
            raise IOError("Unable to open configuration file file")
        
        return super(SR2YamlLoader, cls).__new__(cls)
    
    def __init__(self, yamlConfigFile):
        self.config = yamlConfigFile
        self.components = []    # contains all the buttons that will be placed in the SR2YamlGuiWidget
        pass
    
    def get_buttons(self, **module):
        ''' Creates a list of button-dictionaries. For more details see SR2YamlLoader.parse_config_file()'''
        buttons = []
        try:
            for button in module['buttons']:
                buttons.append(self.generate_button_data(**button))
        except (TypeError, KeyError) as e:
            print(e.message)
            return []
            
        return buttons  
        
    def generate_button_data(self, **button):
        ''' Creates a dictionary representing a button. For more details see SR2YamlLoader.parse_config_file()'''
        try:
            caption = button['caption']
            print("Parsing button '" + caption + "'")
            rospkg = button['rospkg']
            icon = None
            if button['icon'] != None:
                if isfile(button['icon']):
                    icon = button['icon']
            
            # Check if either 'roslaunch_files' (for a roslaunch command) or 'rosnode' (for a rosrun command) are present
            command_ok_1, command_ok_2 = False, False
            roscommand = None
            try:
                if 'rosnode' in button:
                    print("Found 'rosrun' command")
                    command_ok_1 = True
                    roscommand = 'rosrun'
            except UnknownCommandException('Unkown command') as e:
                pass
                    
            try:
                if 'roslaunch_files' in button:
                    print("Found 'roslaunch' command")
                    roscommand = 'roslaunch'
                    command_ok_2 = True
            except UnknownCommandException('Unkown command') as e:
                    pass
                
            # If both 'roslaunch_files' and 'rosnode' are missing
            if (not command_ok_1) and (not command_ok_2):
                raise MissingCommand('Unable to find "rosnode" or "roslaunch_files"')
            
            # If both 'roslaunch_files' and 'rosnode' are present
            if command_ok_1 and command_ok_2:
                raise TooManyCommands('Found both "rosnode" and "roslaunch_files"')
                
            # We skip checking for incorrect rospkg, launch files and node
            # This will be propagated to the GUI hence even if we have errors here
            # the GUI components will be generated later on, displayed but prompt with
            # error messages in case of an error here

            # Extract the target(s) for the selected command
            if roscommand == 'roslaunch':
                if not len(button['roslaunch_files']):
                    raise MissingLaunchFiles('Missing launch files')
                target = []
                for lfile in button['roslaunch_files']:
                    target.append(lfile)
            else:
                target = button['rosnode']
                
            # generate action!
        except (KeyError, MissingCommand, MissingLaunchFiles) as e:
            print(e.message)
            return {}
            
        return {'caption' : caption, 'icon' : icon, 'rospkg' : rospkg, 'roscommand' : roscommand, 'target' : target}
    
    def parse_config_file(self):
        ''' Parses an SR2Dashboard valid configuration file (YAML)
            Returns: [ {menu_items} {main_view_items} ] with
                - [menu_items]: a list that contains all items that will be placed in the SR2Dashboard's menu stack
                - {main_view_items}: a dictionary that contains all modules that will be placed in the SR2Dashboard's
                                     main view (key - module's name, value - module's content); a module is a list of items:
                                     main_view_items = {[module1], [module2], ...}
                                     moduleX = [{button1}, {button2}, ...]
                                     Currently as a module only "Services" and "Launches" are supported
            Each item {button} is a dictionary with following keys:
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
            
            For more information see SR2LayoutCreator
        '''
        with open(self.config, 'r') as f:
            try:
                self.yamlData = load(f)
            except YAMLError as exc:
                print("Error parsing YAML file.")
                print "Full message: ",  exc
                return None
            
            modules = None
            try:
                modules = self.yamlData['modules']                
            except KeyError as e:
                print("No 'modules' present")
                return None
            
            menu_items = []
            main_view_items = {}
            try:
                print("Parsing modules")
                for module in modules:
                    print("Parsing module " + module['name'])
                    if module['name'] == 'Menu Items':
                        # parsing items for menu
                        menu_items = self.get_buttons(**module)
                    elif module['name'] == 'Services':
                        # parsing service items for main view
                        main_view_items['Services'] = self.get_buttons(**module)
                    elif module['name'] == 'Launches':
                        # parsing launch items for main view
                        main_view_items['Launches'] = self.get_buttons(**module)
                    else:
                        raise UnknownModuleException('Unknown module detected')
            except (KeyError, IOError, UnknownModuleException) as e:
                print(e.message)
                return None              
        self.components = [menu_items, main_view_items]
        
    def getComponents(self):
        return self.components
        
    def getActions(self):
        return self.actions
 

import sys #, subprocess
from os import remove, kill, mkdir
from os.path import exists
from signal import SIGINT
from PyQt4.QtGui import QApplication, QWidget, QHBoxLayout, QSizePolicy, QIcon, QToolButton#, QPushButton
from PyQt4.QtCore import QProcess, QSize
from PyQt4.QtCore import Qt

class SR2ButtonWidgetFactory:
    ''' Creates a button widget. For more information see SR2ButtonWidgetFactory.ButtonWidget'''
    
    # {'caption' : caption, 'icon' : icon, 'rospkg' : rospkg, 'roscommand' : roscommand, 'target' : target}
    class ButtonWidget(QWidget):
        ''' A widget with a button controlling a single process (in case of roslaunch it controlls
        indirectly all processes that were spawned by that command due to the nature of roslaunch)'''
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
            if self.icon != None:
                self.qbtn.setIconSize(QSize(self.qbtn.width()/2, self.qbtn.height()/2))
            
#            if self.caption != None:
#                font = self.font()
#                font.setPixelSize(self.qbtn.height()/4)
#                self.setFont(font)
                
        def __init__(self, caption, rospkg, roscommand, target, icon=None):
            super(SR2ButtonWidgetFactory.ButtonWidget, self).__init__()
            QWidget.__init__(self)
            
            print("Generating button")
            self.caption = caption
            self.icon = icon
            self.command = roscommand
            self.args = [rospkg]
            if roscommand == 'roslaunch':
                for launchFile in target:
                    self.args.append(launchFile)
            else:
                self.args.append(target)
                
            self.status = False
            self.pid = 0
            # A PID file is used for storage of the detached process
            # In case the UI crashes or is closed intentionally this
            # file is used to restore the UI's state and reconnect it
            # to the detached process so that it can be controlled
            # The format of a PID file for now is as follows:
            # rosrun:       rospkg + '_' + 'rosrun' + '_' + 'nodeName' + '.pid'
            # roslaunch:    rospkg + '_' + 'roslaunch' + '_' + 'launchFile1' + '_' + 'launchFile2' + '_' + ... + '.pid'
            # PID files are written to the .pid folder inside the folder where the executable is (in order to avoid the "Woops, I deleted this PID file by accident..." error)
            if not exists('.pid'):
                print("Creating hidden folder '.pid' for storing PID files")
                mkdir('.pid')
            
            self.pidFilePath = '.pid/' + rospkg + '_' + roscommand
            if roscommand == 'roslaunch':
                for launchFile in target:
                    self.pidFilePath = self.pidFilePath + '_' + launchFile
            elif roscommand == 'rosrun':
                    self.pidFilePath = self.pidFilePath + '_' + target
                    
            self.pidFilePath = self.pidFilePath + '.pid'
            print("PID file: '" + self.pidFilePath + "'")
            
            if isfile(self.pidFilePath):
                with open(self.pidFilePath) as pidF:
                    self.pid = int(pidF.readline())
                    print("Found '" + self.pidFilePath + "'. Restoring connection to detached process with PID" + str(self.pid))
                    self.status = True
            else:
                print("Warning: No '" + self.pidFilePath + "' detected. If you have started the detached process, closed the UI and deleted this file, the application will be unable to restore its state and the external process will be orphaned!")
            
            self.initUi()
        
        def initUi(self):
            ''' Creates a simply UI for the widget with a single button in it
                For the functionality behind the button see toggleProcess()'''
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
            if len(self.caption) != 0 and self.icon != None:
                self.qbtn = QToolButton(self)
                self.qbtn.setText('Start \"' + self.caption + '\"')
                self.qbtn.setIcon(QIcon(self.icon))
                self.qbtn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
                #self.qbtn.setStyleSheet('''
                #QToolButton {
                #    background: url(flaticon_Freepik_delete30.svg) top center no-repeat;
                #    padding-top: 32px;
                #}
                #''')
            elif len(self.caption) == 0 and self.icon != None:
                self.qbtn = QToolButton(self)
                self.qbtn.setIcon(QIcon(self.icon))
                self.qbtn.setToolButtonStyle(Qt.ToolButtonIconOnly)
            elif len(self.caption) != 0 and self.icon == None:
                self.qbtn = QToolButton(self)
                self.qbtn.setText('Start \"' + self.caption + '\"')
                self.qbtn.setToolButtonStyle(Qt.ToolButtonTextOnly)
            else:
                self.qbtn = QToolButton(self)
                
            self.qbtn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.qbtn.setCheckable(True)
            self.qbtn.resizeEvent = self.onResize
            
            if self.status:
            	self.qbtn.setChecked(True)
                if len(self.caption) != 0:
                     self.qbtn.setText('Stop \"' + self.caption + '\"')
             
            self.qbtn.clicked.connect(self.toggleProcess)
            self.qbtn.resize(self.qbtn.sizeHint())
            self.hbox.addWidget(self.qbtn)
            self.setLayout(self.hbox)
            self.setGeometry(300, 300, 250, 150)
            self.show()
            
        def broadcastedStop(self):
            import rospy
            rospy.loginfo("STOP SIGNAL")
            self.toggleProcess(False)
            
        def toggleProcess(self, val):
            ''' Controls a process (on/off states only) that is controlled by the button calling it
                It also stores the PID of the process inside a special PID file (*.pid), which is used
                in the recovery mechanism of the UI in a case of an UI crash or deliberate termination'''
            # If button pressed
            if val:
                print("Starting process")
        		# Note: when roslaunch is terminated all processes spawned by it are also terminated
        		# thus even if roscore has been started by the roslaunch process it will too be stopped :)
#        		self.status, self.pid = QProcess.startDetached('roslaunch', ['lt', 'talker.launch'], '.') #(self.command, self.args, '.')
                self.status, self.pid = QProcess.startDetached(self.command, self.args, '.') #(self.command, self.args, '.')
                if self.status:
                    print("PID:" + str(self.pid))
                    pidFile = open(self.pidFilePath, 'w')
                    pidFile.write(str(self.pid))
                    pidFile.close()
                    if len(self.caption) != 0:
                        self.qbtn.setText('Stop \"' + self.caption + '\"')			
                else:
                    self.qbtn.setChecked(False)
                    print("Error: Failed to create process!")
            else:
                print("Stopping process")
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
                    if SR2ButtonWidgetFactory.ButtonWidget.check_pid(self.pid):
                        self.success = kill(self.pid, SIGINT)
                    else:
                        print("Error: No process with PID " + str(self.pid) + " detected")
                        if isfile(self.pidFilePath):
                            remove(self.pidFilePath)
        			
                    # NOTE: using scripts (rosrun pkg_name script.py) and not launching ROS-confrom nodes creates
                    # nodes like "talker_121314_12121414", which are impossible to distinguish without too much
                    # fuss and make it really difficult to use 'rosnode kill' hence the requirement to start only
                    # nodes that have a simple, distinguishable name so that rosnode kill can be used or use roslaunch
                    # and launch files to give proper names
                    # == 0 : for subprocess.call() return value | == None : for os.kill() return value (None -> kill was successful)
                    if self.success == 0 or self.success == None:
                        print("Process stopped!")
                        self.status = False
                        self.pid = 0
                        if isfile(self.pidFilePath):
                            remove(self.pidFilePath)
                        if len(self.caption) != 0:
                            self.qbtn.setText('Start \"' + self.caption + '\"')
                    else:
                        print("Error: Failed to stop process!")
                        self.qbtn.setChecked(True)
    
    @staticmethod
    def createButtonWidget(buttonConfig):
        ''' Creates a QWidget with a button inside that is used to start/stop a given process
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
        '''
        assert buttonConfig != None, "Empty button configuration"
        
        return SR2ButtonWidgetFactory.ButtonWidget(buttonConfig['caption'], buttonConfig['rospkg'], buttonConfig['roscommand'], buttonConfig['target'], buttonConfig['icon'])
        
        
class SR2GridGenerator:
    ''' Used for returning the dimensions of a grid layout for the distribution of elements of a given list'''
    @staticmethod
    def isqrt(n):
        ''' Returns the integer square root of a given number using Newton's method'''
        x = n
        y = (x + 1) // 2
        while y < x:
            x = y
            y = (x + n // x) // 2
        return x
    
    @staticmethod
    def get_dim(list_len):
        ''' Splits a list in a square-matrix like structure. Used for creation of grid layouts
        Note: the dimensions first expand horizontally, then vertically'''
        col = SR2GridGenerator.isqrt(list_len)
    
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
        
from PyQt4.QtGui import QGridLayout

class SR2LayoutCreator(QWidget):
    ''' Creates a grid layout with ButtonWidget components for a single module from a valid configuration file (YAML)
        It is to be used as the main view of the SR2Dashboard for each of its components:
        
        Note: currently only services and launch files are supported
        (O = make floating, X = close sub-widget)
        -------------------------------------------------------
        | Menu Items @      @       @              ...        | ----- MENU VIEW
        -------------------------------------------------------
        |  services   O X |   launches   O X|     ...     O X |
        |                 |                 |                 |
        | b1    b2     b3 | b1    b2     b3 |                 |
        | b4    b4     b5 | b4    b4     b5 |      ...        | ----- MAIN VIEW
        |                 |                 |                 |
        |       ...       |       ...       |                 |
        |                 |                 |                 |
        -------------------------------------------------------
    '''  
    
    def __init__(self, yamlConfigFile, moduleName):
        ''' Initializes a grid layouted widget using the information extracted from a valid configuration file (YAML)
            and for a specific module defined in it (currently only Launches and Services)'''
        QWidget.__init__(self)
        super(SR2LayoutCreator, self).__init__()
        
        # Parse the YAML file
        try:
            self.sr2yl = SR2YamlLoader(yamlConfigFile)
        except IOError as e:
            raise e
            
        if self.sr2yl == None:
            raise SR2YamlLoaderFailureException('SR2YamlLoader failed to load the given configuration file')
            
        # Parse the configuration file
        self.sr2yl.parse_config_file()
        
        # Extract the defined components from the configuration file
        self.components = self.sr2yl.getComponents()
        if self.components == []:
            raise MissingComponents('No components found in the given configuration file')

        # We only use the information relevant for the main view here!            
        # Check if the given module as argument is actually present
        if moduleName not in self.components[1]:
            firstModule = True
            availableModules = ' '
            for module in self.components[1]:
                if firstModule:
                    firstModule = False
                    availableModules = availableModules + module
                else:
                    availableModules = availableModules + ', ' + module
                
            raise UnknownModuleException('Unknown module \"' + moduleName + '\". Choose one of the following:' + availableModules)
            
        print(self.components[1][moduleName])
        print("----------------------------------------")
        (self.rows, self.cols) = SR2GridGenerator.get_dim(len(self.components[1][moduleName]))
        print("ROWS: " + str(self.rows))
        print("COLS: " + str(self.cols))
        print '----------------------------------------'
        
        # Calculate the positions of each ButtonWidget within the grid layout
        positions = []
        for r in range(0, self.rows):
            for c in range(0, self.cols):
                positions.append([r, c])
                
        grid = QGridLayout()
        grid.setMargin(20)
        grid.setContentsMargins(5,5,5,5)
        self.setLayout(grid)
        self.setWindowTitle(moduleName)
        
        # Generate all ButtonWidget components and populate the grid layout
        componentIdx = 0
        for (buttonWidget, position) in zip(self.components[1][moduleName], positions):
            if componentIdx < len(self.components[1][moduleName]):
                print("Adding button '" + buttonWidget['caption'] + "' at " + "[row: " + str(position[0]) + "|col: " + str(position[1]) + "]")
                # caption, rospkg, roscommand, target, icon
                bW = SR2ButtonWidgetFactory.createButtonWidget(buttonWidget)
                if bW != None:
                    grid.addWidget(bW, position[0], position[1])

# TESTING
def main(): 
    app = QApplication(sys.argv)
    
#    yl = SR2YamlLoader('config1.yaml')
#    yl.parse_config_file()

#    from pprint import PrettyPrinter # for pretty-printing data structures (remove when finished)
#    pp = PrettyPrinter(indent=3)
#    pp.pprint(yl.getComponents())
     
    try:
         lw = SR2LayoutCreator('config1.yaml', 'Launches') #Services
         lw2 = SR2LayoutCreator('config1.yaml', 'Services')
    except (IOError, UnknownModuleException, MissingComponents, SR2YamlLoaderFailureException) as e:
        print e.message
        sys.exit(1)
        
    lw.show()
    lw2.show()

    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()