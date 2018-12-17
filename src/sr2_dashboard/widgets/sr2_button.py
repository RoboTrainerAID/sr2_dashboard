
# Author: Aleksandar Vladimirov Atanasov
# Co-Author: Gilbert Groten
# Description: A SR2 buttons (for both toolbar and view widgets)

# YAML
from yaml import YAMLError

from python_qt_binding.QtWidgets import QPushButton, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QSpacerItem, QSizePolicy, QFrame, QToolButton
from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer, QThread, pyqtSlot, pyqtSignal, QThreadPool, QSize

import roslib
import roslib.message
roslib.load_manifest('sr2_dashboard')
#ROS-related  modules: 
import rospy
import importlib
from rospy_message_converter import message_converter
import ast
import std_msgs
import trajectory_msgs
import tf2_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_geometry_msgs
import controller_manager_msgs
from controller_manager_msgs import *
import re
#RQT Robot Dashboard
from rqt_robot_dashboard.icon_tool_button import IconToolButton

from ..misc.sr2_monitor_object import SR2Worker, ProcStatus
from ..misc.sr2_runnable_object import SR2ServiceRunnable
from ..misc.sr2_runnable_object import SR2DynamicReconfigureServiceRunnable
from ..misc.sr2_grid_generator import SR2GridGenerator as sr2gg
from ..misc.sr2_ros_entry_extraction import SR2PkgCmdExtractor, IconType
from subprocess import call
from math import sqrt

# TODO Add button name to PID file to make workers for even same commands plus same arguments produce different files based on which part of the UI (toolbar, view) they belong to
# TODO Replace IconToolButton with own, add parent attribute in order to properly clean up files and objects; Add stylesheet stuff to the custom QToolButton
# TODO Pass init-component to view and view-internal widgets to block - currently not possible since all content of a view is destroyed whenever the view is hidden -> perhaps just show/hide view instead of destroying
# these two if init ext.process is not running

#used to check if  the dashboard being created depend on init parameters, which would influence the behaviour of buttons
hasinit = False

#helpfunction to define a default button style
def buttonStyle(button_type, icon, text = '', r = None, g = None, b = None, margin = None, border_radius = None):
    if not margin: margin = '3'
    if not border_radius: border_radius = '4'
    text = str(text)
    if text != '':
        font_size = int(400/sqrt(len(text)))
        if button_type is 'QToolButton':
            font_size /= 20
        font_size = str(font_size)
    else:
        font_size = '100'
    background = 'background: none'
    if r:
        if g:
            if b:
                r = str(r)
                g = str(g)
                b = str(b)
                background = 'background: rgb('+r+', '+g+', '+b+')'
    return button_type+':hover{border: 2px solid black;} '+button_type+'{margin: '+margin+'px; border-radius: '+border_radius+'px; image: url('+icon+') 0 0 0 0 stretch stretch; '+background+'; font: arial; font-size: '+font_size+'px; color: black;}'

#helpfunction to define a default button style for pushButtons
def pushButtonStyle(icon, text = '', r = None, g = None, b = None, margin = None, border_radius = None):
    return buttonStyle('QPushButton', icon, text, r, g, b, margin, border_radius)

#helpfunction to define a default button style for toolButtons
def toolButtonStyle(icon, text = '', r = None, g = None, b = None, margin = None, border_radius = None):
    return buttonStyle('QToolButton', icon, text, r, g, b, margin, border_radius)

#creates a stylesheet depending on the status of a button
def statusStyle(widget, status, type):
    if status == ProcStatus.INACTIVE or status == ProcStatus.FINISHED:
        return buttonStyle(type, widget.icon, widget.text,)
    elif status == ProcStatus.RUNNING:
        return buttonStyle(type, widget.icon, widget.text, 89, 205, 139)
    elif status == ProcStatus.FAILED_START:
        return buttonStyle(type, widget.icon, widget.text, 215, 56, 56)
    elif status == ProcStatus.FAILED_STOP:
        return buttonStyle(type, widget.icon, widget.text, 215, 56, 56)

#creates a stylesheet depending on the status of a button for toolbar-buttons
def toolButtonStatusStyle(widget, status):
    return statusStyle(widget, status, 'QToolButton')

#creates a stylesheet depending on the status of a button for view buttons
def pushButtonStatusStyle(widget, status):
    return statusStyle(widget, status,'QPushButton')

#help method to give a widget a toolButton property (used in multiple classes)
def setupToolButton(widget):
        style = toolButtonStyle(widget.icon, widget.text)
        widget.button = QToolButton(widget)
        widget.button.setStyleSheet(style)
        widget.setStyleSheet(style)
        widget.setFixedSize(QSize(36, 36))
        widget.button.setFixedSize(QSize(36, 36))
        #widget.button.clicked.connect(widget.call)
        return widget

#help method to give a widget a pushButton property (used in multiple classes)
def setupPushButton(widget, type = ''):
        layout = QVBoxLayout(widget)
        layout.setObjectName(widget.name)
        widget.button = QPushButton(widget)
        #controls layout (?)
        controls_layout = QHBoxLayout()
        spacer = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
        controls_layout.addItem(spacer)
        controls_layout.addWidget(widget.button)
        spacer2 = QSpacerItem(40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
        controls_layout.addItem(spacer2)
        layout.addLayout(controls_layout)
        #button main style
        style = pushButtonStyle(widget.icon, widget.text)
        widget.button.setToolTip(widget.tooltip)
        widget.button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        widget.button.setStyleSheet(style)
        widget.button.setStyleSheet(style)
        widget.button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        widget.button.setObjectName(widget.name)
        widget.button.icon = widget.icon
        #layout to display information regarding the process (description labels)
        info_layout = QVBoxLayout()
        if 'service' == type:
            service_nameL = QLabel('Service: ' + widget.srv_name, widget)
            service_nameL.setWordWrap(True)
            info_layout.addWidget(service_nameL)
            widget.reply_statL = QLabel('Reply status:', widget)
            widget.reply_statL.setWordWrap(True)
            info_layout.addWidget(widget.reply_statL)
            widget.reply_msgL = QLabel('Reply message:', widget)
            widget.reply_msgL.setWordWrap(True)
            info_layout.addWidget(widget.reply_msgL)
        elif 'publisher' == type:
            publisher_topicL = QLabel('Publisher Topic: ' + widget.topic, widget)
            publisher_topicL.setWordWrap(True)
            info_layout.addWidget(publisher_topicL)
            msg_str = ','.join(str(s) for s in widget.message_content)
            publisher_messageL = QLabel('Publisher Message: ' + msg_str, widget)
            publisher_messageL.setWordWrap(True)
            info_layout.addWidget(publisher_messageL)
        elif not 'multi' == type and not 'kill' == type:
            info_layout = QVBoxLayout()
            service_nameL = QLabel(widget)
            if widget.cmd not in ['roslaunch', 'rosrun']:
                if widget.display_name:
                    service_nameL.setText('App: ' + widget.display_name)
                else:
                    service_nameL.setText('App: ' + widget.cmd)
            else:
                if widget.display_name:
                    service_nameL.setText('External process: ' + widget.display_name)
                else:
                    service_nameL.setText('External process: ' + widget.args.replace('.launch', ''))
            service_nameL.setWordWrap(True)
            info_layout.addWidget(service_nameL)
        line = QFrame(widget)
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        info_layout.addWidget(line)
        layout.addLayout(info_layout)
        #setup the howl layout
        widget.setLayout(layout)
        widget.resize(layout.sizeHint())
        #connect button to its functionality
        #something something widget.button.connect(widget.call)
        return widget

class Status():
    '''
    Remaps the IconType values for better code semantics and readability
    '''
    inactive = IconType.inactive
    running = IconType.running
    error = IconType.error


class SR2Button():

    @staticmethod
                                    #(None, yaml_content_entry, name + str(idx), None, None, False, None, self)
    def createButton(context, yaml_entry_data, name, display_name=None, parent=None, init=False, init_widget=None, parent_button=None):
        '''
        Parses a YAML node for either a toolbar or a view. Based on successful parsing results on of the following types of buttons will be returned:

          - **SR2ButtonExtProcess** (subclass of QPushButton; for toolbar) - a menu entry that launches an external process, stops it and also monitors its running status
          - **SR2ButtonInitExtProcess** (subclass of SR2ButtonExtProcess) - a menu entry that launches an initialization external process, stops it and also monitors its running status
          - **SR2ViewButtonExtProcess** (subclass of QWidget; for view) - a view entry that launches an external process, stops it and also monitors its running status
          - **SR2ButtonService** (subclass of QPushButton; for toolbar) - a menu entry that calls a ROS Trigger-based service and displays its reply
          - **SR2ViewButtonService** (subclass of QWidget; for view) - a view entry that calls a ROS Trigger-based service and displays its reply
          - **SR2ToolbarButtonWithView** (subclass of QToolButton; for toolbar) - a menu entry that opens a view in the main view of the SR2 Dashaboard

        :param context: dashboard context used for adding a view to the dashboard
        :param yaml_entry_data: a valid YAML node which represents an entry in eithere a menu or a view
        :param name: uniquely generated name used as object name. Note: if name is not unique there will be a conflict between widgets that are part of the same toolbar or view
        :param display_name: optional name displayed used only by view entries; if not present, display name is automatically generated; this name is also only then visible whenever default icon is used
        '''
        if not yaml_entry_data:
            return None

        type = ''
        icon = ''
        global hasinit

        #find out what kind of UI element we are currently reading from the yaml-config and create the widget to display it
        if 'type' in yaml_entry_data:
            yamlEntry = yaml_entry_data['menu_entry']
            # We have an entry that is part of the toolbar
            if yaml_entry_data['type'] == 'noview':
                type, icon, text = SR2PkgCmdExtractor.getRosPkgCmdData(yamlEntry)
                if (not type):
                    type, icon, text = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data)
                    if (not type):
                        rospy.logerr("SR2: Could not find the type of a given Entry!")
                        rospy.logerr(yamlEntry)
                        return None
                rospy.logdebug('\n----------------------------------\n\tCREATE TOOLBAR BUTTON\n\t@Name: %s\n\t@yamlEntry: %s----------------------------------', name, yamlEntry)
                if type == 'service':
                    # Service
                    if not yamlEntry['service'] and not yamlEntry['name']:
                        rospy.logerr(
                            'SR2: Trying to create noview service button but service target is empty')
                        return None
                    if init:
                        hasinit = True
                        return SR2ButtonInitService(yamlEntry['service'], type, name, icon, text, parent, parent_button)
                    else:
                        return SR2ButtonService(yamlEntry['service'], type, name, icon, text, parent, parent_button)
                elif type == 'publisher':
                    #message publisher
                    if init:
                        hasinit = True
                        return SR2ButtonInitPublisher(yamlEntry['publisher'], name, icon, text, parent, parent_button)
                    else:
                        return SR2ButtonPublisher(yamlEntry['publisher'], name, icon, text, parent, parent_button)
                elif type == 'multi':
                    # Multi-Type type
                    if init:
                        hasinit = True
                        return SR2ButtonInitMulti(yamlEntry['multi'], name, icon, text, parent, parent_button)
                    else:
                        return SR2ButtonMulti(yamlEntry['multi'], name, icon, text, parent, parent_button)
                elif type == 'kill':
                    # Kill type
                    if init:
                        hasinit = True
                        return SR2ButtonInitKill(yamlEntry['kill'], name, icon, text, parent, parent_button)
                    else:
                        return SR2ButtonKill(yamlEntry['kill'], name, icon, text, parent, parent_button)
                else:
                    # External process (roslaunch, rosrun or app)
                    if init:
                        hasinit = True
                        return SR2ButtonInitExtProcess(yamlEntry[type], type, name, icon, text, parent)
                    else:
                        return SR2ButtonExtProcess(yamlEntry[type], type, name, icon, text, parent, parent_button)
            elif yaml_entry_data['type'] == 'view':
                # View
                type, icon, text = SR2PkgCmdExtractor.getRosPkgCmdData(yamlEntry, True)
                rospy.logdebug(
                    '\n----------------------------------\n\tCREATE TOOLBAR VIEW\n@Yaml_Contents: %s\n----------------------------------', yamlEntry)
                return SR2ButtonWithView(yamlEntry, type, name, icon, context, init_widget)
            elif yaml_entry_data['type'] == 'divisor':
                # Divisor
                icon = IconType.getDivisor()
                rospy.logdebug(
                    '\n----------------------------------\n\tCREATE TOOLBAR DIVISOR\n@Yaml_Contents: \n----------------------------------')
                return SR2Divisor(name, icon, parent)
            else:
                rospy.logerr(
                    'SR2: Unknown type of entry. Please make sure to specify "type" as either "noview", "view" or "divisor"')
                return None
        else:
            # We have an entry that is part of a view
            type, icon, text = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data)
            if (not type):
                return None
            rospy.logdebug('\n----------------------------------\n\tCREATE VIEW BUTTON\n\t@Name: %s\n\t@yaml_entry_data: %s----------------------------------', name, yaml_entry_data)
            if type == 'service':
                # Service
                if not yaml_entry_data['service']['name']:
                    rospy.logerr('SR2: Trying to create noview service button but service target is empty')
                    return None
                return SR2ViewButtonService(yaml_entry_data['service'], name, display_name, icon, text, parent, parent_button)
            elif type == 'publisher':
                    # Publisher type
                    return SR2ViewButtonPublisher(yaml_entry_data['publisher'], name, icon, text, parent, parent_button)
            elif type == 'multi':
                    # Multi-Type type
                    return SR2ViewButtonMulti(yaml_entry_data['multi'], name, icon, text, parent, parent_button)
            elif type == 'kill':
                    # Kill type
                    return SR2ViewButtonKill(yaml_entry_data['kill'], name, icon, text, parent, parent_button)
            else:
                # External process (roslaunch, rosrun or app)
                return SR2ViewButtonExtProcess(yaml_entry_data[type], type, name, display_name, icon, text, parent, parent_button)

############## SR2Divisor ############

#widget to seperate groups of toolbutton entries from eachother
class SR2Divisor(QLabel):
  
    def __init__(self, name, icon, parent = None):
      
        self.parent = parent
        
        super(SR2Divisor, self).__init__()
        
        self.setFixedSize(QSize(36, 36))
        self.setObjectName(name)
        self.name = name
        self.setText('|')


##########################################################################
# SR2ButtonDefault
##########################################################################

#basic setup for all buttons
class SR2ButtonDefault(QWidget):
    
    text = ''
    
    #identify the parameters for this element TODO redistribute onto child classes?
    def parse_yaml_entry(self, yamlEntry, type):
      
        if 'multi' == type:
            return
      
        #if button calls a service
        if 'service' == type:
            self.srv_name = yamlEntry['name']
            rospy.logdebug('SR2: Found service "%s"', self.srv_name)
            self.srv_name = '/' + self.srv_name
            if 'timeout' in yamlEntry:
                try:
                    self.timeout = int(yamlEntry['timeout'])
                    rospy.logdebug(
                        'SR2: Found timeout for service: %d', self.timeout)
                    if not self.timeout or self.timeout < 0:
                        rospy.logwarn(
                            'SR2: Timeout for service is either negative or equal zero. Falling back to default: 5')
                        self.timeout = 5
                except:
                    rospy.logwarn(
                        'SR2: Found timeout for service but unable to parse it. Falling back to default: 5')
                    self.timeout = 5
            else:
                self.timeout = 5
            
            if 'srv_type' in yamlEntry:
                self.type = yamlEntry['srv_type']
            else: 
                self.type = 'trigger'
            if 'params' in yamlEntry:
                self.params = yamlEntry['params']
                if 'toggle_params' in yamlEntry:
                    self.toggle_params = yamlEntry['toggle_params']
            return
          
        elif 'publisher' == type:
            self.topic = yamlEntry['topic']
            self.message_content = yamlEntry['message']
            self.message_type = yamlEntry['message_type']
            return
          
        #if the button is only meant to kill a rosnode
        elif 'kill' == type:
            self.kill = yamlEntry['name']
            if '/' not in self.kill:
                self.kill = '/'+self.kill
            return
          
        #if button calls a node/app/launch-file        
        elif 'kill' in yamlEntry:
            self.kill = yamlEntry['kill']
            if '/' not in self.kill:
                self.kill = '/'+self.kill
        if 'package' in yamlEntry:
            # We can have either a rosrun or roslaunch
            self.pkg = yamlEntry['package']
            rospy.logdebug('SR2: Found package "%s"', self.pkg)
            if 'node' == type:
                # We have a rosrun command
                self.cmd = 'rosrun'
                self.args = yamlEntry['name']        # rosrun pkg node
                rospy.logdebug('SR2: Found rosrun command for node "%s"', self.args)
            elif 'launch' == type:
                # We have a roslaunch command
                self.cmd = 'roslaunch'
                self.args = yamlEntry['name']      # roslaunch pkg launch_file
                if '.launch' not in self.args:
                    self.args += '.launch'
                rospy.logdebug('SR2: Found roslaunch command for launch file "%s"', self.args)
        elif 'app' == type:
            # We have a standalone application
            self.cmd = yamlEntry['name']
            rospy.logdebug('SR2: Found standalone application "%s"', self.cmd)
            if 'args' in yamlEntry:
                self.args = yamlEntry['args']
                rospy.logdebug('SR2: Found arguments for standalone application "%s"', self.args)
                self.timeout = 0
        else:
            rospy.logerr('SR2: Unable to parse YAML node:\n%s', yamlEntry)
     
    def setupButton(self):
        self = setupToolButton(self)
     
    def __init__(self, yamlEntry, type, name, icon, text = '', parent=None, parentButton=None):
        
        self.text = text
        self.parent = parent
        self.parentButton = parentButton #meant for use of 'multi' buttons or views with hidden functionality
      
        self.parse_yaml_entry(yamlEntry, type)
        
        super(SR2ButtonDefault, self).__init__()

        rospy.logdebug('\n----------------------------------\n\tEXT.PROC\n\t@Name: %s\n\t@yamlEntry: %s----------------------------------', name, yamlEntry)
        
        self.icon = icon
        self.setObjectName(name)
        self.name = name
        self.setToolTip(self.name)
        self.setupButton()
        
        
        #if the text is longer than 11 characters, split it into multiple lines
        if len(text) > 11:
          arr = text.split(" ")
          text = ''
          prev_lines_size = 0 #check size of current line to check if it's time to start a new line, don't check length of previous lines
          for i in range(0, len(arr)):
              if len(text) + len(arr[i]) - prev_lines_size > 10: #10 not 11 because of space!
                  text += " \n "
                  prev_lines_size = len(text)
              else:
                  text += ' '
              text += arr[i]
          #maybe better to not do all that at the beginning (and not put it into attribute), or '\n's will influence font size
          
          
        self.button.setText(text)
        self.button.clicked.connect(self.call)
        
        self.statusOkay = True  # Used for activating the acknowledgement mode where the user has to confirm the error before trying to launch the process again
        self.init_block_enabled = True

##########################################################################
# SR2ButtonExtProcess
##########################################################################


class SR2ButtonExtProcess(SR2ButtonDefault):
    '''
    Part of a toolbar; gives the ability to start/stop and monitor an external process (roslaunch, rosrun or standalone application)
    '''
    start_signal = pyqtSignal()
    stop_signal = pyqtSignal()
    clear_error_signal = pyqtSignal()
    kill = None #will be set in "SR2ButtonDefault.parse_yaml" if necessary

    def setupButton(self):
        self = setupToolButton(self)

    def __init__(self, yamlEntry, type, name, icon, text, parent=None, parentButton=None):

        super(SR2ButtonExtProcess, self).__init__(yamlEntry, type, name, icon, text, parent, parentButton)

        self.active = False    # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
        self.toggleControl = False
        self.setToolTip('Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + ((' ' + self.args) if self.args else '') + '"' + ' inactive')

        self.mutex_recovery = QMutex()
        self.mutex_status = QMutex()

        self.worker_thread = QThread()
        self.createWorker()

    def createWorker(self):
        '''
        Creates a worker (controls and monitors an external process), timer (worker reprots to the UI every 1s) and a thread (holds both the worker and timer)
        '''        
        # Create worker and connect the UI to it
        self.worker = None
        if self.pkg:
            self.worker = SR2Worker(self.name, self.cmd, self.pkg, self.args)
        else:
            self.worker = SR2Worker(
                self.name, cmd=self.cmd, pkg=None, args=self.args)
        QTimer.singleShot(1, self.worker.recover)
        self.worker.statusChanged_signal.connect(self.statusChangedReceived)
        self.worker.block_signal.connect(self.block)
        self.worker.recover_signal.connect(self.recover)
        self.start_signal.connect(self.worker.start)
        self.stop_signal.connect(self.worker.stop)
        self.clear_error_signal.connect(self.worker.clear_error)

        # Create a timer which will trigger the status slot of the worker every
        # 1s (the status slot sends back status updates to the UI (see
        # statusChangedReceived(self, status) slot))
        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.worker.status)

        # Connect the thread to the worker and timer
        self.worker_thread.finished.connect(self.worker.deleteLater)
        self.worker_thread.finished.connect(self.timer.deleteLater)
        self.worker_thread.started.connect(self.timer.start)

        # Move the worker and timer to the thread...
        self.worker.moveToThread(self.worker_thread)
        self.timer.moveToThread(self.worker_thread)

        # Start the thread
        self.worker_thread.start()

    def __del__(self):
        if(self.worker_thread.isRunning()):
            self.worker_thread.exit()
            while(not self.worker_thread.isFinished()):
                pass

    @pyqtSlot(int)
    def statusChangedReceived(self, status):
        '''
        Update the UI based on the status of the running process

        :param status: status of the process started and monitored by the worker

        Following values for status are possible:

          - **INACTIVE/FINISHED** - visual indicator is set to INACTIVE icon; this state indicates that the process has stopped running (without error) or has never been started
          - **RUNNING** - if process is started successfully visual indicator
          - **FAILED_START** - occurrs if the attempt to start the process has failed
          - **FAILED_STOP** - occurrs if the process wasn't stop from the UI but externally (normal exit or crash)
        '''
        self.tooltip = ''
        style = ''
        if status == ProcStatus.INACTIVE or status == ProcStatus.FINISHED:
            rospy.loginfo('SR2: Status has changed to: INACTIVE/FINISHED')
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' inactive/finished'
            self.active = False
        elif status == ProcStatus.RUNNING:
            rospy.loginfo('SR2: Status has changed to: RUNNING')
            self.tooltip = 'Ext.process "' + self.cmd + \
                ((' ' + self.pkg) if self.pkg else '') + \
                ((' ' + self.args) if self.args else '') + '"' + ' running'
            self.active = True
        elif status == ProcStatus.FAILED_START:
            rospy.logerr('SR2: Status has changed to: FAILED_START')
            style = toolButtonStyle(self.icon, self.text, 215, 56, 56)
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' failed to start'
            self.statusOkay = False
            self.active = False
            self.toggleControl = False
        elif status == ProcStatus.FAILED_STOP:
            rospy.logerr('SR2: Status has changed to: FAILED_STOP')
            style =  toolButtonStyle(self.icon, self.text, 215, 56, 56)
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' failed to stop'
            self.statusOkay = False
            self.active = False
            self.toggleControl = False
            
        self.setToolTip(self.tooltip)
        self.setStatusStyle(status)
        
        if self.parentButton:
            self.statusOkay = True #error acknowledged will be handled by parentButton
            self.parentButton.reply(self.active)

    @pyqtSlot(bool)
    def block(self, block_flag):
        '''
        Enable/Disable the button which starts/stops the external process
        This slot is used for preventing the user to interact with the UI while starting/stopping the external process after a start/stop procedure has been initiated
        After the respective procedure has been completed the button will be enabled again

        :param block_flag: enable/disable flag for the button
        '''
        self.setDisabled(block_flag)

    @pyqtSlot(bool)
    def block_override(self, block_override_flag):
        '''
        If connected to an init entry this slot will disable the interaction with the button if the init external process isn't running

        :param block_override_flag: enables/disable click action of button
        '''
        self.init_block_enabled = block_override_flag

    #def call(self):
        #self.call()
        #baum = 5

    @pyqtSlot()
    def call(self):
        '''
        Handles the start, stopping and error confirmation triggered by the button

          - **statusOkay is False** - this occurs ONLY if the process status is an error (FAILED_START or FAILED_STOP). In this case the user has to click twice on the button in order to reinitiate the starting procedure
          - **both statusOkay and toggleControl are True** - attempt to start the process
          - **statusOkay is True but toggleControl is False** - attempt to stop the process
        '''

        #if hasinit:
        #   if not self.init_block_enabled:  
        #        rospy.logerr('SR2: Init ext.process is not running. Unable to control ext.process connected to this button')
        #       return

        if not self.statusOkay:
            # If an error has occurred the first thing the user has to do is
            # reset the state by acknowleding the error
            self.statusOkay = True
            rospy.loginfo('SR2: Error acknowledged')
            self.clear_error_signal.emit()
            return

        self.toggleControl = not self.toggleControl
        if self.toggleControl:
            # Kill process if this is necessary to start own worker
            if self.kill:
                call(['rosnode', 'kill', self.kill])
            self.start_signal.emit()
        else:
            self.stop_signal.emit()

    @pyqtSlot()
    def recover(self):
        '''
        Sets button in an active mode. Triggered only if recovery of external process was successful
        '''
        self.toggleControl = True
        self.active = True
        self.statusOkay = True
        
    #adapt the style of this button to the current status of execution (aka disabled, running, failed)
    def setStatusStyle(self, status):
        self.button.setStyleSheet(toolButtonStatusStyle(self, status))

class SR2ViewButtonExtProcess(SR2ButtonExtProcess):
    '''
    Part of a view; gives the ability to start/stop and monitor an external process (roslaunch, rosrun or standalone application)
    '''
    
    def __init__(self, yamlEntry, type, name, display_name, icon, text, parent=None, parentButton=None):
        self.display_name = display_name
        super(SR2ViewButtonExtProcess, self).__init__(yamlEntry, type, name, icon, text, parent, parentButton)
 
    def setupButton(self):
        self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + ((' ' + self.args) if self.args else '') + '"' + ' inactive'
        setupPushButton(self)
        
    #adapt the style of this button to the current status of execution (aka disabled, running, failed)
    def setStatusStyle(self, status):
        self.button.setStyleSheet(pushButtonStatusStyle(self, status))

##########################################################################
# SR2ToolbarButtonService
##########################################################################
# TODO Add error confirm to service buttons (toolbar and view type) like
# with the external process buttons


class SR2ButtonService(SR2ButtonDefault):
    '''
    Part of a toolbar; initiates a service call and reports back once service has replied or timeout
    '''
    params = ''


    toggle_params = None #will be set in "SR2ButtonDefault.parse_yaml" if necessary

    def __init__(self, yamlEntry, name, icon, text, parent=None, minimal=True, parentButton=None):
      
        super(SR2ButtonService, self).__init__(yamlEntry, 'service', name, icon, text, parent, parentButton)
        
        self.minimal = minimal

        self.tooltip = self.name + ' : "' + 'rosservice call' + \
            ' ' + self.srv_name + '"<br/>Reply: --'
        self.setToolTip(self.tooltip)

        self.thread_pool = QThreadPool()
        if self.type == 'dynamic_reconfigure':
            self.service = SR2DynamicReconfigureServiceRunnable(self.srv_name, self.params, self.timeout)
            if self.toggle_params:
                self.toggle_service = SR2DynamicReconfigureServiceRunnable(self.srv_name, self.toggle_params, self.timeout)
                self.toggle_service.setAutoDelete(False)
                self.toggle_service.signals.srv_running.connect(self.block)
                self.toggle_service.signals.srv_status.connect(self.reply)
        else:
            msg_type = self.type
            try:
              msg_type = roslib.message.get_service_class(self.type)
              if msg_type is None:
                rospy.logwarn('msg_type still not set. why?! ' + self.type)
            except:
              try:
                msg_type = roslib.message.get_service_class('std_msgs/'+self.type)
              except:
                rospy.logerr('SR2: message type is missing package name: '+self.type)
            if msg_type is None:
                msg_type = self.type #workarround that will find its conclusion in SR2ServiceRunnable
            self.service = SR2ServiceRunnable(self.srv_name, self.timeout, msg_type, self.params)
        self.service.setAutoDelete(False)
        self.service.signals.srv_running.connect(self.block)
        self.service.signals.srv_status.connect(self.reply)
        #self.button.clicked.connect(self.call)

        self.disabled = False
        self.toggled = False

    @pyqtSlot()
    def call(self):
        '''
        If button is enabled, initiate a service call
        '''
        
        self.setStatusStyle()

        #if hasinit:
        #    if self.init_block_enabled:
        #        rospy.logerr(
        #            'SR2: Init ext.process is not running. Unable to control ext.process connected to this button')
        #        return

        if not self.statusOkay:
            # If an error has occurred the first thing the user has to do is
            # reset the state by acknowleding the error
            self.button.setToolTip('Service call "' + self.srv_name + '" with timeout  ' + str(self.timeout) + 's inactive')
            self.statusOkay = True
            rospy.loginfo('SR2: Error acknowledged')
            return

        if not self.disabled:
            if not self.toggled:
                rospy.loginfo('SR2: Calling service %s from thread %d with timeout set to %d', self.srv_name, int(QThread.currentThreadId()), self.timeout)
                self.thread_pool.start(self.service)
                if self.toggle_params:
                    self.toggled = True
            else:
                rospy.loginfo('SR2: Calling service %s from thread %d with timeout set to %d and toggle parameters', self.srv_name, int(QThread.currentThreadId()), self.timeout)
                self.thread_pool.start(self.toggle_service)
                self.toggled = False
        else:
            rospy.loginfo(
                'SR2: Service call is currently being processed. Please wait...')
            self.disabled = True

    #adapt the style of this button to the current status of execution (aka disabled, running, failed)
    def setStatusStyle(self):
        if self.disabled:
            style = toolButtonStyle(self.icon, self.text, 89, 205, 139)
            self.setStyleSheet(style)

    @pyqtSlot(bool)
    def block(self, state):
        '''
        Disables button from futher interaction while service is being called (or until timeout occurs)
        '''
        if state:
            self.setStatusStyle()
            # FIXME Currently after running a call the "is being processed" is
            # still attached even though the call is inactive
            self.button.setToolTip('Service call "' + self.srv_name + '" with timeout ' + str(self.timeout) + 's is being processed...')
#      self.message.setText('<nobr>' + self.name + ' : "rosservice call ' + self.srv_name + '"</nobr><br/>Status: Running...')
            self.button.setDisabled(True)

        self.disabled = state

    @pyqtSlot(bool)
    def block_override(self, block_override_flag):
        '''
        If connected to an init entry this slot will disable the interaction with the button if the init external process isn't running

        :param block_override_flag: enables/disable click action of button
        '''
        self.init_block_enabled = block_override_flag

    @pyqtSlot(int, str)
    def reply(self, status, msg):
        '''
        Based on the success status returned by the service call (SUCCESS_TRUE, SUCCESS_FALSE, FAILED)
        the button is enabled and changes its icon while the test of the status label displays information on how the service call went
        '''
        style = ''
        if status in [SR2ServiceRunnable.CallStatus.FAILED, SR2ServiceRunnable.CallStatus.SUCCESS_FALSE]:
            style = toolButtonStyle(self.icon, self.text, 215, 56, 56)
            self.statusOkay = False
            rospy.logerr('SR2: Calling service %s failed due to "%s"',
                         self.srv_name, msg)
            if self.parentButton:
                try:
                    self.parentButton.reply(False)
                except Exception as e:
                    rospy.logerr('SR2: Replying to parent failed due to: ' + e.message)
        else:
            if not self.toggled:
                style = toolButtonStyle(self.icon, self.text, 89, 205, 139)
                rospy.loginfo('SR2: Calling service %s successful. Service returned status %s with message "%s"',
                              self.srv_name, ('True' if not status else 'False'), msg)
            else: 
                style = toolButtonStyle(self.icon, self.text, 89, 205, 139, 6, 8)
                rospy.loginfo('SR2: Calling service %s successful. Service returned status %s with message "%s"',
                              self.srv_name, ('True' if not status else 'False'), msg)
            if self.parentButton:
                try:
                    self.parentButton.reply(True)
                except Exception as e:
                    rospy.logerr('SR2: Replying to parent failed due to: ' + e.message)

        self.tooltip = '<nobr>' + self.name + ' : "rosservice call ' + \
            self.srv_name + '"</nobr><br/>Reply: ' + msg

        self.setToolTip(self.tooltip)
        self.setStyleSheet(style) 


##########################################################################
# SR2ViewButtonService
##########################################################################
class SR2ViewButtonService(SR2ButtonService): #cannot inherit from SR2ButtonService because of widget functionality vs button functionality (?)
    '''
    Part of a view; initiates a service call and reports back once service has replied or timeout
    '''
    def __init__(self, yaml_entry_data, name, display_name, icon, text, parent=None, parentButton=None):
        super(SR2ViewButtonService, self).__init__(yaml_entry_data, name, icon, text, parent, True, parentButton)
     
    def setupButton(self):
        self.tooltip = 'Service call "' + self.srv_name + '" with timeout  ' + str(self.timeout) + 's inactive'
        self = setupPushButton(self, 'service')
    
    #adapt the style of this button to the current status of execution (aka disabled, running, failed)
    def setStatusStyle(self):
        if not self.statusOkay:
            style = pushButtonStyle(self.icon, self.text)
            self.button.setStyleSheet(style)
        if not self.disabled:
            self.reply_statL.setText('Reply status:')
            self.reply_msgL.setText('Reply message:')
        else:
            style = pushButtonStyle(self.icon, self.text, 89, 205, 139)
            self.button.setStyleSheet(style)

    @pyqtSlot(int, str)
    def reply(self, status, msg):
        '''
        Based on the success status returned by the service call (SUCCESS_TRUE, SUCCESS_FALSE, FAILED)
        the button is enabled and changes its icon while the test of the status label displays information on how the service call went
        '''
        style = ''
        if status in [SR2ServiceRunnable.CallStatus.FAILED, SR2ServiceRunnable.CallStatus.SUCCESS_FALSE]:
            style = pushButtonStyle(self.icon, self.text, 215, 56, 56)
            rospy.logerr('SR2: Calling service %s failed due to "%s"', self.srv_name, msg)
            self.reply_statL.setText('Reply status: <font color=\"red\">Error</font>')
            self.reply_msgL.setText('Reply message: <font color=\"red\">' + msg.split('for service', 1)[0] + '</font>')
            self.statusOkay = False
            self.button.setToolTip('Service call "' + self.srv_name + '" with timeout  ' + str(self.timeout) + 's failed')
            if self.parentButton:
                self.parentButton.reply(False)
            
        else:
            if not self.toggled: 
                style = pushButtonStyle(self.icon, self.text, 89, 205, 139)
            else: 
                style = pushButtonStyle(self.icon, self.text, 89, 205, 139, 24, 32)
                
            rospy.loginfo('SR2: Calling service %s successful. Service returned status %s with message "%s"',
                          self.srv_name, ('True' if not status else 'False'), msg)
            self.reply_statL.setText('Reply status: ' + ('True' if status ==
                                                         SR2ServiceRunnable.CallStatus.SUCCESS_TRUE else 'False'))
            self.reply_msgL.setText('Reply message: ' + msg)
            self.button.setToolTip('Service call "' + self.srv_name + '" with timeout  ' + str(self.timeout) + 's successful')
            if self.parentButton:
                self.parentButton.reply(True)

        self.button.setStyleSheet(style)
        self.button.setDisabled(False)

##########################################################################
# SR2ToolbarButtonPublisher
##########################################################################

class SR2ButtonPublisher(SR2ButtonDefault):
  
    def __init__(self, yaml_content, name, icon, text, parent = None, parent_button = None):
        super(SR2ButtonPublisher, self).__init__(yaml_content, 'publisher', name, icon, text, parent, parent_button)
        self.setup_publisher()
        self.setup_message()
        
    def setupButton(self):
        self.tooltip = self.name + ' : "' + 'publish message' + ' '
        self = setupToolButton(self)
        
    def setup_publisher(self):
        pub_class = roslib.message.get_message_class(self.message_type)
        self.publisher = rospy.Publisher(self.topic, pub_class, queue_size=1)
        
    def setup_message(self):
        msg_class = self.message_type
        message = self.message_content
        self.message = message_converter.convert_dictionary_to_ros_message(msg_class, message)
        if hasattr(msg_class, 'header'):
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            message.header = h
        
    def call(self):
        rospy.loginfo('called publisher ' + self.topic)
        rospy.sleep(1)
        self.publisher.publish(self.message)

    @pyqtSlot(bool)
    def block_override(self, block_override_flag):
        '''
        If connected to an init entry this slot will disable the interaction with the button if the init external process isn't running

        :param block_override_flag: enables/disable click action of button
        '''
        self.init_block_enabled = block_override_flag

        
##########################################################################
# SR2ViewButtonPublisher
##########################################################################

class SR2ViewButtonPublisher(SR2ButtonPublisher):
      
      def __init__(self, yaml_content, name, icon, text, parent = None, parent_button = None):
        super(SR2ViewButtonPublisher, self).__init__(yaml_content, name, icon, text, parent, parent_button)

      def setupButton(self):
        self.tooltip = self.name + ' : "' + 'publish message' + ' '
        self = setupPushButton(self, 'publisher')

##########################################################################
# SR2ToolbarButtonMulti
##########################################################################

class SR2ButtonMulti(SR2ButtonDefault):
    def __init__(self, yaml_content, name, icon, text, parent = None, parent_button = None):
        super(SR2ButtonMulti, self).__init__(yaml_content, 'multi', name, icon, text, parent, parent_button)
        
        self.entries = []
        
        idx = 0
        for yaml_content_entry in yaml_content:
            type, icon_, text_ = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_content_entry)
            entry = SR2Button.createButton(None, yaml_content_entry, name + str(idx), None, None, False, None, self)
            if not entry:
                continue
            entry.init_block_enabled = False
            self.entries.append((type, entry))
            idx += 1
            
            #self.button.clicked.connect(self.call)
            
    #call all tasks that are linked to this button
    def call(self):
        
        if self.statusOkay:
            self.replies = 0
            for entry in self.entries:
                entry[1].call()
        else:
            self.statusOkay = True
            rospy.loginfo('SR2: Error acknowledged')
            self.setStatusStyle(ProcStatus.INACTIVE)
                
    #called by child processes on completion/failure
    def reply(self, statusOkay):
        #count replies (wait for replies from all processes)
        self.replies += 1
        #if one reply is negative, concider whole interaction failed
        if not statusOkay:
            self.statusOkay = False
        #wait for replies from all processes
        if self.replies < len(self.entries):
            return
        #
        if not self.statusOkay:
            for entry in self.entries:
                entry[1].call()
                if entry[0] == 'service':
                    entry[1].statusOkay = True
                else:
                    entry[1].status = ProcStatus.FINISHED
            self.replies = 0 #only perform when a failure is Detected and consequences are taken
        #in total => if all processes have ever replied and then one dies at any time, kill (by  toggling) them all => if one process fails on startup, wait for all to start and kill them all, if one fails after all have started, kill them all.
        if self.parentButton:
            self.parentButton.reply(self.statusOkay)
            
        if self.statusOkay:
            self.setStatusStyle(ProcStatus.RUNNING)
        else:
            if self.replies == len(self.entries):
                self.setStatusStyle(ProcStatus.FAILED_START)
            else:
                self.setStatusStyle(ProcStatus.FAILED_STOP)
            
    def setupButton(self):
        self.tooltip = 'Execute multiple commands'
        self = setupToolButton(self)
        
    #adapt the style of this button to the current status of execution (aka disabled, running, failed)
    def setStatusStyle(self, status):
        self.button.setStyleSheet(toolButtonStatusStyle(self, status))
        
    @pyqtSlot(bool)
    def block_override(self, block_override_flag):
        '''
        If connected to an init entry this slot will disable the interaction with the button if the init external process isn't running

        :param block_override_flag: enables/disable click action of button
        '''
        self.init_block_enabled = block_override_flag
            
class SR2ViewButtonMulti(SR2ButtonMulti):
    def setupButton(self):
        self.tooltip = 'Execute multiple commands'
        self = setupPushButton(self, 'multi')
        
    #adapt the style of this button to the current status of execution (aka disabled, running, failed)
    def setStatusStyle(self, status):
        self.button.setStyleSheet(pushButtonStatusStyle(self, status))

##########################################################################
# SR2ButtonKill
##########################################################################

class SR2ButtonKill(SR2ButtonDefault):
    '''
    Part of a toolbar; kills a task
    '''
    
    kill = '' #task to kill

    def __init__(self, yamlEntry, name, icon, text, parent=None, parentButton=None):
        super(SR2ButtonKill, self).__init__(yamlEntry, 'kill', name, icon, text, parent, parentButton)

    def setupButton(self):
        self.tooltip = self.name + ' : "' + 'rosnode kill' + ' ' + self.kill
        self = setupToolButton(self)
    
    @pyqtSlot()
    def call(self):
        
        call(['rosnode', 'kill', self.kill])

        if self.parentButton:
            self.parentButton.reply(True) #TODO any way to really check if it worked?
            
    @pyqtSlot(bool)
    def block_override(self, block_override_flag):
        '''
        If connected to an init entry this slot will disable the interaction with the button if the init external process isn't running

        :param block_override_flag: enables/disable click action of button
        '''
        self.init_block_enabled = block_override_flag


##########################################################################
# SR2ViewButtonKill
##########################################################################

class SR2ViewButtonKill(SR2ButtonKill): #cannot inherit from SR2ButtonService because of widget functionality vs button functionality (?)
    '''
    Part of a view; kills a task
    '''
    def __init__(self, yaml_entry_data, name, icon, text, parent=None, parentButton=None):
        super(SR2ViewButtonKill, self).__init__(yaml_entry_data, name, icon, text, parent, parentButton)
     
    def setupButton(self):
        self.tooltip = self.name + ' : "' + 'rosnode kill' + ' ' + self.kill
        self = setupPushButton(self, 'kill')

##########################################################################
# SR2ToolbarButtonWithView
##########################################################################


class SR2ButtonWithView(QToolButton):
    '''
    Part of a toolbar; opens a view
    '''
    class SR2View(QWidget):

        def __init__(self, name, yaml_button_list, parent=None, init_widget=None):
            super(SR2ButtonWithView.SR2View, self).__init__(parent)

            name = name + ' View'
            self.setObjectName(name)
            self.setWindowTitle(name)

            rospy.logdebug('SR2: Creating view for %s', yaml_button_list)

            grid = QGridLayout()
            #grid.setMargin(20) Qt4
            #grid.setContentsMargins(20) #TODO not enough arguments, how to resolve?
            grid.setContentsMargins(5, 5, 5, 5)

            self.buttons = []
            idx = 0
            for yaml_button_entry in yaml_button_list:
                display_name = None
                button = SR2Button.createButton(
                    None, yaml_button_entry, name + str(idx), display_name, self)
                if not button:
                    continue
                self.buttons.append(button)
                idx += 1

            # Get dimensions of grid based on number of VALID buttons after the
            # YAML entries have been parsed (or failed to)
            (self.rows, self.cols) = sr2gg.get_dim(len(self.buttons))
            rospy.logdebug('SR2: View contains %d buttons which will be distributed on a %d by %d (rows by columns) grid layout' % (
                len(yaml_button_list), self.rows, self.cols))

            positions = []
            for r in range(0, self.rows):
                for c in range(0, self.cols):
                    positions.append([r, c])

            idx = 0
            for pos in positions:
                try:
                    # There is a maximum of a single cell-gap (for example for 3 buttons we generate a 2x2 grid with 4 cells one of which would remain empty)
                    # that needs to be handle that will cause an exception to
                    # be thrown
                    grid.addWidget(self.buttons[idx], pos[0], pos[1])
                    idx += 1
                except:
                    break

            self.setLayout(grid)

        def shutdown(self):
            pass

        def save_settings(self, plugin_settings, instance_settings):
            pass

        def restore_settings(self, plugin_settings, instance_settings):
            pass

    def __init__(self, yaml_entry_data, child_type, name, icon, context, surpress_overlays=False, minimal=True):
        super(SR2ButtonWithView, self).__init__()

        self.clicked.connect(self.toggleView)

        self.icon = icon

        self.child_type = child_type

        if self.child_type:
            if self.child_type == 'service':
                self.virtual_button = SR2ButtonService(yaml_entry_data['service'], name, self.icon, '', None, True, self)
            else:
                self.virtual_button = SR2ButtonExtProcess(yaml_entry_data[child_type], child_type, name, self.icon, '', None, self)
                
            self.virtual_button.block_override(False)
            self.virtual_button.init_block_enabled = False

        style = toolButtonStyle(self.icon, self.text)
        self.setStyleSheet(style)

        try:
            # Buttons contains a list of YAML buttons inside the menu_entry of
            # the view node. This list is used for populating the view with
            # components
            #rospy.logwarn(yaml_entry_data['buttons'])
            self.yaml_view_buttons = yaml_entry_data['buttons']
            if not self.yaml_view_buttons:
                raise Exception('Empty button list')
        except (YAMLError, Exception):
            rospy.logerr(
                'SR2: Detected menu entry with view which does not contain any buttons. Generated view will be empty')

#    self._plugin_settings = None
#    self._instance_settings = None
        self.view_widget = None
        self.minimal = minimal
        self.name = name
        self.toggled = False
        self.context = context
        self.setToolTip(self.name)
        self.setFixedSize(QSize(36, 36))
        #self.setFixedSize(QSize(80, 80))
        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Maximum)


        self.close_mutex = QMutex()
        self.show_mutex = QMutex()

        self.init_block_enabled = True

    @pyqtSlot(bool)
    def block_override(self, block_override_flag):
        '''
        If connected to an init entry this slot will disable the interaction with the button if the init external process isn't running

        :param block_override_flag: enables/disable click action of button

        Currently blocking the view's components is NOT supported
        '''
        self.init_block_enabled = block_override_flag

    def showException(self, e):
        
        if not self.view_widget:
              # self.setIcon(self._icons[IconType.error])
              style = toolButtonStyle(self.icon, self.text, 215, 56, 56)
              # rospy.logwarn(self.yaml_view_buttons)
              rospy.logerr(
                  'SR2: Error during showing SR2View : %s', e.message)
              rospy.logerr(self.name)
              rospy.logerr(self.yaml_view_buttons)
              self.toggled = False

    def reply(self, status):
      
        if status:
            self.showView()
        else:
            self.hideView()


    def showView(self):
      
        try:
            self.toggled = True
            # self.setIcon(self._icons[IconType.running])
            style = toolButtonStyle(self.icon, self.text, 89, 205, 139)
            rospy.logdebug('SR2: Added SR2View "%s"', self.name)
            self.view_widget = SR2ButtonWithView.SR2View(
            self.name, self.yaml_view_buttons)
            self.context.add_widget(self.view_widget)
        except Exception as e:
            self.showException(e)
            style = toolButtonStyle(self.icon, self.text, 215, 56, 56)
        finally:
            self.setStyleSheet(style)

    def hideView(self):
      
        try:
            self.toggled = False
            self.context.remove_widget(self.view_widget)
            self.close()
            #self.setIcon(self._icons[IconType.inactive])
            style = toolButtonStyle(self.icon, self.text)
            rospy.logdebug('SR2: Closed SR2View "%s"', self.name)
        except Exception as e:
            self.showException(e)
        finally:
            self.setStyleSheet(style)

    def toggleView(self):
        '''
        Toggles the visibility of the view (if such exists) that is connected to the menu entry
        '''
        
        # If this functionality is used, execute function of virtual_button
        
        if self.child_type:
            self.virtual_button.call()
            return

        # Sadly the way the views in the dashboard work doesn't allow
        # for a view's components to emit feedback to the menu entry
        # since those are destroyed every  time the view is hidden thus
        # only the menu entry remains...OR MAYBE NOT XD

        global hasinit
        #if hasinit:
        #    rospy.logout('hasinit')
        #    if not self.init_block_enabled:
        #        rospy.logerr('SR2: Init ext.process is not running. Unable to control ext.process connected to this button')
        #        return

        if not self.yaml_view_buttons:
            return

        with QMutexLocker(self.show_mutex):
            style = ''
            if self.toggled:
                # If menu entry is already displaying a view, remove it
                self.hideView()
            else:
                # If menu entry doesn't display a view, create it and
                # display it
                self.showView()

    def close(self):
        '''
        Unloads the plugin from memory
        '''
        if self.toggled:
            with QMutexLocker(self.close_mutex):
                #        if self._plugin_settings:
                #          self.view_widget.save_settings(self._plugin_settings,
                #                                         self._instance_settings)
                if self.view_widget:
                    self.view_widget.shutdown()
                    self.view_widget.close()
                    self.view_widget = None

    def save_settings(self, plugin_settings, instance_settings):
        #    if self.toggled:
        #      self.view_widget.save_settings(self._plugin_settings,
        #                            self._instance_settings)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


##########################################################################
# SR2ButtonInit*
##########################################################################

class SR2ButtonInitService(SR2ButtonService):
  
    toggle_params = None #will be set in "SR2ButtonDefault.parse_yaml" if necessary

    def __init__(self, yamlEntry, name, icon, text, parent=None, minimal=True, parentButton=None):
      
        super(SR2ButtonInitService, self).__init__(yamlEntry, name, icon, text, parent, minimal, parentButton)
        
        super(SR2ButtonInitService, self).call()

  
class SR2ButtonInitPublisher(SR2ButtonPublisher):
    
    def __init__(self, yamlEntry, name, icon, text, parent = None, parent_button = None):
    
      super(SR2ButtonInitPublisher,self).__init__(yamlEntry, name, icon, text, parent, parent_button)
  
      super(SR2ButtonInitPublisher,self).call()
      
      
class SR2ButtonInitMulti(SR2ButtonMulti):
  
    def __init__(self, yamlEntry, name, icon, text, parent = None, parent_button = None):
      
      super(SR2ButtonInitMulti,self).__init__(yamlEntry, name, icon, text, parent, parent_button)
      
      super(SR2ButtonInitMulti,self).call()
      
      
class SR2ButtonInitKill(SR2ButtonKill):
    
    def __init__(self, yamlEntry, name, icon, text, parent = None, parent_button = None):
      
      super(SR2ButtonInitKill,self).__init__(yamlEntry, name, icon, text, parent, parent_button)
      
      super(SR2ButtonInitKill,self).call()
      

class SR2ButtonInitExtProcess(SR2ButtonExtProcess):

    def __init__(self, yamlEntry, type, name, icon, text, parent=None):
      
        super(SR2ButtonInitExtProcess, self).__init__(yamlEntry, type, name, icon, text, parent)
        
        super(SR2ButtonInitExtProcess, self).createWorker()
        super(SR2ButtonInitExtProcess, self).call()
