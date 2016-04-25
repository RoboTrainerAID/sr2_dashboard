# Author: Aleksandar Vladimirov Atanasov
# Description: A SR2 buttons (for both toolbar and view widgets)

# YAML
from yaml import YAMLError

# PyQt
# QtGui modules
from python_qt_binding.QtGui import QPushButton, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QSpacerItem, QSizePolicy, QFrame, QToolButton
# QtCore modules
from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer, QThread, pyqtSlot, pyqtSignal, QThreadPool, QSize
# from PyQt4.QtSvg import QSvgRenderer # TODO Add support for SVG for the
# icons and pixmaps: http://stackoverflow.com/a/35138314/1559401 |
# http://www.qtcentre.org/threads/7321-Loading-SVG-icons

import roslib
roslib.load_manifest('sr2_dashboard')
import rospy
# ROS-related  modules: RQT Robot Dashboard
from rqt_robot_dashboard.icon_tool_button import IconToolButton
# from rqt_robot_dashboard.widgets import ...

from ..misc.sr2_monitor_object import SR2Worker, ProcStatus
from ..misc.sr2_runnable_object import SR2ServiceRunnable
from ..misc.sr2_grid_generator import SR2GridGenerator as sr2gg
from ..misc.sr2_ros_entry_extraction import SR2PkgCmdExtractor, IconType

# TODO Add button name to PID file to make workers for even same commands plus same arguments produce different files based on which part of the UI (toolbar, view) they belong to
# TODO Replace IconToolButton with own, add parent attribute in order to properly clean up files and objects; Add stylesheet stuff to the custom QToolButton
# TODO Pass init-component to view and view-internal widgets to block - currently not possible since all content of a view is destroyed whenever the view is hidden -> perhaps just show/hide view instead of destroying
# these two if init ext.process is not running


class Status():
    '''
    Remaps the IconType values for better code semantics and readability
    '''
    inactive = IconType.inactive
    running = IconType.running
    error = IconType.error


class SR2Button():

    @staticmethod
    def createButton(context, yaml_entry_data, name, display_name=None, parent=None, init=False, init_widget=None):
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
        :param display_name: optional name displayed in view components; if not present, display name is automatically generated; name is also only then visible whenever default icon is used for a view entry
        '''
        if not yaml_entry_data:
            return None

        pkg = ''
        cmd = ''
        args = ''
        timeout = 0
        icon = ''

        if 'type' in yaml_entry_data:
            # We have an entry that is part of the toolbar
            if yaml_entry_data['type'] == 'noview':
                pkg, cmd, args, icon, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(yaml_entry_data[
                    'menu_entry'])
                if (not cmd and not pkg and not args and not timeout):
                    return None
                rospy.logdebug('\n----------------------------------\n\tCREATE TOOLBAR BUTTON\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n\t@Timeout: %d\n----------------------------------', name, cmd, args, pkg, timeout)
                if timeout:
                    # Service
                    if not args:
                        rospy.logerr(
                            'SR2: Trying to create noview service button but service target is empty')
                        return None
                    return SR2ButtonService(name, args, timeout, icon, parent)
                else:
                    # External process (roslaunch, rosrun or app)
                    if init:
                        return SR2ButtonInitExtProcess(name, cmd, pkg, args, icon, parent)
                    else:
                        return SR2ButtonExtProcess(name, cmd, pkg, args, icon, parent)
            elif yaml_entry_data['type'] == 'view':
                # View
                rospy.logdebug(
                    '\n----------------------------------\n\tCREATE TOOLBAR VIEW\n@Yaml_Contents: %s\n----------------------------------', yaml_entry_data)
                return SR2ButtonWithView(name, yaml_entry_data['menu_entry'], context, init_widget)
            else:
                rospy.logerr(
                    'SR2: Unknown type of entry. Please make sure to specify "type" as either "noview" or "view"')
                return None
        else:
            # We have an entry that is part of a view
            pkg, cmd, args, icon, timeout = SR2PkgCmdExtractor.getRosPkgCmdData(
                yaml_entry_data)
            if (not cmd and not pkg and not args and not timeout):
                return None
            rospy.logdebug('\n----------------------------------\n\tCREATE VIEW BUTTON\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n\t@Timeout: %d\n----------------------------------', name, cmd, args, pkg, timeout)
            if timeout:
                # Service
                if not args:
                    rospy.logerr(
                        'SR2: Trying to create noview service button but service target is empty')
                    return None
                return SR2ViewButtonService(name, display_name, args, timeout, icon, parent)
            else:
                # External process (roslaunch, rosrun or app)
                return SR2ViewButtonExtProcess(name, display_name, cmd, pkg, args, icon, parent)

############## QToolButton ############


class SR2ToolButton(QToolButton):

    '''
    TODO Add own version of IconToolButton
    '''

    def __init__(self, name, icon, icon_type):
        pass

##########################################################################
# SR2ButtonExtPro
##########################################################################


class SR2ButtonExtProcess(QToolButton):
    '''
    Part of a toolbar; gives the ability to start/stop and monitor an external process (roslaunch, rosrun or standalone application)
    '''
    start_signal = pyqtSignal()
    stop_signal = pyqtSignal()
    clear_error_signal = pyqtSignal()

    # TODO Replace IconToolButton with own version (add parent property!)
    def __init__(self, name, cmd, pkg, args, icon, parent=None):
        super(SR2ButtonExtProcess, self).__init__()

        rospy.logdebug(
            '\n----------------------------------\n\tEXT.PROC\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n----------------------------------', name, cmd, args, pkg)

        self.icon = icon
        style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
            self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
        self.setStyleSheet(style)
        self.setFixedSize(QSize(36, 36))
#        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Maximum)
        self.setObjectName(name)
        self.name = name
        self.setToolTip(self.name)

        self.cmd = cmd
        self.args = args
        self.pkg = pkg

        self.statusOkay = True  # Used for activating the acknowledgement mode where the user has to confirm the error before trying to launch the process again
        self.active = False    # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
        self.toggleControl = False
        self.setToolTip('Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') +
                        ((' ' + self.args) if self.args else '') + '"' + ' inactive')

        self.clicked.connect(self.toggle)

        self.mutex_recovery = QMutex()
        self.mutex_status = QMutex()

        self.worker_thread = QThread()
        self.createWorker()

        self.init_block_enabled = True

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
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' inactive/finished'
            self.active = False
        elif status == ProcStatus.RUNNING:
            rospy.loginfo('SR2: Status has changed to: RUNNING')
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
            self.tooltip = 'Ext.process "' + self.cmd + \
                ((' ' + self.pkg) if self.pkg else '') + \
                ((' ' + self.args) if self.args else '') + '"' + ' running'
            self.active = True
        elif status == ProcStatus.FAILED_START:
            rospy.logerr('SR2: Status has changed to: FAILED_START')
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(215, 56, 56);}'
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' failed to start'
            self.statusOkay = False
            self.active = True
            self.toggleControl = False
        elif status == ProcStatus.FAILED_STOP:
            rospy.logerr('SR2: Status has changed to: FAILED_STOP')
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(215, 56, 56);}'
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' failed to stop'
            self.statusOkay = False
            self.active = True
            self.toggleControl = False

        self.setToolTip(self.tooltip)
        self.setStyleSheet(style)

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

    @pyqtSlot()
    def toggle(self):
        '''
        Handles the start, stopping and error confirmation triggered by the button

          - **statusOkay is False** - this occurs ONLY if the process status is an error (FAILED_START or FAILED_STOP). In this case the user has to click twice on the button in order to reinitiate the starting procedure
          - **both statusOkay and toggleControl are True** - attempt to start the process
          - **statusOkay is True but toggleControl is False** - attempt to stop the process
        '''
        if self.init_block_enabled:
            rospy.logerr(
                'SR2: Init ext.process is not running. Unable to control ext.process connected to this button')
            return

        if not self.statusOkay:
            # If an error has occurred the first thing the user has to do is
            # reset the state by acknowleding the error
            self.statusOkay = True
            rospy.loginfo('SR2: Error acknowledged')
            self.clear_error_signal.emit()
            return

        self.toggleControl = not self.toggleControl
        if self.toggleControl:
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

##########################################################################
# SR2ButtonInitExt
##########################################################################


class SR2ButtonInitExtProcess(SR2ButtonExtProcess):
    '''
    Part of a toolbar; gives the ability to start/stop and monitor an external process (roslaunch, rosrun or standalone application)
    Represents the init YAML entry used for adding a single critical external process which can enable/disable all other entries (currently view-entries are not supported)
    '''

    block_override = pyqtSignal(bool)  # Connect this to the toolbar component that has to depend on the state of INIT

    def __init__(self, name, cmd, pkg, args, icon, parent=None):
        super(SR2ButtonInitExtProcess, self).__init__(
            name, cmd, pkg, args, parent)
        rospy.logdebug('\n----------------------------------\n\tINIT EXT.PROC\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n----------------------------------', name, cmd, args, pkg)
        self.init_block_enabled = False
        # The block override will override the blocking from the workers of these buttons based on the enable-state of the INIT button
        # TODO If super.state is INACTIVE or FAILED -> emit block_override

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

        In case the init external process isn't running a blocking signal is sent to all components connected to this button in order to disable the interaction with them
        The interaction with other connected buttons is disable until the init external process isn't running again
        '''
        super(SR2ButtonInitExtProcess, self).statusChangedReceived(status)
        if status != ProcStatus.RUNNING:
            rospy.logerr('SR2: Init ext.process stopped running')
            self.block_override.emit(True)
        else:
            rospy.loginfo(
                'SR2: Init ext.process is now running. Connected entries can now be used')
            self.block_override.emit(False)

    @pyqtSlot()
    def recover(self):
        '''
        Sets button in an active mode. Triggered only if recovery of external process was successful
        In addition it also enables all connected components
        '''
        super(SR2ButtonInitExtProcess, self).recover()
        rospy.loginfo(
            'SR2: Recovery successful. Connected to running init ext.process. Connected components will be available to the user')
        self.block_override.emit(False)

#  def changeEvent(self, event):
#    print('Change event')

##########################################################################
# SR2ButtonExtPro
##########################################################################


class SR2ViewButtonExtProcess(QWidget):
    '''
    Part of a view; gives the ability to start/stop and monitor an external process (roslaunch, rosrun or standalone application)
    '''
    start_signal = pyqtSignal()
    stop_signal = pyqtSignal()
    clear_error_signal = pyqtSignal()

    def __init__(self, name, display_name, cmd, pkg, args, icon, parent=None):
        super(SR2ViewButtonExtProcess, self).__init__(parent)

        rospy.logdebug(
            '\n----------------------------------\n\tEXT.PROC\n\t@Name: %s\n\t@Cmd: %s\n\t@Args: %s\n\t@Pkg: %s\n----------------------------------', name, cmd, args, pkg)

        self.setObjectName(name)
        self.name = name
        self.icon = icon
        style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
            self.icon + ') 0 0 0 0 stretch stretch; background: none;}'

        self.cmd = cmd
        self.args = args
        self.pkg = pkg

        layout = QVBoxLayout(self)
        layout.setObjectName(name + 'Layout Ext Proc')

        controls_layout = QHBoxLayout()
        spacer = QSpacerItem(40, 20, QSizePolicy.Preferred,
                             QSizePolicy.Preferred)
        controls_layout.addItem(spacer)
        self.execute_button = QPushButton(self)
        self.execute_button.setStyleSheet(style)
        self.execute_button.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
            (' ' + self.args) if self.args else '') + '"' + ' inactive'
        self.execute_button.setToolTip(self.tooltip)
        controls_layout.addWidget(self.execute_button)
        spacer = QSpacerItem(40, 20, QSizePolicy.Preferred,
                             QSizePolicy.Preferred)
        controls_layout.addItem(spacer)
        layout.addLayout(controls_layout)

        if 'default_' in self.icon:
            info_layout = QVBoxLayout()
            service_nameL = QLabel(self)
            if self.cmd not in ['roslaunch', 'rosrun']:
                if display_name:
                    service_nameL.setText('App: ' + display_name)
                else:
                    service_nameL.setText('App: ' + self.cmd)
            else:
                if display_name:
                    service_nameL.setText('External process: ' + display_name)
                else:
                    service_nameL.setText('External process: ' + self.args.replace('.launch', ''))
            service_nameL.setWordWrap(True)
            info_layout.addWidget(service_nameL)
            line = QFrame(self)
            line.setFrameShape(QFrame.HLine)
            line.setFrameShadow(QFrame.Sunken)
            info_layout.addWidget(line)
            layout.addLayout(info_layout)

        self.statusOkay = True  # Used for activating the acknowledgement mode where the user has to confirm the error before trying to launch the process again
        self.active = False    # Whenever button is clicked and a process is launched successfully self.active is set to True until status is received that process is no longer running | this variable is used to deactivate the start-trigger
        self.toggleControl = False

        self.mutex_recovery = QMutex()
        self.mutex_status = QMutex()

        self.worker_thread = QThread()
        self.createWorker()
        self.execute_button.clicked.connect(self.toggle)

        self.setLayout(layout)
        self.resize(layout.sizeHint())

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
        style = ''
        if status == ProcStatus.INACTIVE or status == ProcStatus.FINISHED:
            rospy.loginfo('SR2: Status has changed to: INACTIVE/FINISHED')
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' inactive/finished'
            self.active = False
        elif status == ProcStatus.RUNNING:
            rospy.loginfo('SR2: Status has changed to: RUNNING')
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
            self.tooltip = 'Ext.process "' + self.cmd + \
                ((' ' + self.pkg) if self.pkg else '') + \
                ((' ' + self.args) if self.args else '') + '"' + ' running'
            self.active = True
        elif status == ProcStatus.FAILED_START:
            rospy.logerr('SR2: Status has changed to: FAILED_START')
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(215, 56, 56);}'
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' failed to start'
            self.statusOkay = False
            self.active = True
            self.toggleControl = False
        elif status == ProcStatus.FAILED_STOP:
            rospy.logerr('SR2: Status has changed to: FAILED_STOP')
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(215, 56, 56);}'
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' failed to stop'
            self.statusOkay = False
            self.active = True
            self.toggleControl = False

        self.execute_button.setToolTip(self.tooltip)
        self.execute_button.setStyleSheet(style)

    @pyqtSlot(bool)
    def block(self, block_flag):
        '''
        Enable/Disable the button which starts/stops the external process
        This slot is used for preventing the user to interact with the UI while starting/stopping the external process after a start/stop procedure has been initiated
        After the respective procedure has been completed the button will be enabled again

        :param block_flag: enable/disable flag for the button
        '''
        self.execute_button.setDisabled(block_flag)

#    @pyqtSlot(bool)
#    def block_override(self, block_override_flag):
#        '''
#        If connected to an init entry this slot will disable the interaction with the button if the init external process isn't running
#
#        :param block_override_flag: enables/disable click action of button
#
#        Currently blocking the view's components is NOT supported
#        '''
#        self.init_block_enabled = block_override_flag

    @pyqtSlot()
    def toggle(self):
        '''
        Handles the start, stopping and error confirmation triggered by the button

          - **statusOkay is False** - this occurs ONLY if the process status is an error (FAILED_START or FAILED_STOP). In this case the user has to click twice on the button in order to reinitiate the starting procedure
          - **both statusOkay and toggleControl are True** - attempt to start the process
          - **statusOkay is True but toggleControl is False** - attempt to stop the process
        '''
        if not self.statusOkay:
            # If an error has occurred the first thing the user has to do is
            # reset the state by acknowleding the error
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
            self.execute_button.setStyleSheet(style)
            self.tooltip = 'Ext.process "' + self.cmd + ((' ' + self.pkg) if self.pkg else '') + (
                (' ' + self.args) if self.args else '') + '"' + ' inactive/finished'
            self.execute_button.setToolTip(self.tooltip)
            self.statusOkay = True
            rospy.loginfo('SR2: Error acknowledged')
            self.clear_error_signal.emit()
            return

        self.toggleControl = not self.toggleControl
        if self.toggleControl:
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

##########################################################################
# SR2ToolbarButtonSe
##########################################################################
# TODO Add error confirm to service buttons (toolbar and view type) like
# with the external process buttons


class SR2ButtonService(QToolButton):
    '''
    Part of a toolbar; initiates a service call and reports back once service has replied or timeout
    '''

    def __init__(self, name, args, timeout, icon, parent=None, minimal=True):
        super(SR2ButtonService, self).__init__()

        self.icon = icon
        style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
            self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
        self.setStyleSheet(style)
        self.setFixedSize(QSize(36, 36))
#        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Maximum)
        self.minimal = minimal
        self.name = name
        self.timeout = timeout
        self.args = args  # Args contains the Trigger service that we want to call

        rospy.logdebug(
            '\n----------------------------------\n\tSERVICE\n\t@Name: %s\n\t@Args: %s\n\t@Timeout: %d\n----------------------------------', name, args, timeout)

        self.tooltip = self.name + ' : "' + 'rosservice call' + \
            ' ' + self.args + '"<br/>Reply: --'
        self.setToolTip(self.tooltip)

        self.thread_pool = QThreadPool()
        self.service = SR2ServiceRunnable(self.args, self.timeout)
        self.service.setAutoDelete(False)
        self.service.signals.srv_running.connect(self.block)
        self.service.signals.srv_status.connect(self.reply)
        self.clicked.connect(self.call)

        self.disabled = False
        self.init_block_enabled = True
        self.statusOkay = True

    @pyqtSlot()
    def call(self):
        '''
        If button is enabled, initiate a service call
        '''
        if self.init_block_enabled:
            rospy.logerr(
                'SR2: Init ext.process is not running. Unable to control ext.process connected to this button')
            return

        if not self.statusOkay:
            # If an error has occurred the first thing the user has to do is
            # reset the state by acknowleding the error
            self.statusOkay = True
            rospy.loginfo('SR2: Error acknowledged')
            return

        if not self.disabled:
            rospy.loginfo('SR2: Calling service %s from thread %d with timeout set to %d',
                          self.args, int(QThread.currentThreadId()), self.timeout)
            self.thread_pool.start(self.service)
        else:
            rospy.loginfo(
                'SR2: Service call is currently being processed. Please wait...')
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
            self.setStyleSheet(style)
            self.disabled = True

    @pyqtSlot(bool)
    def block(self, state):
        '''
        Disables button from futher interaction while service is being called (or until timeout occurs)
        '''
        if state:
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
            self.setStyleSheet(style)

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
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(215, 56, 56);}'
            self.statusOkay = False
            rospy.logerr('SR2: Calling service %s failed due to "%s"',
                         self.args, msg)
        else:
            style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
            rospy.loginfo('SR2: Calling service %s successful. Service returned status %s with message "%s"',
                          self.args, ('True' if not status else 'False'), msg)

        self.tooltip = '<nobr>' + self.name + ' : "rosservice call ' + \
            self.args + '"</nobr><br/>Reply: ' + msg

        self.setToolTip(self.tooltip)
        self.setStyleSheet(style)


##########################################################################
# SR2ViewButtonS
##########################################################################
class SR2ViewButtonService(QWidget):
    '''
    Part of a view; initiates a service call and reports back once service has replied or timeout
    '''
    @pyqtSlot()
    def call(self):
        '''
        If button is enabled, initiate a service call
        '''
        style = ''
        if not self.statusOkay:
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
            self.service_caller.setStyleSheet(style)
            self.service_caller.setToolTip(
                'Service call "' + self.args + '" with timeout  ' + str(self.timeout) + 's inactive')
            self.statusOkay = True
            rospy.loginfo('SR2: Error acknowledged')
            return

        if not self.disabled:
            self.reply_statL.setText('Reply status:')
            self.reply_msgL.setText('Reply message:')
            rospy.loginfo('SR2: Calling service %s with timeout %d',
                          self.args, self.timeout)
            self.thread_pool.start(self.service)
        else:
            rospy.loginfo(
                'SR2: Service call is currently being processed. Please wait...')
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
            self.service_caller.setStyleSheet(style)
            self.disabled = True

        self.service_caller.setStyleSheet(style)

    @pyqtSlot(bool)
    def block(self, state):
        '''
        Disables button from futher interaction while service is being called (or until timeout occurs)
        '''
        if state:
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
            self.service_caller.setStyleSheet(style)
            # FIXME Currently after running a call the "is being processed" is
            # still attached even though the call is inactive
            self.service_caller.setToolTip(
                'Service call "' + self.args + '" with timeout ' + str(self.timeout) + 's is being processed...')
#      self.message.setText('<nobr>' + self.name + ' : "rosservice call ' + self.args + '"</nobr><br/>Status: Running...')
            self.service_caller.setDisabled(True)

        self.disabled = state

    @pyqtSlot(int, str)
    def reply(self, status, msg):
        '''
        Based on the success status returned by the service call (SUCCESS_TRUE, SUCCESS_FALSE, FAILED)
        the button is enabled and changes its icon while the test of the status label displays information on how the service call went
        '''
        style = ''
        if status in [SR2ServiceRunnable.CallStatus.FAILED, SR2ServiceRunnable.CallStatus.SUCCESS_FALSE]:
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(215, 56, 56);}'
            rospy.logerr(
                'SR2: Calling service %s failed due to "%s"', self.args, msg)
            self.reply_statL.setText(
                'Reply status: <font color=\"red\">Error</font>')
            self.reply_msgL.setText(
                'Reply message: <font color=\"red\">' + msg.split('for service', 1)[0] + '</font>')
            self.statusOkay = False
            self.service_caller.setToolTip(
                'Service call "' + self.args + '" with timeout  ' + str(self.timeout) + 's failed')
        else:
            style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
                self.icon + \
                    ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
            rospy.loginfo('SR2: Calling service %s successful. Service returned status %s with message "%s"',
                          self.args, ('True' if not status else 'False'), msg)
            self.reply_statL.setText('Reply status: ' + ('True' if status ==
                                                         SR2ServiceRunnable.CallStatus.SUCCESS_TRUE else 'False'))
            self.reply_msgL.setText('Reply message: ' + msg)
            self.service_caller.setToolTip(
                'Service call "' + self.args + '" with timeout  ' + str(self.timeout) + 's successful')

        self.service_caller.setStyleSheet(style)
        self.service_caller.setDisabled(False)

    def __init__(self, name, display_name, args, timeout, icon, parent=None):
        super(SR2ViewButtonService, self).__init__(parent)

        self.icon = icon
        self.args = args
        self.timeout = timeout
        self.name = name
        self.setObjectName(name)

        layout = QVBoxLayout(self)
        layout.setObjectName(name + ' Layout Service')

        controls_layout = QHBoxLayout()
        spacer = QSpacerItem(40, 20, QSizePolicy.Preferred,
                             QSizePolicy.Preferred)
        controls_layout.addItem(spacer)
        self.service_caller = QPushButton(self)
        style = 'QPushButton:hover{border: 4px solid transparent;} QPushButton{margin: 3px; border-radius: 4px; image: url(' + \
            self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
        self.service_caller.setStyleSheet(style)
        self.service_caller.setToolTip(
            'Service call "' + self.args + '" with timeout  ' + str(self.timeout) + 's inactive')
        self.service_caller.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)
        controls_layout.addWidget(self.service_caller)
        spacer2 = QSpacerItem(
            40, 20, QSizePolicy.Preferred, QSizePolicy.Preferred)
        controls_layout.addItem(spacer2)
        layout.addLayout(controls_layout)

        info_layout = QVBoxLayout()
        if 'default_' in self.icon:
            service_nameL = QLabel('Service: ' + self.args, self)
            service_nameL.setWordWrap(True)
            info_layout.addWidget(service_nameL)
        self.reply_statL = QLabel('Reply status:', self)
        self.reply_statL.setWordWrap(True)
        info_layout.addWidget(self.reply_statL)
        self.reply_msgL = QLabel('Reply message:', self)
        self.reply_msgL.setWordWrap(True)
        info_layout.addWidget(self.reply_msgL)
        line = QFrame(self)
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        info_layout.addWidget(line)
        layout.addLayout(info_layout)

        self.thread_pool = QThreadPool(self)
        self.service = SR2ServiceRunnable(self.args, self.timeout)
        self.service.setAutoDelete(False)
        self.service.signals.srv_running.connect(self.block)
        self.service.signals.srv_status.connect(self.reply)
        self.service_caller.clicked.connect(self.call)

        self.disabled = False
        self.statusOkay = True

        self.setLayout(layout)
        self.resize(layout.sizeHint())

##########################################################################
# SR2ToolbarButtonWi
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
            grid.setMargin(20)
            grid.setContentsMargins(5, 5, 5, 5)

            self.buttons = []
            idx = 0
            for yaml_button_entry in yaml_button_list:
                display_name = None
                if 'name' in yaml_button_entry:
                    display_name = yaml_button_entry['name']
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

    def __init__(self, name, yaml_entry_data, context, surpress_overlays=False, minimal=True):
        super(SR2ButtonWithView, self).__init__()

        self.icon = ''
        if 'icon' in yaml_entry_data:
            # Set icon: IconType.checkImagePath(yaml_entry_data['icon'],
            # view=True)
            self.icon = IconType.checkImagePath(
                yaml_entry_data['icon'], icon_type=IconType.type_view)
        else:
            self.icon = IconType.checkImagePath(icon_type=IconType.type_view)

        style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
            self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
        self.setStyleSheet(style)

        try:
            # Buttons contains a list of YAML buttons inside the menu_entry of
            # the view node. This list is used for populating the view with
            # components
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
        self.clicked.connect(self.toggleView)
        self.context = context
        self.setToolTip(self.name)
        self.setFixedSize(QSize(36, 36))
#        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Maximum)


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

    def toggleView(self):
        '''
        Toggles the visibility of the view (if such exists) that is connected to the menu entry
        '''
        # Sadly the way the views in the dashboard work doesn't allow
        # for a view's components to emit feedback to the menu entry
        # since those are destroyed every  time the view is hidden thus
        # only the menu entry remains...OR MAYBE NOT XD

        if self.init_block_enabled:
            rospy.logerr(
                'SR2: Init ext.process is not running. Unable to control ext.process connected to this button')
            return

        if not self.yaml_view_buttons:
            return

        with QMutexLocker(self.show_mutex):
            style = ''
            try:
                if self.toggled:
                    # If menu entry is already displaying a view, remove it
                    self.toggled = False
                    self.context.remove_widget(self.view_widget)
                    self.close()
#                    self.setIcon(self._icons[IconType.inactive])
                    style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                        self.icon + ') 0 0 0 0 stretch stretch; background: none;}'
                    rospy.logdebug('SR2: Closed SR2View "%s"', self.name)
                else:
                    # If menu entry doesn't display a view, create it and
                    # display it
                    self.toggled = True
#                    self.setIcon(self._icons[IconType.running])
                    style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                        self.icon + \
                            ') 0 0 0 0 stretch stretch; background: rgb(89, 205, 139);}'
                    rospy.logdebug('SR2: Added SR2View "%s"', self.name)
                    self.view_widget = SR2ButtonWithView.SR2View(
                        self.name, self.yaml_view_buttons)

                    self.context.add_widget(self.view_widget)
            except Exception as e:
                if not self.view_widget:
                    #                    self.setIcon(self._icons[IconType.error])
                    style = 'QToolButton:hover{border: 2px solid black;} QToolButton{margin: 3px; border-radius: 4px; image: url(' + \
                        self.icon + \
                            ') 0 0 0 0 stretch stretch; background: rgb(215, 56, 56);}'
                    rospy.logerr(
                        'SR2: Error during showing SR2View : %s', e.message)
                self.toggled = False
            finally:
                self.setStyleSheet(style)

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
