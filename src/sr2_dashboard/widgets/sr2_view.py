# Author: Aleksandar Vladimirov Atanasov
# Description: A SR2 view represents a module - one or multiple controllable processes in a common context

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
from python_qt_binding.QtGui import QWidget, QGridLayout
from python_qt_binding.QtCore import pyqtSlot, pyqtSignal
# QtCore modules
#from python_qt_binding.QtCore import ...

# Widgets
#from sr2_view_button import SR2MenuEntryViewWidget as sr2mev
from ..misc.sr2_grid_generator import SR2GridGenerator as sr2gg

class SR2MenuEntryViewWidget(QWidget):
  '''
  Shows/hides a collection of SR2ButtonWidgets whenever a corresponding SR2MenuEntryWidget is clicked
  It is meant to provide a module of SR2ButtonWidgets with similar functionality
  '''

  # viewFeedbackSignal is captured by its menu entry and reports the overall status of all
  # widgets that are part of the view. If a signle error occurs, the menu entry's icon is set
  # to error status until problem is resolved
  viewFeedbackSignal = pyqtSignal(int)

  @pyqtSlot(int, str)
  def getOverallStatus(status, obj_name):
    '''
    :param status: contains the status of a view's component
    :param obj_name: contains the object name of a view's component
    '''
    # TODO Assign obj_name to each view component (module name + counter)
    # TODO Connect each created view component to getOverallStatus() slot
    # TODO Add signal in each view component that emits status+view component's name
    # TODO Add dictionary that contains view component's object name as key and its status as value
    # TODO Every time getOverallStatus() slot is triggered, check all dict entries:
    #      - if all view components have the same status send viewFeedbackSignal with that status
    #      - if a single view component is running and no errors are present in any of the others send viewFeedbackSignal with status RUNNING as value
    #      - if a single view component has failed send viewFeedBackSignal with value FAILED_STOP/FAILED_START (depending on the cause of failure)
    pass

  @staticmethod
  def createWidget(name, yamlButtonList, icons):
    return SR2MenuEntryViewWidget(yamlButtonList, name, icons)

  def __init__(self, yamlButtonList, name, icons):
    '''
    Creates a grid of custom widgets for controlling one or more processes
    :param yamlButtonList: YAML node containing one or multiple buttons
    :param name: name of the module the view represents (IMPORTANT: has to be unique => no other module can have the same name)
    :param icons: used for all custom widgets (tristate: inactive, running and error)
    '''
    super(SR2MenuEntryViewWidget, self).__init__()

    obj_name = name
    self.setObjectName(obj_name)
    self.setWindowTitle(obj_name)



    rospy.loginfo('SR2: Creating view for menu entry "%s"' % name)

    self.grid = QGridLayout()
    self.grid.setMargin(20)
    self.grid.setContentsMargins(5,5,5,5)

    self.setLayout(self.grid)

    if yamlButtonList == None:
      rospy.logwarn('SR2: No data to populate view. View will be empty')
      return

    # Get dimensions of grid based on number of buttons
    (self.rows, self.cols) = sr2gg.get_dim(len(yamlButtonList))
    rospy.loginfo('SR2: View contains %d buttons which will be distributed on a %d by %d (rows by columns) grid layout' % (len(yamlButtonList), self.rows, self.cols))

    self.buttons = []
    for ybutton in range(0, len(yamlButtonList)):
      # Use createButtonWidget(...) (from yaml_gui.py) to generate each button
      # and then append it to the the list with buttons

      #rospy.loginfo('SR2: Parsing button: %s' % yamlButtonList[ybutton])
      #button = SR2ButtonWidget.createWidget(yamlButtonList[ybutton])
      #self.buttons.append(button)

      # TODO create a feeback signal from each button to the view that reports its status
      #
      pass

    for button in self.buttons:
      self.grid.addWidget(button)

  def shutdown(self):
    rospy.loginfo('SR2: Bye!')

  def save_settings(self, plugin_settings, instance_settings):
    pass

  def restore_settings(self, plugin_settings, instance_settings):
    pass