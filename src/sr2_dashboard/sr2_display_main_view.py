# Author: Aleksandar Vladimirov Atanasov
# Description: Displays the main view with various buttons for controlling nodes and services on the SR2 platform

import os

# ROS
import rospkg
import rospy
from python_qt_binding.QtCore import Signal, QMutex, QMutexLocker, QTimer
#from python_qt_binding.QtGui import QWidget

# Rqt Dashboard Widget
from icon_tool_button import IconToolButton # Base class for all dashboard widgets (totally diggin the name... LOL)
from rqt_robot_dashboard.util import IconHelper
from yaml_gui import SR2LayoutCreator

# COB messages
#from cob_msgs.msg import EmergencyStopState

class SR2MainViewWidget(IconToolButton):        

  state_changed = Signal(bool)

  def __init__(self, name='SR2 Main View', icons=None, icon_paths=None, minimal=True):
    if icons == None:
      icons = []
      icons.append(['emergency_stop/emergency_stop_off.svg'])
      icons.append(['emergency_stop/emergency_stop_on.svg'])
    
    #icon_paths = (icon_paths if icon_paths else []) + [['rqt_robot_dashboard', 'images']]
    icon_paths = (icon_paths if icon_paths else []) + [['sr2_dashboard', 'resources/images']]
    
    super(SR2MainViewWidget, self).__init__('SR2 Main View', icons, icon_paths=icon_paths)
    
    paths = []
    rp = rospkg.RosPack()
    for path in icon_paths:
        paths.append(os.path.join(rp.get_path(path[0]), path[1]))
    self._icon_helper = IconHelper(paths, name)
    converted_icons = self._icon_helper.set_icon_lists(icons)
    
    self._some_widget = None
    self._icons = converted_icons[0]
    self._name = name
    self._stall_timer = QTimer()
    self._stall_timer.timeout.connect(self._stalled)
    #    self._state = False
    self.mainview_visible = False
    self._plugin_settings = None
    self._instance_settings = None
    self.setCheckable(True)
    self.setIcon(self._icons[1])
    self.state_changed.connect(self._update_state)
    #    self.update_state(self._state)
    self.clicked.connect(self._show_main_view)
    
    self._close_mutex = QMutex()
    self._show_mutex = QMutex()
    
  def _stalled(self):
      self._stall_timer.stop()
    
  def _show_main_view(self, v):
    with QMutexLocker(self._show_mutex):
        try:
            # use v here to control since our button is togglable
            if self.mainview_visible:
                self.context.remove_widget(self._some_widget)
                self._close_main_view()
                self.mainview_visible = False
            else:
                self._some_widget = SR2LayoutCreator('config1.yaml', 'Launches')
                if self._plugin_settings:
                    self._some_widget.restore_settings(self._plugin_settings, self._instance_settings)
                self.mainview_visible = True
        except Exception:
            if self.mainview_visible == False:
                raise
            #TODO: when closeEvents is available fix this hack
            # (It ensures the button will toggle correctly)
            self.mainview_visible = False
            self._show_main_view(True)
    if v:
      self.setToolTip("Show main view")
      rospy.loginfo('HELLO!')
      self.setIcon(self._icons[0])
    else:
      self.setToolTip("Hide main view")
      rospy.loginfo('BYE!')
      self.setIcon(self._icons[1])
      
  def _close_main_view(self):
    if self._monitor_shown:
      with QMutexLocker(self._close_mutex):
        if self._plugin_settings:
          self._some_widget.save_settings(self._plugin_settings, self._instance_settings)
          self._some_widget.shutdown()
          self._some_widget.close()
          #self._graveyard.append(self._monitor)
          self._some_widget = None
          
  def shutdown_widget(self):
      self._stall_timer.stop()
      if self._monitor:
          self._monitor.shutdown()
          self._diagnostics_toplevel_state_sub.unregister()

  def save_settings(self, plugin_settings, instance_settings):
        if self._monitor_shown:
            self._monitor.save_settings(self._plugin_settings,
                                        self._instance_settings)

  def restore_settings(self, plugin_settings, instance_settings):
        self._plugin_settings = plugin_settings
        self._instance_settings = instance_settings

#  def update_state(self, state):
#    self._state = state
#    self.state_changed.emit(self._state)

#  def _update_state(self, state):
#    if self.state:
#      self.setToolTip("Show main view")
#      self.setIcon(self._icons[1])
#    else:
#      self.setToolTip("Hide main view")
#      self.setIcon(self._icons[0])

  @property
  def state(self):
    return self._state
