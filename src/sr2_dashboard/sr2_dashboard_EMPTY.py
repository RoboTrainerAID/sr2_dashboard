#!/usr/bin/env python

import roslib
import rospy

# COB messages
# ...

# RQT Robot Dashboard
from rqt_robot_dashboard.dashboard import Dashboard
#from rqt_robot_dashboard.widgets import ...

# PyQt
#from python_qt_binding.QtCore import ...

# Widgets
# ...

class SR2Dashboard(Dashboard):

  def setup(self, context):
    self.name = 'SR2Dashboard'
    
    # declare subscribers and publishers

  def get_widgets(self):
    return []

  def shutdown_dashboard(self):
    # unregister all subscribers
    pass

  def save_settings(self, plugin_settings, instance_settings):
    pass

  def restore_settings(self, plugin_settings, instance_settings):
    pass
