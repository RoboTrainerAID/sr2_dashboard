# Author: Aleksandar Vladimirov Atanasov
# Description: Gives visual feedback to the user if the emergency stop is active or not
#	       The widgets inherits the QLabel since there no interaction is required

import os

# ROS
import rospkg
import rospy
from python_qt_binding.QtCore import Signal, QSize
from python_qt_binding.QtGui import QIcon, QLabel

# Rqt Dashboard Widget
# from .icon_tool_button import IconToolButton # Base class for all
# dashboard widgets (totally diggin the name... LOL)
from rqt_robot_dashboard.util import IconHelper

# COB messages
#from cob_msgs.msg import EmergencyStopState


class SR2EmergencyStopWidget(QLabel):

    state_changed = Signal(bool)

    def __init__(self, name='SR2 Emergency Stop', icons=None, icon_paths=None, minimal=True):
        super(SR2EmergencyStopWidget, self).__init__('SR2 Emergency Stop')

        if icons == None:
            icons = []
            icons.append(['emergency_stop/emergency_stop_off.svg'])
            icons.append(['emergency_stop/emergency_stop_on.svg'])

        #icon_paths = (icon_paths if icon_paths else []) + [['rqt_robot_dashboard', 'images']]
        icon_paths = (icon_paths if icon_paths else []) + \
            [['sr2_dashboard', 'resources/images']]
        paths = []
        rp = rospkg.RosPack()
        for path in icon_paths:
            paths.append(os.path.join(rp.get_path(path[0]), path[1]))
        self._icon_helper = IconHelper(paths, name)
        converted_icons = self._icon_helper.set_icon_lists(icons)

        self._icons = converted_icons[0]
        self._name = name
        self.setMargin(5)
        self._state = False
        self.state_changed.connect(self._update_state)
        self.update_state(self._state)
        self.clicked.connect(self._print_hello)

    def update_state(self, state):
        self._state = state
        self.state_changed.emit(self._state)

    def _update_state(self, state):
        if self.state:
            self.setPixmap(self._icons[1].pixmap(QSize(30, 30)))
            self.setToolTip("Emergency stop is ON. Press the emergency stop button,\n"
                            "turn it around (follow the arrows) and turn for a very\n"
                            "short period of the time the key to ON/RESET")
        else:
            self.setPixmap(self._icons[0].pixmap(QSize(30, 30)))
            self.setToolTip(
                "Emergency stop is OFF. Press the emergency stop button")

    def _print_hello():
        rospy.loginfo("HELLO BIATCH!")

    @property
    def state(self):
        return self._state
