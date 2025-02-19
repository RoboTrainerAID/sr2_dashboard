# Author: Aleksandar Vladimirov Atanasov
# Description: Gives visual feedback to the user if the emergency stop is active or not
#	       The widgets inherits the QLabel since there no interaction is required

import os

# ROS
import rospkg
import rospy
from python_qt_binding.QtCore import Signal, QSize
from python_qt_binding.QtGui import QIcon, QLabel, QStatusBar

# Rqt Dashboard Widget
# from .icon_tool_button import IconToolButton # Base class for all
# dashboard widgets (totally diggin the name... LOL)
from rqt_robot_dashboard.util import IconHelper

# COB messages
#from cob_msgs.msg import EmergencyStopState


class SR2Statusbar(QStatusBar):

    state_changed = Signal(str, int)
    sr2_state_normal = 0
    sr2_state_warning = 1
    sr2_state_error = 2

    def __init__(self, name='SR2 Statusbar', icons=None, icon_paths=None, minimal=True):
        super(SR2Statusbar, self).__init__('SR2 Statusbar')
        self.statusIcon = QLabel()
        self.addWidget(statusIcon, 1)

        if icons == None:
            icons = []
            icons.append(['status/status_normal.svg'])
            icons.append(['status/status_warning.svg'])
            icons.append(['status/status_error.svg'])

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
        self._state = ''
        self._status = 0
        self.state_changed.connect(self._update_state)
        self.update_state(self._state, status)

    def update_state(self, state, status):
        self._state = state
        self.state_changed.emit(self._state, status)

    def _update_state(self, state, status):
        if self.status >= 0 and self.status <= 2:
            self.statusIcon.setPixmap(
                self._icons[status].pixmap(QSize(10, 10)))
            self.showMessage(state)
        else
            pass

    @property
    def state(self):
        return self._state
