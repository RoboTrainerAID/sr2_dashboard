# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
#
# Modification:
# This file has been modified by Aleksandar Vladimirov Atanasov in order to
# adapt it to the SR2 platform.
# THE MODIFICATIONS ARE PROPERTY OF KARLSRUHE INSTITUTE OF TECHNOLOGY,
# KARLSRUHE, GERMANY


import os
import rospkg
import rospy
from python_qt_binding.QtCore import Signal, QSize
from python_qt_binding.QtGui import QIcon, QLabel
from rqt_robot_dashboard.util import IconHelper


class SR2BatteryDashWidget(QLabel):
    """
    A Widget which displays incremental battery state, including a status tip.
    To use this widget simply call :func:`update_perc` and :func:`update_time`
    to change the displayed charge percentage and time remaining, respectively.
    :param name: The name of this widget
    :type name: str
    """
    state_changed = Signal(int)

    def __init__(self, name='SR2 Battery', icons=None, charge_icons=None, icon_paths=None, suppress_overlays=False):
        super(SR2BatteryDashWidget, self).__init__()
        if icons == None:
            icons = []
            charge_icons = []
            for x in range(6):
                icons.append(['battery/battery-%s.svg' % (x * 20)])
                charge_icons.append(
                    ['battery/battery-charge-%s.svg' % (x * 20)])
        icon_paths = (icon_paths if icon_paths else []) + \
            [['sr2_dashboard', 'resources/images']]
        paths = []
        rp = rospkg.RosPack()
        for path in icon_paths:
            paths.append(os.path.join(rp.get_path(path[0]), path[1]))
        self._icon_helper = IconHelper(paths, name)

        converted_icons = self._icon_helper.set_icon_lists(
            icons, charge_icons, suppress_overlays)
        self._icons = converted_icons[0]
        self._charge_icons = converted_icons[1]
        self._name = name
        self._charging = False
        self.__state = 0
        self.setMargin(5)
        self.state_changed.connect(self._update_state)
        self.update_perc(0)
        self.update_time(0)

    def _update_state(self, state):
        #        if self._charging:
        #            self.setPixmap(self._charge_icons[state].pixmap(QSize(60, 100)))
        #        else:
        #            self.setPixmap(self._icons[state].pixmap(QSize(60, 100)))

        self.setPixmap(self._icons[state].pixmap(QSize(60, 100)))
        pass

    @property
    def state(self):
        """
        Read-only accessor for the widgets current state.
        """
        return self.__state

    def set_charging(self, value):
        self._charging = value

    def update_perc(self, val):
        """
        Update the displayed battery percentage.
        The default implementation of this method displays in 20% increments
        :param val: The new value to be displayed.
        :type val: int
        """
        self.update_state(round(val / 20.0))

    def update_state(self, state):
        """
        Set the state of this button.
        This will also update the icon for the button based on the ``self._icons`` list
        :raises IndexError: If state is not a proper index to ``self._icons``
        :param state: The state to set.
        :type state: int
        """
        # The /power_state topic should be modified to contain the charging status (true/false)
        # along with the publisher of /power_state that has to determine this status
#        if 0 <= state and state < len(self._icons):
#            self.__state = state
#            self.state_changed.emit(self.__state)
#        else:
#            raise IndexError("%s update_state received invalid state: %s" % (self._name, state))

        self.__state = state
        self.state_changed.emit(self.__state)
        pass

    def update_time(self, value):
        try:
            fval = float(value)
            self.setToolTip("%s: %.2f%% remaining" % (self._name, fval))
        except ValueError:
            self.setToolTip("%s: %s%% remaining" % (self._name, value))
