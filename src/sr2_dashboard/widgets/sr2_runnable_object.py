# -*- coding: utf-8 -*-
"""
Created on Tue Feb 23 15:40:13 2016

@author: atanasov2
"""


import rospy
from std_srvs.srv import Trigger
from PyQt4.QtCore import QRunnable, pyqtSignal, QObject

class ServiceCallSignals(QObject):
  srv_status = pyqtSignal(int, str)
  srv_running = pyqtSignal(bool)

class ServiceRunnable(QRunnable):

  class CallStatus():
    SUCCESS_TRUE = 0
    SUCCESS_FALSE = 1
    FAILED = 2

  def __init__(self, service_name, timeout=10):
    super(ServiceRunnable, self).__init__()
    self.signals = ServiceCallSignals()
    self.service = service_name
    self.timeout = timeout

  def run(self):
    response_status = 0
    response_msg = ''

    self.signals.srv_running.emit(True)
    rospy.wait_for_service(self.service, self.timeout)
    try:
      trigger_call = rospy.ServiceProxy(self.service, Trigger)
      call = trigger_call()
      response_status = ServiceRunnable.CallStatus.SUCCESS_TRUE if call.success else ServiceRunnable.CallStatus.SUCCESS_FALSE
      response_msg = call.message
    except rospy.ServiceException, e:
      response_status = ServiceRunnable.CallStatus.FAILED
      response_msg = e

    self.signals.srv_running.emit(False)
    self.signals.srv_status.emit(response_status, response_msg)