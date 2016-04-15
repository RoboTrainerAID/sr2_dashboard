# Author: Aleksandar Vladimirov Atanasov
# Description: Infrastructure for executing ROS service calls

import rospy
from std_srvs.srv import Trigger
from PyQt4.QtCore import QRunnable, pyqtSignal, QObject


class ServiceCallSignals(QObject):
    '''
    Contains state-related signals that are used by ServiceRunnable
    '''
    srv_status = pyqtSignal(
        int, str)  # (one of the values stored inside CallStatus, reply message)
    # if True caller UI component will be blocked else unblocked
    srv_running = pyqtSignal(bool)


class SR2ServiceRunnable(QRunnable):
    '''
    Intiate a ROS service call and reports back to the UI component that has triggered the procedure
    '''

    class CallStatus():
        '''
        A service call can have three states upon completion:

          - **successful with status True** - button's icon changes to INACTIVE
          - **successful with status False** - button's icon changes to INACTIVE
          - **failed** - button's icon changes to ERROR
        '''
        SUCCESS_TRUE = 0
        SUCCESS_FALSE = 1
        FAILED = 2

    def __init__(self, service_name, timeout=0):
        '''
        Initializes the ServiceRunnable
        :param service_name: name of the ROS service that we want to call
        :param timout: maximum amount of time (in seconds) required for connecting to the given service
        '''
        super(SR2ServiceRunnable, self).__init__()
        self.signals = ServiceCallSignals()
        self.service = service_name
        self.timeout = timeout

    def run(self):
        '''
        When executed (using QThreadPool.start(<SR2ServiceRunnable_instance>)) following
      happens:

        - Button that has triggerd it gets blocked
        - Button's icon changed to RUNNING
        - ROS service gets called (of type Trigger)
        - After given timeout or upon receiving a reply in the given time status is
          reported back to the button
        - Button that has triggered it gets unblocked
        - Button's icon changes to INACTIVE (upon successful service call) or ERROR (if timeout or other exception has been raised)

      All is ran inside a separate thread retrieved from the thread pool
        '''
        response_status = 0
        response_msg = ''

        self.signals.srv_running.emit(True)
        try:
            # See if service is avialable for a given timeout (default 0: wait until available)
            rospy.wait_for_service(self.service, self.timeout)
            # If service is found, call the server and receive a response
            trigger_call = rospy.ServiceProxy(self.service, Trigger)
            call = trigger_call()
            response_status = SR2ServiceRunnable.CallStatus.SUCCESS_TRUE if call.success else SR2ServiceRunnable.CallStatus.SUCCESS_FALSE
            response_msg = call.message
        except rospy.ROSException, e:
            response_status = SR2ServiceRunnable.CallStatus.FAILED
            response_msg = e.message

        self.signals.srv_running.emit(False)
        self.signals.srv_status.emit(response_status, response_msg)
