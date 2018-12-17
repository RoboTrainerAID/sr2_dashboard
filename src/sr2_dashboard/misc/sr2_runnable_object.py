# Author: Aleksandar Vladimirov Atanasov
# Description: Infrastructure for executing ROS service calls

import rospy
from std_srvs.srv import *
from PyQt5.QtCore import QRunnable, pyqtSignal, QObject
from subprocess import call #for workarround for broken service types
import dynamic_reconfigure.client

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

    def __init__(self, service_name, timeout=0, srv_type = Trigger, params = None):
      
        '''
        Initializes the ServiceRunnable
        :param service_name: name of the ROS service that we want to call
        :param timout: maximum amount of time (in seconds) required for connecting to the given service
        '''
        super(SR2ServiceRunnable, self).__init__()
        self.signals = ServiceCallSignals()
        self.service = service_name
        self.timeout = timeout
        self.srv_type = srv_type
        self.params = params

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

        #if there is something wrong with the "service type" provided: use the console call, which doesn't need a service type
        if isinstance(self.srv_type, str) or  isinstance(self.srv_type, basestring) or self.srv_type is None:
            rospy.logwarn("SR2: Service type '" + self.srv_type + "' not found! Trying to call service via console.")
            call(["rosservice", "call", self.service, str(self.params)])
            return
          
        try:
            # See if service is avialable for a given timeout (default 0: wait
            # until available)
            rospy.wait_for_service(self.service, self.timeout)
            # If service is found, call the server and receive a response
            trigger_call = rospy.ServiceProxy(self.service, self.srv_type)
            srv_call = trigger_call()
            if hasattr(srv_call, 'success'):
                response_status = SR2ServiceRunnable.CallStatus.SUCCESS_TRUE if srv_call.success else SR2ServiceRunnable.CallStatus.SUCCESS_FALSE
            else:
                response_status = SR2ServiceRunnable.CallStatus.SUCCESS_TRUE #we must assume the call was successful
            if hasattr(srv_call, 'message'):
                response_msg = srv_call.message
            else:
                response_msg = str(srv_call)
        except rospy.ROSException, e:
            response_status = SR2ServiceRunnable.CallStatus.FAILED
            response_msg = e.message

        self.signals.srv_running.emit(False)
        self.signals.srv_status.emit(response_status, response_msg)
        
class SR2DynamicReconfigureServiceRunnable(SR2ServiceRunnable):

    def __init__(self, service_name, params, timeout=0):
        '''
        Initializes the ServiceRunnable
        :param service_name: name of the ROS service that we want to call
        :param timout: maximum amount of time (in seconds) required for connecting to the given service
        '''
        super(SR2DynamicReconfigureServiceRunnable, self).__init__(service_name, timeout)
        self.params = params


    def run(self):
      
        response_status = 0
        response_msg = ''

        self.signals.srv_running.emit(True)

        try: 
            #rospy.wait_for_service(self.service)
            client = dynamic_reconfigure.client.Client(self.service, self.timeout)
            client.update_configuration(self.params)
        except rospy.ROSException, e:
            response_status = SR2ServiceRunnable.CallStatus.FAILED
            response_msg = e.message
          

        self.signals.srv_running.emit(False)
        self.signals.srv_status.emit(response_status, response_msg)
