from rosstream2boot.interfaces import ROSCommandsAdapter
from bootstrapping_olympics.interfaces.streamels import make_streamels_1D_float
import numpy as np
from bootstrapping_olympics.interfaces.stream_spec import StreamSpec
import warnings


class TwistAdapter(ROSCommandsAdapter):
    
    def __init__(self, topic, max_lin_vel, max_ang_vel):
        self.topic = topic
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        
    def get_relevant_topics(self):
        return [self.topic]
    
    def get_stream_spec(self):        
        streamels = make_streamels_1D_float(nstreamels=3, lower=(-1), upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)
    
    def commands_from_messages(self, messages):
        """
            Converts the ROS topics listed here to a numpy array.
            Returns commands_source (name of agent), array.
        """

        msg = messages[self.topic]
        u = np.zeros(3, 'float32')        
        u[0] = msg.linear.x
        u[1] = msg.linear.y
        u[2] = msg.angular.z
        commands = np.clip(u, -1, 1)
        commands_source = 'twist'  # XXX
        warnings.warn('Must implement proper commands_source.')
        return commands_source, commands
    
    def messages_from_commands(self, commands):
        msg = geometry_msgs.msg.Twist()  # @UndefinedVariable
    
        msg.linear.x = commands[0]
        msg.linear.y = commands[1]
        msg.linear.z = 0
    
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = commands[2]
    
        return msg

    def command2topic(self):
        msg = geometry_msgs.msg.Twist()  # @UndefinedVariable
        raise NotImplementedError()

    def __str__(self):
        return 'TwistAdapter(%s,%.3fm/s,%.3frad/s)' % (self.topic, self.max_lin_vel, self.max_ang_vel)
