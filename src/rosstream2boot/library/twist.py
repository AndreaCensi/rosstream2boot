from bootstrapping_olympics import StreamSpec, make_streamels_1D_float
from contracts import contract
from rosstream2boot import ROSCommandsAdapter
import numpy as np
import warnings


__all__ = ['TwistAdapter']


class TwistAdapter(ROSCommandsAdapter):
    
    @contract(topic='str', topic_out='str',
              max_lin_vel='float,>0', max_ang_vel='float,>0')
    def __init__(self, topic, topic_out, max_lin_vel, max_ang_vel):
        self.topic = topic
        self.topic_out = topic_out
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        
    def get_relevant_topics(self):
        from geometry_msgs.msg import Twist
        return [(self.topic, Twist)]

    def get_published_topics(self):
        from geometry_msgs.msg import Twist
        return [(self.topic_out, Twist)]
    
    def get_stream_spec(self):        
        streamels = make_streamels_1D_float(nstreamels=3, lower=(-1), upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)
    
    def commands_from_messages(self, messages):
        """
            Converts the ROS topics listed here to a numpy array.
            Returns commands_source (name of agent), array.
        """
        msg = messages[self.topic]
        # assert isinstance(msg, Twist)

        u = np.zeros(3, 'float32')        
        u[0] = msg.linear.x / self.max_lin_vel
        u[1] = msg.linear.y / self.max_lin_vel
        u[2] = msg.angular.z / self.max_ang_vel
        commands = np.clip(u, -1, 1)
    
        commands_source = 'twist'  # XXX
        warnings.warn('Must implement proper commands_source.')
        
        return commands_source, commands
    
    @contract(returns='dict(str:*)', commands='array')
    def messages_from_commands(self, commands):
        from geometry_msgs.msg import Twist
        msg = Twist()
    
        msg.linear.x = commands[0] * self.max_lin_vel
        msg.linear.y = commands[1] * self.max_lin_vel
        msg.linear.z = 0
    
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = commands[2] * self.max_ang_vel
    
        return {self.topic_out: msg}    

    def __str__(self):
        return ('TwistAdapter(%s,%.3fm/s,%.3frad/s)' % 
                (self.topic, self.max_lin_vel, self.max_ang_vel))
