import warnings

from bootstrapping_olympics import StreamSpec, make_streamels_1D_float
from contracts import contract
from geometry import se2_from_linear_angular, se3_from_se2
import numpy as np

from .twist import TwistAdapter


__all__ = ['TwistUnicycle']


class TwistUnicycle(TwistAdapter):
    """ 
        This adapter only allows x and theta rotations 
    
        cmd0: angular velocity
        cmd1: linear velocity
    """
    
    
    def get_stream_spec(self):        
        streamels = make_streamels_1D_float(nstreamels=2, lower=(-1), upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)
    
    def commands_from_messages(self, messages):
        """
            Converts the ROS topics listed here to a numpy array.
            Returns commands_source (name of agent), array.
        """
        msg = messages[self.topic]
        
        if msg.linear.y != 0:
            return None

        u = np.zeros(2, 'float32')        
        u[1] = msg.linear.x / self.max_lin_vel
        u[0] = msg.angular.z / self.max_ang_vel
        commands = np.clip(u, -1, 1)
    
        commands_source = 'twistuni'  # XXX
        warnings.warn('Must implement proper commands_source.')
        
        return commands_source, commands
    
    @contract(commands='array', returns='se3')
    def debug_get_vel_from_commands(self, commands):
        vx = commands[1] * self.max_lin_vel
        vy = 0
        w = commands[0] * self.max_ang_vel
        se2 = se2_from_linear_angular([vx, vy], w)
        return se3_from_se2(se2)
    
    @contract(returns='dict(str:*)', commands='array')
    def messages_from_commands(self, commands):
        from geometry_msgs.msg import Twist
        msg = Twist()
    
        msg.linear.x = commands[1] * self.max_lin_vel
        msg.linear.y = 0 
        msg.linear.z = 0
    
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = commands[0] * self.max_ang_vel
    
        return {self.topic_out: msg}    

    def __str__(self):
        return ('TwistUnicycle(%s,%.3fm/s,%.3frad/s)' % 
                (self.topic, self.max_lin_vel, self.max_ang_vel))
