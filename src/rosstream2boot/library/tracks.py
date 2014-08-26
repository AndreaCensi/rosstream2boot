import traceback
import warnings

from contracts import contract
from contracts.utils import indent

from geometry import se2_from_linear_angular, se3_from_se2
import numpy as np
from rosstream2boot import ROSCommandsAdapter, logger
from streamels import StreamSpec, make_streamels_1D_float


__all__ = ['TracksAdapter']


class TracksAdapter(ROSCommandsAdapter):
    """ Used for landroid's track input """
    
    @contract(topic='str')
    def __init__(self, topic, max_value):
        self.topic = topic
        self.max_value = max_value
        self.use_diff = False
        # Do not want to serialize this - ROS sucks in so many ways
        # self._data_class = None
   
    def _get_data_class(self):
        """ Tries to load the data class """
        import roslib  # @UnresolvedImport
        try:
            roslib.load_manifest('landroid_murraylab')  # @IgnorePep8
            from landroid_murraylab.msg import ldr_tracks  # @UnresolvedImport
        except Exception as e:
            msg = 'Could not import "ldr_tracks" datatype: %s' % e
            msg += indent(traceback.format_exc(e), '> ')
            logger.warn(msg)
            ldr_tracks = None
        return ldr_tracks
            
    def get_relevant_topics(self):
        ldr_tracks = self._get_data_class()
        return [(self.topic, ldr_tracks)]

    def get_published_topics(self):
        return self.get_relevant_topics()
    
    def get_stream_spec(self):        
        streamels = make_streamels_1D_float(nstreamels=2, lower=(-1), upper=1)
        return StreamSpec(id_stream='tracks', streamels=streamels,
                          extra=dict(max_value=self.max_value))
    
    def commands_from_messages(self, messages):
        """
            Converts the ROS topics listed here to a numpy array.
            Returns commands_source (name of agent), array.
        """
        if not self.topic in messages:
            return None
        msg = messages[self.topic]
        
        u = np.zeros(2, 'float32')        
        u[0] = msg.left / self.max_value
        u[1] = msg.right / self.max_value
        commands = np.clip(u, -1, 1)
    
        commands_source = 'tracks'
        return commands_source, commands
    
    @contract(returns='dict(str:*)', commands='array')
    def messages_from_commands(self, commands):
        L = commands[0] * self.max_value
        R = commands[1] * self.max_value
        ldr_tracks = self._get_data_class()
        msg = ldr_tracks(L, R)
        return {self.topic: msg}
        
    @contract(commands='array', returns='se3')
    def debug_get_vel_from_commands(self, commands):
        warnings.warn("this is not precise, we don't know the conversion")
        L = commands[0] * self.max_value
        R = commands[1] * self.max_value
        vx = ((L + R) / 2) * 0.01
        vy = 0
        omega = (L - R) * 0.01
        vel = se2_from_linear_angular([vx, vy], omega)
        return se3_from_se2(vel)
    
    def __str__(self):
        return ('TracksAdapter(%s,%.3f)' % 
                (self.topic, self.max_value))
