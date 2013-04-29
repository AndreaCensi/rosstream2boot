from bootstrapping_olympics.interfaces.stream_spec import StreamSpec
from bootstrapping_olympics.interfaces.streamels import make_streamels_1D_float
from rosstream2boot.interfaces import ROSCommandsAdapter
import numpy as np

from contracts import contract

__all__ = ['TracksAdapter']


class TracksAdapter(ROSCommandsAdapter):
    """ Used for landroid's track input """
    
    @contract(topic='str')
    def __init__(self, topic, max_value):
        self.topic = topic
        self.max_value = max_value
        self.use_diff = False
        self._data_class = None
   
    def _get_data_class(self):
        if self._data_class is None:
            import roslib
            roslib.load_manifest('landroid_murraylab')  # @IgnorePep8
            from landroid_murraylab.msg import ldr_tracks  # @UnresolvedImport
            self._data_class = ldr_tracks
        return self._data_class
            
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
        
    
    def __str__(self):
        return ('TracksAdapter(%s,%.3f)' % 
                (self.topic, self.max_value))
