from rosstream2boot.interfaces.ros_adapter import ROSObservationsAdapter
from contracts import contract
from bootstrapping_olympics.interfaces.stream_spec import StreamSpec
from bootstrapping_olympics.interfaces.streamels import make_streamels_2D_float
from procgraph_ros.conversions import ros2rgb
from procgraph_images.filters import rgb2gray
import numpy as np


class CameraAdapterGray(ROSObservationsAdapter):
    
    @contract(topic='str', shape='seq[2](int)')
    def __init__(self, topic, shape):
        self.topic = topic
        self.shape = shape
    
    @contract(returns='list(str)')    
    def get_relevant_topics(self):
        return [self.topic]
    
    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        streamels = make_streamels_2D_float(shape=self.shape, lower=0, upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ Converts the topics listed here to a class of XX """
        msg = messages[self.topic]
        rgb = ros2rgb(msg)
        
        gray = rgb2gray(rgb)
        gray01 = np.clip(gray / 255.0, 0, 1)
        return gray01

    def __str__(self):
        return 'CameraAdapterGray(%s,%s)' % (self.topic, self.shape)
