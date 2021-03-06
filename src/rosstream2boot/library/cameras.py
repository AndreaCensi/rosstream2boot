from bootstrapping_olympics import (StreamSpec, make_streamels_2D_float,
    make_streamels_rgb_float)
from contracts import contract
from procgraph_images import rgb2gray
from procgraph_pil import resize
from rosstream2boot import ROSObservationsAdapter
import numpy as np


__all__ = ['CameraAdapterGray', 'CameraAdapter']

class CameraAdapterGray(ROSObservationsAdapter):
    
    @contract(topic='str', shape='seq[2](int)')
    def __init__(self, topic, shape):
        self.topic = topic
        self.shape = shape
    
    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        from sensor_msgs.msg import CompressedImage
        return [(self.topic, CompressedImage)]

    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        streamels = make_streamels_2D_float(shape=self.shape, lower=0, upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ Converts the topics listed here to a class of XX """
        from procgraph_ros.conversions import ros2rgb  # XXX

        msg = messages[self.topic]
        rgb = ros2rgb(msg)
        rgb2 = resize(rgb, width=self.shape[1], height=self.shape[0]) 
        
        gray = rgb2gray(rgb2)
        gray01 = np.clip(gray / 255.0, 0, 1)
        return gray01

    def __str__(self):
        return 'CameraAdapterGray(%s,%s)' % (self.topic, self.shape)


class CameraAdapter(ROSObservationsAdapter):
    
    @contract(topic='str', shape='seq[2](int)')
    def __init__(self, topic, shape):
        self.topic = topic
        self.shape = shape
    
    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        from sensor_msgs.msg import CompressedImage
        return [(self.topic, CompressedImage)]

    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        streamels = make_streamels_rgb_float(shape=self.shape)
        return StreamSpec(id_stream='camera', streamels=streamels,
                          extra=None)

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ Converts the topics listed here to a class of XX """
        from procgraph_ros.conversions import ros2rgb
        msg = messages[self.topic]
        rgb = ros2rgb(msg)       
        rgb2 = resize(rgb, width=self.shape[1], height=self.shape[0])
        rgb2 = np.clip(rgb2 / 255.0, 0, 1) 
        return rgb2

    def __str__(self):
        return 'CameraAdapter(%s,%s)' % (self.topic, self.shape)
