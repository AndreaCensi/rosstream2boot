from contracts import contract

from bootstrapping_olympics.interfaces.streamels.stream_spec import StreamSpec
from bootstrapping_olympics.interfaces.streamels.streamels_make import (
    make_streamels_rgb_float)
import numpy as np
from rosstream2boot import ROSObservationsAdapter


__all__ = ['NPImage']


class NPImage(ROSObservationsAdapter):

    @contract(topic='str', shape='seq[2](int)')
    def __init__(self, topic, shape):
        self.topic = topic
        self.shape = shape

    @contract(returns='list(tuple(str,*))')
    def get_relevant_topics(self):
        shape = (self.shape[0], self.shape[1], 3)
        dtype = np.dtype(('uint8', shape))
        return [(self.topic, dtype)]

    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        streamels = make_streamels_rgb_float(shape=self.shape)
        return StreamSpec(id_stream='camera',
                          streamels=streamels,
                          extra=None)

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        rgb = messages[self.topic]
        assert isinstance(rgb, np.ndarray)
        rgb2 = np.clip(rgb * (1.0 / 255.0), 0, 1)
        return rgb2

    def __str__(self):
        return 'NPImage(%s,%s)' % (self.topic, self.shape)
