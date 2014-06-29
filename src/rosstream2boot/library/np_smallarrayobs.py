import warnings

from contracts import contract

import numpy as np
from rosstream2boot import ROSObservationsAdapter
from streamels import StreamSpec, make_streamels_float


__all__ = ['NPSmallArrayAsObs']

class NPSmallArrayAsObs(ROSObservationsAdapter):

    @contract(topic='str', shape='tuple')
    def __init__(self, topic, shape):
        self.topic = topic
        self.shape = shape

    @contract(returns='list(tuple(str,*))')
    def get_relevant_topics(self):
        shape = self.shape
        dtype = np.dtype(('float32', shape))
        return [(self.topic, dtype)]

    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        warnings.warn('need to find bounds')
        M = 1000 * 1000
        streamels = make_streamels_float(self.shape, lower=-M, upper=M)
        return StreamSpec(id_stream='camera',
                          streamels=streamels,
                          extra=None)

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        a = messages[self.topic]
        assert isinstance(a, np.ndarray)
        assert a.shape == self.shape
        return a.astype('float32')

    def __str__(self):
        return 'NPSmallArrayAsObs(%s,%s)' % (self.topic, self.shape)
