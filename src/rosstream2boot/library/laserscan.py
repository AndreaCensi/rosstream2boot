from contracts import contract

from bootstrapping_olympics import StreamSpec, make_streamels_1D_float
import numpy as np
from rosstream2boot import ROSObservationsAdapter


__all__ = ['LaserScanAdapter']


class LaserScanAdapter(ROSObservationsAdapter):
    
    @contract(topic='str', min_range='float,finite,x', max_range='float,finite,>x')
    def __init__(self, topic, min_range, max_range, index_from, index_to):
        """ index_to is included in the range """
        self.topic = topic
        self.min_range = min_range
        self.max_range = max_range
        self.index_from = index_from
        self.index_to = index_to
        self.n = index_to - index_from + 1
    
    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        from sensor_msgs.msg import LaserScan
        return [(self.topic, LaserScan)]
    
    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        streamels = make_streamels_1D_float(self.n, lower=0, upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ Converts the topics listed here to a class of XX """
        msg = messages[self.topic]
        readings = np.array(msg.ranges)
        y = (readings - self.min_range) / (self.max_range - self.min_range)
        y = np.clip(y, 0, 1)
        y = y[self.index_from:(self.index_to + 1)]
        return y

    def __repr__(self):
        return 'LaserScanAdapter(%s,%s:%s)' % (self.topic, self.index_from, self.index_to)


#
# class LaserScanAdapterFlexible(ROSObservationsAdapter):
#     """ This does not need to know the number of readings before. """
#
#     @contract(topic='str', min_range='float,finite,x', max_range='float,finite,>x')
#     def __init__(self, topic, min_range, max_range):
#         """ index_to is included in the range """
#         self.topic = topic
#         self.min_range = min_range
#         self.max_range = max_range
#
#     @contract(returns='list(tuple(str,*))')
#     def get_relevant_topics(self):
#         from sensor_msgs.msg import LaserScan
#         return [(self.topic, LaserScan)]
#
#     @contract(returns=StreamSpec)
#     def get_stream_spec(self):
#         streamels = make_streamels_1D_float(self.n, lower=0, upper=1)
#         return StreamSpec(id_stream=None, streamels=streamels, extra=None)
#
#     @contract(messages='dict(str:*)')
#     def observations_from_messages(self, messages):
#         """ Converts the topics listed here to a class of XX """
#         msg = messages[self.topic]
#         readings = np.array(msg.ranges)
#         y = (readings - self.min_range) / (self.max_range - self.min_range)
#         y = np.clip(y, 0, 1)
#         y = y[self.index_from:(self.index_to + 1)]
#         return y

    def __repr__(self):
        return 'LaserScanAdapter(%s,%s:%s)' % (self.topic, self.index_from, self.index_to)
