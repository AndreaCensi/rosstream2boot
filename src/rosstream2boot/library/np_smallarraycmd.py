import warnings

from contracts import contract

import numpy as np
from rosstream2boot import ROSCommandsAdapter
from streamels import StreamSpec, make_streamels_1D_float


__all__ = ['NPSmallArrayAsCmd']

class NPSmallArrayAsCmd(ROSCommandsAdapter):

    @contract(topic='str')
    def __init__(self, topic, n):
        self.topic = topic
        self.n = n

    def get_relevant_topics(self):
        dtype = np.dtype(('float', self.n))
        return [(self.topic, dtype)]

    def get_published_topics(self):
        return self.get_relevant_topics()

    def get_stream_spec(self):
        streamels = make_streamels_1D_float(nstreamels=self.n, lower=(-1), upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)

    @contract(commands='array', returns='se3')
    def debug_get_vel_from_commands(self, commands):
        raise NotImplementedError(type(self))
#         vx = commands[0] * self.max_lin_vel
#         vy = commands[1] * self.max_lin_vel
#         w = commands[2] * self.max_ang_vel
#         se2 = se2_from_linear_angular([vx, vy], w)
#         return se3_from_se2(se2)

    def commands_from_messages(self, messages):
        """
            Converts the ROS topics listed here to a numpy array.
            Returns commands_source (name of agent), array.
        """
        msg = messages[self.topic]

        commands = np.array(msg)
        commands = np.clip(commands, -1, +1)


        commands_source = 'sma'  # XXX
        warnings.warn('Must implement proper commands_source.')

        return commands_source, commands

    @contract(returns='dict(str:*)', commands='array')
    def messages_from_commands(self, commands):
        raise NotImplemented
