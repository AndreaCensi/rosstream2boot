from bootstrapping_olympics.interfaces.stream_spec import StreamSpec
from contracts import contract
from rosstream2boot.config.rbconfig import get_rs2b_config
from rosstream2boot.interfaces import ROSObservationsAdapter
import numpy as np


class SensorJoin(ROSObservationsAdapter):
    """ Joins two sensors in a unique observation vector. """
    
    @contract(obs_adapters='list[>=1]')
    def __init__(self, obs_adapters):
        self.obs_adapters = obs_adapters
        self.specs = [x.get_stream_spec() for x in obs_adapters]
        self.streamels = [spec.get_streamels() for spec in self.specs]
        
    @contract(returns='list(str)')    
    def get_relevant_topics(self): 
        topics = set()
        for x in self.obs_adapters:
            topics.update(x.get_relevant_topics())
        return list(topics)
    
    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        streamels = join_arrays(self.streamels)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ Converts the topics listed here to a class of XX """
        obs = [a.observations_from_messages(messages) for a in self.obs_adapters]
        observations = join_arrays(obs)
        return observations

    def __str__(self):
        return 'SensorJoin(%s)' % (self.obs_adapters)

    # @contract(obs_adapters='list(str)')
    @staticmethod
    def from_yaml(obs_adapters):
        config = get_rs2b_config()
        adapters = [config.obs_adapters.instance(x) for x in obs_adapters]
        return SensorJoin(adapters)
    

@contract(x='list[>=1](shape(x))')
def join_arrays(x):
    if x[0].ndim == 1:
        return np.hstack(x)
    elif x[0].ndim == 2:
        raise NotImplemented()
    else:
        raise NotImplemented()
        
                
