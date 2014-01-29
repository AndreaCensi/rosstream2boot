from contracts import contract

from bootstrapping_olympics import RepresentationNuisance, StreamSpec
from bootstrapping_olympics.library.nuisances import Chain
from rosstream2boot import ROSObservationsAdapter, get_conftools_obs_adapters


__all__ = ['ROSObservationsAdapterNuisance']


class ROSObservationsAdapterNuisance(ROSObservationsAdapter):
    
    @contract(adapter=ROSObservationsAdapter, nuisance=RepresentationNuisance)
    def __init__(self, adapter, nuisance):
        self.adapter = adapter
        self.nuisance = nuisance
        
        self.adapter_spec = self.adapter.get_stream_spec()
        self.spec = self.nuisance.transform_spec(self.adapter_spec)

    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        """ Returns the list of topics that are relevant for us. """
        return self.adapter.get_relevant_topics()

    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        return self.spec
        
    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        obs = self.adapter.observations_from_messages(messages)
        return self.nuisance.transform_value(obs)

    @staticmethod
    @contract(adapter="str|code_spec",
              nuisances="list(str|code_spec)")
    def from_yaml(adapter, nuisances):
        adapter = get_conftools_obs_adapters().instance_smarter(adapter)[1]
        nuisance = Chain.instance_specs(nuisances)
        return ROSObservationsAdapterNuisance(adapter, nuisance)

