from bootstrapping_olympics.configuration import get_boot_config
from bootstrapping_olympics.interfaces import RepresentationNuisance, StreamSpec
from contracts import contract
from rosstream2boot.interfaces import ROSObservationsAdapter


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

    # @contract(id_adapter='str', id_nuisance='str')
    # XXX: bug with contracts
    @staticmethod
    def from_yaml(id_adapter, id_nuisance):
        from rosstream2boot.config import get_rs2b_config
        adapter = get_rs2b_config().obs_adapters.instance(id_adapter)
        nuisance = get_boot_config().nuisances.instance(id_nuisance)  
        return ROSObservationsAdapterNuisance(adapter, nuisance)

