from abc import abstractmethod

from contracts import contract, ContractsMeta

from bootstrapping_olympics import StreamSpec


__all__ = ['ROSObservationsAdapter']

class ROSObservationsAdapter(object):
    __metaclass__ = ContractsMeta

    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        """ 
            Returns the list of topics that are relevant for us
            This is a list of tuples (topic, data_class).
        """
        
    @abstractmethod
    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        ''' Returns the sensorimotor spec for this robot
            (a BootSpec object). '''

    @abstractmethod
    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ 
            Converts the topics listed here 
            to an array respecting the spec above. 
        
            (return None if no message generated?)
        """

