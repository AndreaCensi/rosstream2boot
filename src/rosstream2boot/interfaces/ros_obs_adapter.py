from contracts import contract
from abc import abstractmethod, ABCMeta
from bootstrapping_olympics.interfaces.stream_spec import StreamSpec


class ROSObservationsAdapter:
    __metaclass__ = ABCMeta

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
        """ Converts the topics listed here 
            to an array respecting the spec above. """

