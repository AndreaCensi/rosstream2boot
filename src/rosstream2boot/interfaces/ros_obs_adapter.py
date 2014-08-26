from abc import abstractmethod

from contracts import contract, ContractsMeta

from bootstrapping_olympics import StreamSpec
from blocks.library.simple.with_queue import WithQueue
from blocks.library.timed.checks import check_timed_named


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

    def get_translator(self):
        class Translator(WithQueue):
            def __init__(self, adapter):
                self.adapter = adapter
            def reset(self):
                WithQueue.reset(self)
            def put_noblock(self, value):
                check_timed_named(value)
                (t, (signal, x)) = value
                messages = {}
                messages[signal] = x
                y = self.adapter.observations_from_messages(messages)
                if y is not None:
                    self.append((t, ('observations', y)))
        return Translator(self)
