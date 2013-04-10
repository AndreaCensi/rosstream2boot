from contracts import contract
from abc import abstractmethod, ABCMeta
from bootstrapping_olympics.interfaces.stream_spec import StreamSpec


class ROSCommandsAdapter():
    __metaclass__ = ABCMeta

    @contract(returns='list(str)')    
    def get_relevant_topics(self):
        """ Returns the list of topics that are relevant for us. """
        pass
        
    @abstractmethod
    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        ''' Returns the sensorimotor spec for this robot
            (a BootSpec object). '''
    
    @abstractmethod
    @contract(messages='dict(str:*)', returns='tuple(str,array)')
    def commands_from_messages(self, messages):
        """ 
            Converts the ROS topics listed here to a numpy array.
            Returns commands_source (name of agent), array.
        """

    @abstractmethod
    @contract(returns='dict(str:*)')
    def messages_from_commands(self):
        """ Must return a topic or a message. """
            
