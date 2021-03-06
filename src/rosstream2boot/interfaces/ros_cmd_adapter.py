from abc import abstractmethod
from bootstrapping_olympics import StreamSpec
from contracts import contract, ContractsMeta

__all__ = ['ROSCommandsAdapter']


class ROSCommandsAdapter(object):
    __metaclass__ = ContractsMeta

    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        """ 
            Returns the list of topics that are relevant for us
            This is a list of tuples (topic, data_class).
        """

    @abstractmethod
    @contract(returns='list(tuple(str,*))')    
    def get_published_topics(self):
        """ 
            Returns the list of topics that we need to publish to.
            This is a list of tuples (topic, data_class).
        """

    @abstractmethod
    @contract(returns=StreamSpec)
    def get_stream_spec(self):
        ''' Returns the sensorimotor spec for this robot
            (a BootSpec object). '''
    
    
    @abstractmethod
    @contract(messages='dict(str:*)', returns='None|tuple(str,array)')
    def commands_from_messages(self, messages):
        """ 
            Converts the ROS topics listed here to a numpy array.
            Returns commands_source (name of agent), array,
            or None if it is not interpretable.
        """
        
    @abstractmethod
    @contract(returns='dict(str:*)', commands='array')
    def messages_from_commands(self, commands):
        """ Returns the messages to publish. """
       
    @abstractmethod     
    @contract(commands='array', returns='se3')
    def debug_get_vel_from_commands(self, commands):
        pass
        
