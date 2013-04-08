from contracts import contract
from abc import abstractmethod, ABCMeta
from bootstrapping_olympics.interfaces.boot_spec import BootSpec
from bootstrapping_olympics.interfaces.stream_spec import StreamSpec
from rosstream2boot.config.rbconfig import get_rs2b_config


class ROSObservationsAdapter:
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
    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ Converts the topics listed here to a class of XX """
    


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
    @contract(messages='dict(str:*)')
    def commands_from_messages(self, messages):
        """ Converts the ROS topics listed here to a numpy array. """

    @abstractmethod
    @contract(returns='dict(str:*)')
    def messages_from_commands(self):
        """ Must return a topic or a message. """
            

class ROSRobotAdapter:
    
    def __init__(self, obs_adapter, cmd_adapter):
        self.obs_adapter = obs_adapter
        self.cmd_adapter = cmd_adapter
        self.obs_spec = self.obs_adapter.get_stream_spec()
        self.cmd_spec = self.cmd_adapter.get_stream_spec()
        self.boot_spec = BootSpec(self.obs_spec, self.cmd_spec)
        
    @contract(returns='list(str)')    
    def get_relevant_topics(self):
        """ Returns the list of topics that are relevant for us. """
        obs_topics = self.obs_adapter.get_relevant_topics()
        cmd_topics = self.cmd_adapter.get_relevant_topics()
        return obs_topics + cmd_topics

    @contract(returns=BootSpec)
    def get_spec(self):
        return self.boot_spec

    @contract(messages='dict(str:*)')
    def observations_from_messages(self, messages):
        """ Converts the topics listed here to an array respecting the spec. """
        return self.obs_adapter.observations_from_messages(messages)
    
    def commands_from_messages(self, messages):
        return self.cmd_adapter.commands_from_messages(messages)
    
    @contract(returns='dict(str:*)')
    def messages_from_commands(self, command):
        """ Returns a dictionary string -> Message """
        return self.cmd_adapter.msgs_from_commands(command)
    
        
        
        
def ROSRobotAdapter_from_yaml(obs_adapter, cmd_adapter):
    config = get_rs2b_config()
    obs_adapter = config.obs_adapters.instance(obs_adapter)
    cmd_adapter = config.cmd_adapters.instance(cmd_adapter)
    return ROSRobotAdapter(obs_adapter, cmd_adapter)


    
