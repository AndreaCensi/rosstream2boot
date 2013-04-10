from contracts import contract
from bootstrapping_olympics.interfaces.boot_spec import BootSpec
from rosstream2boot.config.rbconfig import get_rs2b_config
from bootstrapping_olympics.interfaces import RobotObservations


class ROSRobotAdapter:
    
    OBS_FIRST_TOPIC = 'obs-first-topic'
    
    def __init__(self, obs_adapter, cmd_adapter, sync):
        self.obs_adapter = obs_adapter
        self.cmd_adapter = cmd_adapter
        self.obs_spec = self.obs_adapter.get_stream_spec()
        self.cmd_spec = self.cmd_adapter.get_stream_spec()
        self.boot_spec = BootSpec(self.obs_spec, self.cmd_spec)
        
        self.sync_policy = sync['policy']
        assert sync['policy'] in [ROSRobotAdapter.OBS_FIRST_TOPIC]    
        
    @contract(returns='list(str)')    
    def get_relevant_topics(self):
        """ Returns the list of topics that are relevant for us. """
        self.obs_topics = self.obs_adapter.get_relevant_topics()
        self.cmd_topics = self.cmd_adapter.get_relevant_topics()
        return self.obs_topics + self.cmd_topics

    @contract(returns=BootSpec)
    def get_spec(self):
        return self.boot_spec

    @contract(returns='dict(str:*)')
    def messages_from_commands(self, command):
        """ Returns a dictionary string -> Message """
        return self.cmd_adapter.msgs_from_commands(command)

    @contract(returns='None|RobotObservations', messages='dict(str:*)')
    def get_observations(self, messages, last_topic, last_msg, last_t):
        """
            messages: topic -> last ROS Message
            returns: an instance of RobotObservations, or None if the update is not ready.
                
        """
        if self.is_ready(messages, last_topic, last_msg, last_t):    
            observations = self.obs_adapter.observations_from_messages(messages)
            commands_source, commands = self.cmd_adapter.commands_from_messages(messages)            
            episode_end = False
            timestamp = last_t.to_sec()
            robot_pose = None
            return RobotObservations(timestamp=timestamp,
                                     observations=observations,
                                     commands=commands, commands_source=commands_source,
                                     episode_end=episode_end,
                                     robot_pose=robot_pose)
        
    def is_ready(self, messages, last_topic, last_msg, last_t):  # @UnusedVariable
        if self.sync_policy == ROSRobotAdapter.OBS_FIRST_TOPIC:
            return last_topic == self.obs_topics[0]
        else:
            raise NotImplementedError
 
    @staticmethod
    def from_yaml(obs_adapter, cmd_adapter, sync):
        config = get_rs2b_config()
        obs_adapter = config.obs_adapters.instance(obs_adapter)
        cmd_adapter = config.cmd_adapters.instance(cmd_adapter)
        return ROSRobotAdapter(obs_adapter, cmd_adapter, sync)

# ROSRobotAdapter_from_yaml
    
