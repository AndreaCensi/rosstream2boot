from bootstrapping_olympics.interfaces import RobotObservations
from bootstrapping_olympics.interfaces.boot_spec import BootSpec
from contracts import contract
from rosstream2boot.config.rbconfig import get_rs2b_config
from nav_msgs.msg._Odometry import Odometry
from geometry.poses import SE3_from_SE2, SE2_from_translation_angle



class ROSRobotAdapter(object):
    
    OBS_FIRST_TOPIC = 'obs-first-topic'
    
    def __init__(self, obs_adapter, cmd_adapter, sync):
        self.obs_adapter = obs_adapter
        self.cmd_adapter = cmd_adapter
        self.obs_spec = self.obs_adapter.get_stream_spec()
        self.cmd_spec = self.cmd_adapter.get_stream_spec()
        self.boot_spec = BootSpec(self.obs_spec, self.cmd_spec)
        
        self.sync_policy = sync['policy']
        assert sync['policy'] in [ROSRobotAdapter.OBS_FIRST_TOPIC]    
        
        self.obs_topics = self.obs_adapter.get_relevant_topics()
        self.cmd_topics = self.cmd_adapter.get_relevant_topics()
        self.odom_topic = '/odom'
        self.my_topics = [(self.odom_topic, Odometry)]
        self.relevant_topics = self.obs_topics + self.cmd_topics + self.my_topics
        
    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        """ Returns the list of topics that are relevant for us. """
        return self.relevant_topics

    @contract(returns='list(tuple(str,*))')    
    def get_published_topics(self):
        """ Returns the list of topics that we want to publish. """
        return self.cmd_adapter.get_published_topics()

    @contract(returns=BootSpec)
    def get_spec(self):
        return self.boot_spec

    @contract(returns='dict(str:*)')
    def messages_from_commands(self, command):
        """ Returns a dictionary string -> Message """
        return self.cmd_adapter.messages_from_commands(command)

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
            
            odometry = messages[self.odom_topic]
            ros_pose = odometry.pose.pose
            x = ros_pose.position.x
            y = ros_pose.position.y
            theta = ros_pose.orientation.z
            
            robot_pose = SE3_from_SE2(SE2_from_translation_angle([x, y], theta))
            
            return RobotObservations(timestamp=timestamp,
                                     observations=observations,
                                     commands=commands, commands_source=commands_source,
                                     episode_end=episode_end,
                                     robot_pose=robot_pose)
        else:
            return None
        
    def is_ready(self, messages, last_topic, last_msg, last_t):  # @UnusedVariable
        # first check that we have all topics
        for topic, _ in self.relevant_topics:
            if not topic in messages:
                return False
            
        if self.sync_policy == ROSRobotAdapter.OBS_FIRST_TOPIC:
            sync_topic = self.obs_topics[0][0]
            assert isinstance(sync_topic, str)
            ready = last_topic == sync_topic
            # rospy.loginfo('Ready: %s because %r ? %r ' % (ready, last_topic, sync_topic))
            return ready
        else:
            raise NotImplementedError
 
    @staticmethod
    def from_yaml(obs_adapter, cmd_adapter, sync):
        config = get_rs2b_config()
        obs_adapter = config.obs_adapters.instance(obs_adapter)
        cmd_adapter = config.cmd_adapters.instance(cmd_adapter)
        return ROSRobotAdapter(obs_adapter, cmd_adapter, sync)

    
