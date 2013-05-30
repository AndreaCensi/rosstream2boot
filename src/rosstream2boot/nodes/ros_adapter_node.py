from contracts import contract
from rosstream2boot import ROSRobotAdapter
from ros_node_utils.nodes.ros_node import ROSNode
from bootstrapping_olympics.interfaces.observations import ObsKeeper
import warnings

__all__ = ['ROSRobotAdapterNode']


class ROSRobotAdapterNode(ROSNode):
    
    """
    
        Deprecated --- Use RosRobot
        
        This node provides online BootObservations 
        according to a ROSRobotAdapter.
        
        Messages:
        ~boot_observations_ready, String: announces 
        that there are observations ready    
    """
    
    def __init__(self):
        ROSNode.__init__(self, 'ROSObsAdapterNode') 
        self.buffer = []
        self.counter = 0
        
    @contract(adapter=ROSRobotAdapter)    
    def init_messages(self, adapter):
        self.topic2message = {}
        self.adapter = adapter
        
        warnings.warn('id_robot')
        boot_spec = self.adapter.get_spec()
        self.obs_keeper = ObsKeeper(boot_spec, id_robot='XXX',
                                    check_valid_values=True)
        
        self.init_messages_subscribers()
        self.init_messages_publishers()
        
        import rospy
        from std_msgs.msg import String
        self.pub_ready = rospy.Publisher('~boot_observations_ready', String)

    def init_messages_subscribers(self):
        # list of tuples (topic, data_class)
        topics = self.adapter.get_relevant_topics()
        
        import rospy

        for topic, data_class in topics:
            self.info('Subscribing to %s' % topic)
            rospy.Subscriber(topic, data_class,
                             self.callback, callback_args=topic)

    def init_messages_publishers(self):
        import rospy
        self.publishers = {}
        pub_topics = self.adapter.get_published_topics()
        for topic, data_class in pub_topics:
            self.info('Publishing to %s' % topic)
            self.publishers[topic] = rospy.Publisher(topic, data_class)
    
    def callback(self, msg, topic):
        # self.info('Received %s' % topic)
        if not topic in self.topic2message:
            self.info('Received first %s' % topic)
        self.topic2message[topic] = msg
        from rospy.rostime import Time
        self.check_ready(topic, msg, Time.now())
        
    def check_ready(self, last_topic, last_msg, last_t):
        obs = self.adapter.get_observations(self.topic2message,
                                            last_topic, last_msg, last_t)  
        if obs is None:
            # not ready
            # self.info('Not ready yet msgs=%s last:%s' % (self.topic2message.keys(), last_topic))
            pass
        else:
            timestamp = last_t.to_sec()
            observations = obs.observations
            commands = obs.commands
            commands_source = obs.commands_source
            warnings.warn('fill these up')
            id_episode = 'XXX'  # XXX
            id_world = 'xxx'  # XXX
            obs_array = self.obs_keeper.push(timestamp, observations,
                                             commands, commands_source,
                                             id_episode, id_world)
        
            self.buffer.append(obs_array)
            
            msg = 'BootObservations ready (buf: %s)' % len(self.buffer)
            # self.info(msg)
            from std_msgs.msg import String
            self.pub_ready.publish(String(msg))
            
            if self.counter == 0:
                self.info('Published first boot observations.')
            self.counter += 1

    def obs_callback(self, callback):
        """ Gives a callback to be called when boot observations are ready """
        from std_msgs.msg import String
        import rospy
        rospy.Subscriber('~boot_observations_ready', String, callback)
            
    @contract(returns='list')
    def get_boot_observations_buffer(self):
        b = self.buffer
        self.buffer = []
        return b
    
    def send_commands(self, commands):
        messages = self.adapter.messages_from_commands(commands)
        for topic, msg in messages.items():
            self.publishers[topic].publish(msg)
              
        
