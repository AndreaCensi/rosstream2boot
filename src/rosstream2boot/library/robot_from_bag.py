from .. import logger
from bootstrapping_olympics import BootSpec
from bootstrapping_olympics.interfaces.robot import (RobotObservations,
    PassiveRobotInterface, EpisodeDesc)
from bootstrapping_olympics.utils import unique_timestamp_string
from contracts import contract
from ros_node_utils import ROSNode
from rosbag_utils import read_bag_stats, read_bag_stats_progress, rosbag_info, resolve_topics
from rospy import ROSException
from rospy.rostime import Time
from rosstream2boot import ROSRobotAdapter
from rosstream2boot.config.rbconfig import get_rs2b_config
from rosstream2boot.interfaces import ExperimentLog
import Queue
import warnings

   
class ROSRobot(PassiveRobotInterface, ROSNode):
    """
    
        self.asked: list of topics asked; might include "*"
        self.resolved: list of topics resolved
        self.resolved2asked: map between the two
        
        self.topic2last: last message
    """
    @contract(adapter=ROSRobotAdapter)
    def __init__(self, adapter):
        ROSNode.__init__(self)
        self.adapter = adapter
        self.iterator = None
        
    @contract(returns=BootSpec)
    def get_spec(self):
        return self.adapter.get_spec()
    
    @contract(returns=RobotObservations)
    def get_observations(self):
        self.make_sure_initialized()
        try:
            topic, msg, t, _ = self.iterator()  
        except RobotObservations.NotReady:
            raise
        except StopIteration:
            raise RobotObservations.Finished()
        
        asked = self.resolved2asked[topic]
        self.topic2last[asked] = msg
        if len(self.topic2last) != len(self.resolved):
            raise RobotObservations.NotReady()
        
        obs = self.adapter.get_observations(self.topic2last, asked, msg, t)
        if obs is None:
            raise RobotObservations.NotReady()
            
        return obs        

    @staticmethod
    def from_yaml(adapter):
        rs2b_config = get_rs2b_config()
        _, adapter = rs2b_config.adapters.instance_smarter(adapter)
        return ROSRobot(adapter)

    def read_from_bag(self, bagfile):
        bag_info = rosbag_info(bagfile)

        self.asked = [name for name, _ in self.adapter.get_relevant_topics()]
                  
        self.resolved = resolve_topics(bag_info, self.asked)
        
        self.resolved2asked = {}
        for a, t in zip(self.asked, self.resolved):
            self.resolved2asked[t] = a
        
        self.info('   asked: %s ' % self.asked)
        self.info('resolved: %s ' % self.resolved)
        
        source = read_bag_stats(bagfile, topics=self.resolved)
        show_progress = read_bag_stats_progress(source, logger, interval=5)

        # Topic to last message
        self.topic2last = {}
        self.iterator = show_progress.next
        

    @contract(explog=ExperimentLog)
    def read_from_log(self, explog):
        self.id_environment = explog.get_id_environment()
        bagfile = explog.get_bagfile()
        self.read_from_bag(bagfile)

    def connect_to_ros(self):
        """ Connects to the ROS nodes in a live system. """
        import rospy
        known = [t['topic'] for t in rospy.get_published_topics()]
        warnings.warn('temporary')
        try:
            rospy.init_node('boot_interface', disable_signals=True)
        except ROSException as e:
            self.debug('Ignoring %s' % e)
            
        self.publishers = {}
        pub_topics = self.adapter.get_published_topics()
        for topic, data_class in pub_topics:
            self.info('Publishing to %s' % topic)
            publisher = rospy.Publisher(topic, data_class, latch=True)
            self.publishers[topic] = publisher
            
        self.queue = Queue.Queue()
        
        def callback(msg, topic):
            what = (topic, msg, Time.now(), {})
            self.queue.put(what)
        
        topics = self.adapter.get_relevant_topics()
        for topic, data_class in topics:
            self.info('Subscribing to %s' % topic)
            rospy.Subscriber(topic, data_class,
                             callback, callback_args=topic)
        
        def next_message():
            try:
                return self.queue.get_nowait()
            except Queue.Empty:
                raise RobotObservations.NotReady()
            except:
                raise
                
        self.iterator = next_message 
    
        self.send_rest_command()
        
    def send_rest_command(self):
        # Send rest commands so we can latch        
        spec = self.get_spec()
        commands = spec.get_commands().get_default_value()
        self.set_commands(commands, 'rest')     

    def set_commands(self, commands, commands_source):  # @UnusedVariable
        warnings.warn('set commands_source')
        self.make_sure_initialized()
        messages = self.adapter.messages_from_commands(commands)
        for topic, msg in messages.items():
            self.publishers[topic].publish(msg)

    def new_episode(self):
        self.send_rest_command()
        id_episode = unique_timestamp_string()
        extra = None
        warnings.warn('get environment from some place')
        id_environment = 'lab'
        desc = EpisodeDesc(id_episode, id_environment, extra)
        return desc
  
    def make_sure_initialized(self):
        """ 
            Checks that either connect_to_ros() or read_log() was
            called. If neithet, use connect_to_ros(). 
        """    
        if self.iterator is None:
            self.connect_to_ros()

