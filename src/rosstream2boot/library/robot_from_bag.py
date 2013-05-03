from .. import logger
from bootstrapping_olympics import (RobotObservations, PassiveRobotInterface,
    EpisodeDesc, BootSpec)
from bootstrapping_olympics.utils import unique_timestamp_string
from contracts import contract
from ros_node_utils import ROSNode
from rosbag_utils import (read_bag_stats, read_bag_stats_progress, rosbag_info,
    resolve_topics)
from rosbag_utils.rosbag_flexible_read import topics_in_bag
from rospy import ROSException
from rospy.rostime import Time
from rosstream2boot import ROSRobotAdapter, get_rs2b_config
from rosstream2boot.interfaces import ExperimentLog
import Queue
import warnings

   
class ROSRobot(PassiveRobotInterface, ROSNode):
    """
    
        self.asked: list of topics asked; might include "*"
        self.resolved: list of topics resolved
        self.resolved2asked: map between the two
        
        self._topic2last: last message
    """
    @contract(adapter=ROSRobotAdapter)
    def __init__(self, adapter):
        ROSNode.__init__(self)
        self.adapter = adapter
        self.iterator = None
        
    @contract(returns=BootSpec)
    def get_spec(self):
        return self.adapter.get_spec()
    
    def _read_queue(self):
        
        read = []
        while True:
            try:
                topic, msg, t, _ = self.iterator()
                read.append((topic, msg, t))
                # self.debug('Got topic %s' % topic)  
            except RobotObservations.NotReady:
                if not read:
                    raise
                else:
                    break
            except StopIteration:
                raise RobotObservations.Finished()
            
            asked = self.resolved2asked[topic]
            if not asked in self._topic2last:
                self.info('Received first %s' % asked)
            
            self._topic2last[asked] = msg
            
        return read
        
    @contract(returns=RobotObservations)
    def get_observations(self):
        self.make_sure_initialized()
        read = self._read_queue()
        _, _, last_t = read[-1]
        last_topics = [x[0] for x in read]
        if len(last_topics) >= len(set(last_topics)):
            # print('dropping: %s')
            warnings.warn('warn that we are dropping')
            pass
        
        obs = self.adapter.get_observations(self._topic2last, last_topics, last_t)
        if obs is None:
            # self.info('Adapter not ready')
            raise RobotObservations.NotReady()
            
        return obs        

    @staticmethod
    def from_yaml(adapter):
        rs2b_config = get_rs2b_config()
        _, adapter = rs2b_config.adapters.instance_smarter(adapter)
        return ROSRobot(adapter)

    def read_from_bag(self, bagfile):
        bag_info = rosbag_info(bagfile)

        known = topics_in_bag(bag_info)

        self._match_topics_with_known(known)
        
        source = read_bag_stats(bagfile, topics=self.resolved)
        show_progress = read_bag_stats_progress(source, logger, interval=5)

        # Topic to last message
        self._topic2last = {}
        self.iterator = show_progress.next
        
    def _match_topics_with_known(self, known):
        a = self.adapter
        all_topics = a.get_relevant_topics() + a.get_published_topics()
        self.asked = [name for name, _ in all_topics]          
        match = resolve_topics(known, self.asked) 
        self.resolved, self.resolved2asked, self.asked2resolved = match
            
        self.info('   asked: %s ' % self.asked)
        self.info('resolved: %s ' % self.resolved)
        

    @contract(explog=ExperimentLog)
    def read_from_log(self, explog):
        self.id_environment = explog.get_id_environment()
        bagfile = explog.get_bagfile()
        self.read_from_bag(bagfile)

    def connect_to_ros(self):
        """ Connects to the ROS nodes in a live system. """
        import rospy
        
        known = [name for name, _ in rospy.get_published_topics()]
        self._match_topics_with_known(known)

        warnings.warn('temporary')
        try:
            rospy.init_node('boot_interface', disable_signals=True)
        except ROSException as e:
            self.debug('Ignoring %s' % e)
            
        self.publishers = {}
        pub_topics = self.adapter.get_published_topics()
        for topic, data_class in pub_topics:
            resolved = self.resolved2asked[topic]
            self.info('Publishing %s -> %s' % (topic, resolved))
            publisher = rospy.Publisher(topic, data_class, latch=True)
            self.publishers[topic] = publisher
            
        self.queue = Queue.Queue()
        
        def callback(msg, topic):
            # self.info('callback %s' % topic)
            what = (topic, msg, Time.now(), {})
            self.queue.put(what)
            # self.info('put %s into %s' % (topic, id(self.queue)))
        
        topics = self.adapter.get_relevant_topics()
        for topic, data_class in topics:
            resolved = self.resolved2asked[topic]
            self.info('Subscribing to %s -> %s' % (topic, resolved))
            rospy.Subscriber(resolved, data_class,
                             callback, callback_args=topic)
        
        def next_message():
            try:
                # self.info('next_message')
                ob = self.queue.get_nowait()
                # self.info('got message from %s' % id(self.queue))
                return ob
            except Queue.Empty:
                # self.info('next_message: empty %s' % id(self.queue))
                raise RobotObservations.NotReady()
            except:
                raise
        
        self._topic2last = {}        
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

