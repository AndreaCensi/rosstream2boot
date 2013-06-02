from abc import abstractmethod
from bootstrapping_olympics import BootSpec, RobotObservations
from contracts import contract, ContractsMeta
from geometry import (SE3, SE3_from_rotation_translation, SE2_from_SE3,
    translation_from_SE2, rotation_from_quaternion)
from rosstream2boot import get_rs2b_config, logger
import numpy as np

__all__ = ['ROSRobotAdapterInterface', 'ROSRobotAdapter']


class ROSRobotAdapterInterface(object):
    __metaclass__ = ContractsMeta
    
    @abstractmethod
    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        pass
    
    @abstractmethod
    @contract(returns='list(tuple(str,*))')    
    def get_published_topics(self):
        pass
    
    @abstractmethod
    @contract(returns=BootSpec)
    def get_spec(self):
        pass

    @contract(returns='dict(str:*)')
    def messages_from_commands(self, command):
        """ Returns a dictionary string -> Message """
        pass

    @contract(returns='None|RobotObservations', messages='dict(str:*)',
              queue='list(tuple(*,*,*))')
    def get_observations(self, messages, queue):
        """
            messages: topic -> last ROS Message received for the topic
            queue: all messages read recently
            returns: an instance of RobotObservations, or None if the update is not ready.            
        """
        pass

       
class ROSRobotAdapter(ROSRobotAdapterInterface):
    
    OBS_FIRST_TOPIC = 'obs-first-topic'
    
    def __init__(self, obs_adapter, cmd_adapter, sync, use_odom_topic=False,
                 use_tf=True):
        self.obs_adapter = obs_adapter
        self.cmd_adapter = cmd_adapter
        self.obs_spec = self.obs_adapter.get_stream_spec()
        self.cmd_spec = self.cmd_adapter.get_stream_spec()
        self.boot_spec = BootSpec(self.obs_spec, self.cmd_spec)
        
        self.sync_policy = sync['policy']
        assert sync['policy'] in [ROSRobotAdapter.OBS_FIRST_TOPIC]    
        
        self.obs_topics = self.obs_adapter.get_relevant_topics()
        self.cmd_topics = self.cmd_adapter.get_relevant_topics()
        
        from nav_msgs.msg import Odometry
        
        self.my_topics = []
        
        self.use_odom_topic = use_odom_topic
        if use_odom_topic:
            self.odom_topic = '/odom'
            self.my_topics.append((self.odom_topic, Odometry))
            self.prev_odom = None
            
#         use_tf = False
        self.use_tf = use_tf
        if self.use_tf:
            try:
                from tf.msg import tfMessage  # @UnresolvedImport
            except:
                tfMessage = None
            self.tfreader = TFReader()
            self.my_topics.append(('/tf', tfMessage))
            
        self.relevant_topics = self.obs_topics + self.cmd_topics + self.my_topics
        
        self.debug_nseen = 0
        self.debug_nskipped = 0
        
    @contract(returns='list(tuple(str,*))')    
    def get_relevant_topics(self):
        """ 
            Returns the list of topics that are relevant for us
            and we wish to subscribe to.     
        """
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

    def get_observations(self, messages, queue):
        """
            messages: topic -> last ROS Message
            returns: an instance of RobotObservations, or None if the update is not ready.
                
        """
        self.debug_nseen += 1
        
        if self.use_tf:
            for topic, msg, _ in queue:
                if topic == '/tf':
                    pose = self.tfreader.update(msg)
                    if pose is not None:
                        robot_pose = pose
                
                
        if self.is_ready(messages, queue):    
            observations = self.obs_adapter.observations_from_messages(messages)
            cmds = self.cmd_adapter.commands_from_messages(messages)
            if cmds is None:
                # we couldn't interpret a meaningful message for use
                # logger.info('Skipping because not interpretable by my cmd_adapter.')
                self.debug_nskipped += 1
                if self.debug_nskipped % 100 == 0:
                    perc = 100.0 * self.debug_nskipped / self.debug_nseen
                    msg = ('Skipped %6d/%6d  = %.1f%% because cmd_adapter could not interpret.' % 
                           (self.debug_nskipped, self.debug_nseen, perc))
                    logger.info(msg)
                return None
            
            commands_source, commands = cmds
            episode_end = False
            _, _, last_t = queue[-1]
            timestamp = last_t.to_sec()
            
            robot_pose = None
            
            if self.use_odom_topic:
                odometry = messages[self.odom_topic]
                ros_pose = odometry.pose.pose
                x = ros_pose.position.x
                y = ros_pose.position.y
                theta = ros_pose.orientation.z
                from geometry import SE3_from_SE2, SE2_from_translation_angle
                robot_pose = SE3_from_SE2(SE2_from_translation_angle([x, y], theta))
                # print('odom: %s' % SE2.friendly(SE2_from_SE3(robot_pose)))

                if self.prev_odom is not None:
                    delta = SE3.multiply(SE3.inverse(self.prev_odom), robot_pose)
                    # print('odom delta: %s' % SE2.friendly(SE2_from_SE3(delta)))
       
                self.prev_odom = robot_pose
            
            if self.use_tf:
                robot_pose = self.tfreader.get_pose()
            
            return RobotObservations(timestamp=timestamp,
                                     observations=observations,
                                     commands=commands, commands_source=commands_source,
                                     episode_end=episode_end,
                                     robot_pose=robot_pose)
        else:
            return None
        
    def is_ready(self, messages, queue):  # @UnusedVariable
        # first check that we have all topics
        for topic, _ in self.relevant_topics:
            if not topic in messages:
                # print('Topic %r not found' % topic)
                return False
            
        if self.sync_policy == ROSRobotAdapter.OBS_FIRST_TOPIC:
            sync_topic = self.obs_topics[0][0]
            assert isinstance(sync_topic, str)
            last_topics = [x[0] for x in queue]
            ready = sync_topic in last_topics
            if not ready:
                # print('not ready  because %r ? %r ' % (last_topic, sync_topic))
                pass
            return ready
        else:
            raise NotImplementedError
 
    @staticmethod
    def from_yaml(obs_adapter, cmd_adapter, **other):
        config = get_rs2b_config()
        _, obs_adapter = config.obs_adapters.instance_smarter(obs_adapter)
        _, cmd_adapter = config.cmd_adapters.instance_smarter(cmd_adapter)
        return ROSRobotAdapter(obs_adapter, cmd_adapter, **other)

    
class TFReader():
    
    def __init__(self):
        self.pose = None
        self.stamp = None
        
    def update(self, msg):
        for x in msg.transforms:
            self.update_(x)
                
    def update_(self, msg):
        child_frame_id = msg.child_frame_id
        if child_frame_id == '/base_footprint':
            new_pose = pose_from_ROS_transform(msg.transform)
            if False:
                if self.pose is not None:
                    delta = SE3.multiply(SE3.inverse(self.pose), new_pose)
                    x, y = translation_from_SE2(SE2_from_SE3(delta))
                    interval = (msg.header.stamp - self.stamp).to_sec()
                    print('base_delta: delta %.3f seconds %.4f y %.4f' % (interval, x, y))
            self.pose = new_pose
            self.stamp = msg.header.stamp


    def get_pose(self):
        return self.pose

def pose_from_ROS_transform(transform):
    tx = transform.translation.x
    ty = transform.translation.y
    tz = transform.translation.z
    R = rotation_from_ROS_quaternion(transform.rotation)
    t = np.array([tx, ty, tz])
    pose = SE3_from_rotation_translation(R, t)
    return pose


def rotation_from_ROS_quaternion(r):
    import numpy as np
    x, y, z, w = r.x, r.y, r.z, r.w
    # my convention: w + ix + ...
    q = np.array([w, x, y, z])
    return rotation_from_quaternion(q)
    
#         if frame_id == '/odom' and child_frame_id == look:
# #         if look in [frame_id, child_frame_id]:
#             
#             print msg
#         
#     header:
#       seq: 0
#       stamp:
#         secs: 1364960270
#         nsecs: 601320999
#       frame_id: /base_footprint
#     child_frame_id: /cam_back
#     transform:
#       translation:
#         x: -0.015
#         y: 0.0
#         z: 0.0
#       rotation:
#         x: 0.0
#         y: 0.0
#         z: 0.999999682932
#         w: 0.000796326710733
#         
        
