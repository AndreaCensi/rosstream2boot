from bootstrapping_olympics import get_boot_config
from bootstrapping_olympics.library.robots import EquivRobot
from bootstrapping_olympics.misc import bd_sequence_from_robot
from contracts import describe_type
from procgraph import Block
from procgraph.block_utils import IteratorGenerator
from rosstream2boot.library import ROSRobot
import warnings


class ROSRobotRead(IteratorGenerator):
    ''' 
        This block reads a bag file (ROS logging format).
    '''
    Block.alias('rosrobot_read')
    
    Block.output('bd', 'Boot data array')
    
    Block.config('file', 'Bag file to read')
    Block.config('id_robot', 'ROSRobot instance')
      
    
    def init_iterator(self):
        id_robot = self.config.id_robot
        bagfile = self.config.file
        
        boot_config = get_boot_config()
        robot = boot_config.robots.instance(id_robot)
        
        warnings.warn('Make this more general')
        if isinstance(robot, EquivRobot):
            orig_robot = robot.get_original_robot()
        else:
            orig_robot = robot
        
        if not isinstance(orig_robot, ROSRobot):
            msg = 'Expected ROSRobot, got %s' % describe_type(robot)
            raise ValueError(msg)
    
        orig_robot.read_from_bag(bagfile)

        bd_seq = bd_sequence_from_robot(id_robot, robot, sleep_wait=0,
                                        id_episode='xxx', id_environment='xxx',
                                        check_valid_values=False)
        
        def make_boot_iterator():
            for bd in bd_seq:
                yield 0, bd['timestamp'].item(), bd
        
        return make_boot_iterator()
        


