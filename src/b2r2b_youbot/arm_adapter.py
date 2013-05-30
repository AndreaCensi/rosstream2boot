from .message import get_joint_velocity_msg, get_joint_position_msg
from bootstrapping_olympics import StreamSpec, make_streamels_1D_float
from contracts import contract
from rosstream2boot import ROSCommandsAdapter
import numpy as np
import warnings
from b2r2b_youbot import get_JointVelocities, get_JointPositions


names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4',
         'arm_joint_5']

__all__ = ['YoubotArm']

class YoubotArm(ROSCommandsAdapter):
    
    @contract(max_vel='seq[5](>=0)',
              joint_home='seq[5](number)',
              joint_min='seq[5](number)',
              joint_max='seq[5](number)')
    def __init__(self, arm, max_vel, joint_home, joint_min, joint_max):
        self.topic_joint_state = '/joint_states' 
        self.topic_pos_cmd = '%s/arm_controller/position_command' % arm 
        self.topic_vel_cmd = '%s/arm_controller/velocity_command' % arm  
        self.joint2pos = {}
        self.joint2vel = {}
        self.max_vel = np.array(max_vel).astype('float32')
        
        self.joint_home = np.array(joint_home).astype('float32')
        self.joint_max = np.array(joint_max).astype('float32')
        self.joint_min = np.array(joint_min).astype('float32')
        for i in range(5):
            h = self.joint_home[i]
            m = self.joint_min[i]
            M = self.joint_max[i]
            
            if not(m <= h <= M):
                msg = 'Joint %d info is not consistent.' % i
                msg += '\n min: %s' % self.joint_min
                msg += '\nhome: %s' % self.joint_home
                msg += '\n max: %s' % self.joint_max
                raise ValueError(msg)
        
    def get_relevant_topics(self):
        from sensor_msgs.msg import JointState
        t = []
        t += [(self.topic_joint_state, JointState)]
        return t

    def get_published_topics(self):
        JointPositions = get_JointPositions()
        JointVelocities = get_JointVelocities()
        t = []
        t += [(self.topic_vel_cmd, JointVelocities)]
        t += [(self.topic_pos_cmd, JointPositions)]
        return t
    
    def get_stream_spec(self):
        n = 5        
        streamels = make_streamels_1D_float(nstreamels=n, lower=(-1), upper=1)
        return StreamSpec(id_stream=None, streamels=streamels, extra=None)
    
    
    #### 
    
    def read_joint_states(self, messages):
        msg = messages[self.topic_joint_state]
        for name, pos in zip(msg.name, msg.position):
            self.joint2pos[name] = pos
        for name, vel in zip(msg.name, msg.velocity):
            self.joint2vel[name] = vel
            
    def get_joint_pos_vel(self):
        if not all(name in self.joint2pos for name in names):
            print('not ready yet')
            return None, None
        
        pos = [self.joint2pos[name] for name in names]
        vel = [self.joint2vel[name] for name in names]
        return np.array(pos), np.array(vel)
    
    def commands_from_messages(self, messages):
        self.read_joint_states(messages)
        self.pos, vel = self.get_joint_pos_vel()
        if vel is None:
            return None
        commands = self.commands_from_jointvels(vel)
        commands_source = 'arm'  # XXX
        warnings.warn('Must implement proper commands_source.')
        return commands_source, commands
    
    def commands_from_jointvels(self, vel):
        warnings.warn('Check feasible')
        zeros = self.max_vel == 0
        cmds = np.clip(vel / self.max_vel, -1, +1)
        cmds[zeros] = 0
        return cmds
    
    def jointvels_from_commands(self, commands):
        warnings.warn('Check feasible')
        commands = np.clip(commands, -1, +1)
        return commands * self.max_vel
            
    def is_in_safe_zone(self):
        """ Returns True if the joints are in our safe zone. """
        pos, _ = self.get_joint_pos_vel()
        if pos is None:
            print('No messages arrived yet')
            return False
        
        t1 = pos <= self.joint_max
        t2 = pos >= self.joint_min
        const = np.logical_and(t1, t2)
        ok = not np.all(const)
        if not ok:
            msg = 'Out of safe zone:'
            p = self._joint_string
            msg += '\n min' + p(self.joint_min)
            msg += '\n max' + p(self.joint_max)
            msg += '\n cur' + p(pos)
            msg += '\n ok?' + p(const)
            print(msg)
        return ok 
        
    def _joint_string(self, v):
        return ' '.join('%5.3f' % x for x in v)
        
    @contract(returns='dict(str:*)', commands='array[5]')
    def messages_from_commands(self, commands):
        if self.is_in_safe_zone(): 
            jointvels = self.jointvels_from_commands(commands)
            msg = get_joint_velocity_msg(jointvels)
            return {self.topic_vel_cmd: msg}
        else:
            print('Out of safe zone; sending commands')
            pos, _ = self.get_joint_pos_vel()
            if pos is not None:
                print('cur: %s' % self._joint_string(pos))
            # Send message to go back to safe zone
            msg = get_joint_position_msg(self.joint_home)
            return {self.topic_pos_cmd: msg}
        
    def __str__(self):
        return ('YoubotArm')

