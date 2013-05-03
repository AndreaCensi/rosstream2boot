from . import JointVelocities, JointValue, JointPositions

def get_joint_velocity_msg(array, timeStamp=None):
    '''
    
    :param array:    float array with values to the joints
    '''
    
    num_joints = len(array)
    
    msg = JointVelocities()
    msg.poisonStamp.description = 'Joint velocities generated by b2r2b.'
    
    for i in range(num_joints):
        joint_value = JointValue()
        
        joint_value.joint_uri = 'arm_joint_' + str(i + 1)
        if timeStamp is not None:
            joint_value.timeStamp = timeStamp
        joint_value.unit = 's^-1 rad'
        joint_value.value = array[i]
        
        msg.velocities.append(joint_value)
        
    assert len(msg.velocities) == num_joints
    return msg



def get_joint_position_msg(array, timeStamp=None):
    '''
    
    :param array:    float array with values to the joints
    '''
    
    num_joints = len(array)
    
    msg = JointPositions()
    msg.poisonStamp.description = 'Joint velocities generated by b2r2b'
    
    for i in range(num_joints):
        joint_value = JointValue()

        joint_value.joint_uri = 'arm_joint_' + str(i + 1)
        if timeStamp is not None:
            joint_value.timeStamp = timeStamp
        joint_value.unit = 'rad'
        joint_value.value = array[i]
        
        msg.positions.append(joint_value)
        
    assert len(msg.positions) == num_joints
    return msg
