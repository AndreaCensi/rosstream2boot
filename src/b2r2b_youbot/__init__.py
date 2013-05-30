# we want to be able to import it even though the packages are not installed
def load_roslib():
    import roslib; roslib.load_manifest('brics_actuator')  # @IgnorePep8
      
def get_JointVelocities():
    load_roslib()
    from brics_actuator.msg import JointVelocities  # @UnresolvedImport
    return JointVelocities

def get_JointValue():
    load_roslib()
    from brics_actuator.msg import JointValue  # @UnresolvedImport
    return JointValue
    
def get_JointPositions():
    load_roslib()
    from brics_actuator.msg import JointPositions  # @UnresolvedImport
    return JointPositions


from .message import *
from .arm_adapter import *

