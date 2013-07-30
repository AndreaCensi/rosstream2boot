from rawlogs import RawSignal, logger
from rawlogs.library import LogWithAnnotations
from rosbag_utils import read_bag_stats, rosbag_info
import warnings

__all__ = ['ROSLog']


class ROSLog(LogWithAnnotations):
    
    def __init__(self, filename, annotations={}):
        LogWithAnnotations.__init__(self, annotations=annotations)
        self.bag = filename
        
        self._bag_info = None  # cached result
        
    def get_rosbag_info(self):
        if self._bag_info is None:
            self._bag_info = rosbag_info(self.bag)
        return self._bag_info
    
    def get_signals(self):
        """ Returns the signals available """
        info = self.get_rosbag_info()
        signals = {}
        for t in info['topics']:
            name = t['topic']
            ros_type = t['type']
            s = ROSLogSignal(signal_type=ros_type, bagfile=self.bag)
            signals[name] = s 
        return signals  

# types:
#     - type: array_msgs/FloatArray
#       md5: 788898178a3da2c3718461eecda8f714 
# topics:
#     - topic: /arm_1/arm_controller/position_command
#       type: brics_actuator/JointPositions

    def get_time_bounds(self):
        """ Returns a tuple of floats representing approximate start and end times. """ 
        raise Exception()
    
    def get_resources(self): 
        return [self.bag]

    def read(self, topics, start=None, stop=None, use_stamp_if_available=False):
        warnings.warn('implement start/stop')
        source = read_bag_stats(self.bag, topics, logger=None)
        # sequence = read_bag_stats_progress(source, logger, interval=5)
        oldt = None
        for topic, msg, t, _ in source:  
            name = topic
            
            value = msg
            
            if use_stamp_if_available:
                try:
                    stamp = msg.header.stamp
                except:
                    timestamp = t.to_sec()
                else:
                    timestamp = stamp.to_sec()
                    print(timestamp, t.to_sec())
                    if timestamp == 0:
                        timestamp = t.to_sec()
            else:
                timestamp = t.to_sec()
                
            
            if oldt is not None:
                if oldt == timestamp:
                    logger.error('Sequence with invalid timestamp? %s' % timestamp)
                    
                    
            oldt = timestamp
            yield timestamp, (name, value)
            # yield RawSignalData(name=name, timestamp=timestamp, value=value)
        

class ROSLogSignal(RawSignal):
    
    def __init__(self, signal_type, bagfile):
        self.bagfile = bagfile
        self.signal_type = signal_type
        
    def get_signal_type(self):
        return self.signal_type
                
    def get_time_reference(self):
        return 'rostime'
        
    def get_resources(self):
        return [self.bagfile]
    
    def get_time_bounds(self):
        pass
    
    def read(self, start=None, stop=None):
        pass    
    
    def get_annotations(self): 
        return {}
    
    
    
