from contracts import contract
from rosstream2boot.config.rbconfig import get_rs2b_config
from abc import abstractmethod, ABCMeta
import rosbag
from .. import logger
from procgraph_ros.bag_utils import read_bag_stats


class ExperimentLog:
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def read_all(self, topics):
        """ Yields topic, msg, t """
    
 
class ExpLogFromYaml(ExperimentLog):
    
    @contract(files='dict(str:str)', annotations='dict(str:*)')
    def __init__(self, files, annotations):
        self.files = files
        self.annotations = annotations

    def read_all(self, topics):
        bagfile = self.files['bag']
#         logger.debug('Reading bag file: %s' % bagfile)
#         bag = rosbag.Bag(bagfile)
#         logger.debug('- bag opened')
#         first = True
        return read_bag_stats(bagfile, topics=topics, logger=logger)
#         for topic, msg, t in bag.read_messages(topics=topics):
#             if first:
#                 logger.debug('- first arrived')
#                 first = False
#             yield topic, msg, t


class LogSlice(ExperimentLog):
    
    def __init__(self, id_log, t0, duration):
        self.id_log = id_log
        self.t0 = t0
        self.duration = duration
        
        self.log = get_rs2b_config().explogs  # XXX


class MultiLog(ExperimentLog):
    
    def __init__(self, logs):
        config = get_rs2b_config()
        
        logs = config.explogs.expand_names(logs)
        self.logs = []
        
        for id_log in logs:
            log = config.explogs.instance(id_log)
            self.logs.append(log)

    def read_all(self, topics):
        for l in self.logs:
            # Change episode?
            for m in l.read_all(topics):
                yield m
                
                
                
                 
