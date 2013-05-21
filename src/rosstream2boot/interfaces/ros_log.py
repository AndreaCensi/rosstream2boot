from .. import logger
from abc import abstractmethod, ABCMeta
from contracts import contract
from rosbag_utils import read_bag_stats
from rosstream2boot import get_rs2b_config


class ExperimentLog:
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def read_all(self, topics):
        """ Yields topic, msg, t, extra """
    
    @abstractmethod
    def get_id_environment(self):
        pass
 
 
class ExpLogFromYaml(ExperimentLog):
    
    @contract(files='dict(str:str)', annotations='dict(str:*)')
    def __init__(self, files, annotations):
        """
            List of annotations used:
            
                annotations['environment']['name'] => id_environment used
        
        """
        self.files = files
        self.annotations = annotations

    def get_outside_movie(self):
        """ REturns filename for external movie, or None if not available. """
        return self.files.get('outside', None)
    
    def get_bagfile(self):
        return self.files['bag']
    
    def read_all(self, topics):
        bagfile = self.get_bagfile()
        return read_bag_stats(bagfile, topics=topics, logger=logger)

    def get_id_environment(self):
        return self.annotations['environment']['name']

    def get_metadata(self):
        md = {}
        a = self.annotations
        md['robot_name'] = a['robot']['name']        
        md['robot_shape'] = a['robot']['shape']
        md['robot_profile'] = a['robot']['profile']
                
        md['environment_name'] = a['environment']['name']
        md['environment_config'] = a['environment']['config']
        return md
        
#         annotations:
#             robot: &unicornA_base1
#               name: unicorn
#               shape: unicornA
#               profile: base1
#             motion:
#               agent: babbler  
#             environment: &lab_boxes3
#               name: vehicles_lab
#               config: boxes3
#     


# class LogSlice(ExperimentLog):
#     
#     def __init__(self, id_log, t0, duration):
#         self.id_log = id_log
#         self.t0 = t0
#         self.duration = duration
#         
#         self.log = get_rs2b_config().explogs  # XXX


class MultiLog(ExperimentLog):
    
    def __init__(self, logs):
        config = get_rs2b_config()
        
        logs = config.explogs.expand_names(logs)
        self.logs = []
        
        for id_log in logs:
            log = config.explogs.instance(id_log)
            self.logs.append(log)

    def get_id_environment(self):
        raise NotImplementedError()
    
    def read_all(self, topics):
        for l in self.logs:
            # Change episode?
            for m in l.read_all(topics):
                yield m
                
                
                
                 
