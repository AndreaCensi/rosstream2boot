from rosstream2boot import logger
from abc import abstractmethod
from contracts import contract, ContractsMeta
from rosbag_utils import read_bag_stats
from rawlogs import RawLog
from rawlogs_ros.rawlog_ros import ROSLog

__all__ = ['ExperimentLog', 'ExpLogFromYaml']

        
        
class ExperimentLog(object):
    __metaclass__ = ContractsMeta
    
    @abstractmethod
    def read_all(self, topics):
        """ Yields topic, msg, t, extra """
    
    @abstractmethod
    def get_id_environment(self):
        pass
    
    @abstractmethod
    @contract(returns='dict(str:str)')
    def get_files(self):
        """ Returns a list of the files associated with this log. """
 
    @abstractmethod
    @contract(returns='list(str)')
    def get_tags(self):
        """ Returns a list of tags used to organize the logs. """
        
    def get_annotations(self):
        """ """
        
        
        
class ExpLogFromYaml(ExperimentLog, RawLog):
    
    @contract(files='dict(str:str)', annotations='dict(str:*)')
    def __init__(self, files, annotations):
        """
            List of annotations used:
            
                annotations['environment']['name'] => id_environment used
        
        """
        self.files = files
        self.annotations = annotations

        self.roslog = ROSLog(self.get_bagfile(), annotations)

    def get_annotations(self):
        """ """
        return self.annotations
        
    def get_files(self):
        return dict(**self.files)
    
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

    def get_tags(self):
        return self.annotations.get('tags', [])
    
    def get_metadata(self):
        md = {}
        a = self.annotations
        md['robot_name'] = a['robot']['name']        
        md['robot_shape'] = a['robot']['shape']
        md['robot_profile'] = a['robot']['profile']
                
        md['environment_name'] = a['environment']['name']
        md['environment_config'] = a['environment']['config']
        return md
        

    # # This is the RawLog interface
    # @contract(returns='dict(str:isinstance(RawSignal))')
    # def get_signals(self):
    #     # TODO: add movie if possible
    #     return self.roslog.get_signals()

    # @contract(returns='list(str)')
    # def get_resources(self):
    #     return self.get_files()

    # def read(self, topics, start=None, stop=None):
    #     # TODO: add movie if possible
    #     for a in self.roslog.read(topics, start, stop):
    #         yield a
                
                 
