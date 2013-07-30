from conf_tools import ConfigMaster

__all__ = ['get_conftools_rawlogs']

class RawlogsConfig(ConfigMaster):
    
    def __init__(self):
        ConfigMaster.__init__(self, 'rawlogs')

        from rawlogs.interface import RawLog
        
        self.rawlogs = self.add_class_generic('rawlogs', '*.rawlogs.yaml', RawLog)        
        
    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("rawlogs", "configs")


get_rawlogs_config = RawlogsConfig.get_singleton


def get_conftools_rawlogs():
    return get_rawlogs_config().rawlogs



