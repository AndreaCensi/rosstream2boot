from conf_tools import ConfigMaster

__all__ = ['get_rs2b_config', 'get_conftools_cmd_adapters',
           'get_conftools_obs_adapters',
           'get_conftools_robot_adapters',
           'get_conftools_explogs']

class RBConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'rs2b')

        from rosstream2boot import (ROSObservationsAdapter, ROSCommandsAdapter,
                                  ROSRobotAdapter, ExperimentLog)
        
        self.explogs = self.add_class_generic('explogs', '*.explogs.yaml', ExperimentLog)
        
        self.adapters = self.add_class_generic('adapters',
                                               '*.robot_adapters.yaml',
                                               ROSRobotAdapter)
        
        self.obs_adapters = self.add_class_generic('obs_adapters',
                                                   '*.obs_adapters.yaml',
                                                    ROSObservationsAdapter)
        
        self.cmd_adapters = self.add_class_generic('cmd_adapters',
                                                   '*.cmd_adapters.yaml',
                                                   ROSCommandsAdapter)

  
    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("rosstream2boot", "configs")


get_rs2b_config = RBConfigMaster.get_singleton


def get_conftools_robot_adapters():
    return get_rs2b_config().adapters

def get_conftools_obs_adapters():
    return get_rs2b_config().obs_adapters

def get_conftools_cmd_adapters():
    return get_rs2b_config().cmd_adapters

def get_conftools_explogs():
    return get_rs2b_config().explogs
