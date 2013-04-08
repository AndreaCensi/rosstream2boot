
from conf_tools import ConfigMaster


class RBConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'ros2boot')

        from ..interfaces import (ROSObservationsAdapter, ROSCommandsAdapter,
                                  ROSRobotAdapter, ExperimentLog)
        
        self.explogs = self.add_class_generic('explogs', '*.rs2b_explogs.yaml', ExperimentLog)
        self.adapters = self.add_class_generic('adapters',
                                               '*.rs2b_adapters.yaml',
                                               ROSRobotAdapter)
        self.obs_adapters = self.add_class_generic('obs_adapters', '*.rs2b_obs_adapters.yaml',
                                                    ROSObservationsAdapter)
        self.cmd_adapters = self.add_class_generic('cmd_adapters', '*.rs2b_cmd_adapters.yaml',
                                                   ROSCommandsAdapter)

        self.convert_sets = self.add_class('convert_sets', '*.rs2b_convert_sets.yaml',
                                                   check_good_convert_set)


        
    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("rosstream2boot", "configs")


    singleton = None

def get_rs2b_config():
    if RBConfigMaster.singleton is None:
        RBConfigMaster.singleton = RBConfigMaster()
#         msg = 'Must call set_rs2b_config() before.'
#         raise Exception(msg)
    return RBConfigMaster.singleton 

def set_rs2b_config(c):
    RBConfigMaster.singleton = c  


def check_good_convert_set(x):
    pass  # TODO
    
