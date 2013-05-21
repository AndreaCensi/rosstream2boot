from conf_tools import ConfigMaster

class RBConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'rs2b')

        from ..interfaces import (ROSObservationsAdapter, ROSCommandsAdapter,
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

        # from rosstream2boot.programs import ConvertJob
#         self.convert_jobs = self.add_class_generic('convert_jobs',
#                                                    '*.convert_jobs.yaml',
#                                                    ConvertJob)
        
    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("rosstream2boot", "configs")


get_rs2b_config = RBConfigMaster.get_singleton
set_rs2b_config = RBConfigMaster.set_singleton

