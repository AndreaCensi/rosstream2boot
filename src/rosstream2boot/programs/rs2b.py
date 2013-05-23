from conf_tools import GlobalConfig
from quickapp import QuickMultiCmdApp
from rosstream2boot import get_rs2b_config

__all__ = ['RS2B', 'main_rs2b']

class RS2B(QuickMultiCmdApp):
    
    cmd = 'rs2b'
    description = 'Main program'

    def define_multicmd_options(self, params):
        params.add_flag('dummy', help='workaround for a bug')
        params.add_string_list('config', help='Configuration directory',
                               default=[])
  
    def initial_setup(self):        
        rs2b_config = get_rs2b_config()
        rs2b_config.load(rs2b_config.get_default_dir())

        GlobalConfig.global_load_dirs(self.options.config)
        
 


main_rs2b = RS2B.get_sys_main()
