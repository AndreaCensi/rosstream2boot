from quickapp.library.app_commands.app_with_commands import (QuickMultiCmdApp,
    QuickMultiCmd, add_subcommand)
from rosstream2boot.config.rbconfig import get_rs2b_config
from bootstrapping_olympics.configuration.master import get_boot_config
from quickapp.library.app.quickapp_imp import quickapp_main


class RS2B(QuickMultiCmdApp):
    
    cmd = 'rs2b'
    short = 'Main program'
    
    def define_multicmd_options(self, params):
        params.add_flag('dummy', help='workaround for a bug')
        params.add_string_list('config', help='Configuration directory')
  
    def initial_setup(self):
        config = get_rs2b_config()
        config.load(config.get_default_dir())
        
        options = self.get_options()
        config_dirs = options.config
        if config_dirs is None:
            config_dirs = []
            
        rs2b_config = get_rs2b_config()
        rs2b_config.load_dirs(config_dirs)
        
        boot_config = get_boot_config()
        boot_config.load_dirs(config_dirs)
        
        self.rs2b_config = config
        self.boot_config = boot_config
                  
                 
                             
class RS2BCmd(QuickMultiCmd):
    
    def get_rs2b_config(self):
        return self.get_parent().rs2b_config

    def get_boot_config(self):
        return self.get_parent().boot_config


def RS2Bsub(x):  # decorator
    add_subcommand(RS2B, x)
    return x


def main_rs2b():
    quickapp_main(RS2B)
