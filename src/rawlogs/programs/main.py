from conf_tools import GlobalConfig
from quickapp import QuickMultiCmdApp

__all__ = ['RawlogsMain', 'RawlogsMainCmd', 'rawlogs_main']


class RawlogsMain(QuickMultiCmdApp):
    """ Main campaign program """
    
    cmd = 'rawlogs'
    
    def define_multicmd_options(self, params):
        params.add_string("config", help="Configuration dirs", default='')

    def initial_setup(self):
        GlobalConfig.global_load_dir(self.options.config)
                    
                             
class RawlogsMainCmd(RawlogsMain.get_sub()):
    pass


rawlogs_main = RawlogsMain.get_sys_main()
