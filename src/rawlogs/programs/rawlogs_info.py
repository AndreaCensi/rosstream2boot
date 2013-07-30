from .main import RawlogsMainCmd
from conf_tools.utils import friendly_path
from quickapp import QuickAppBase
from rawlogs import get_conftools_rawlogs

__all___ = ['RawLogsInfo']


class RawLogsInfo(RawlogsMainCmd, QuickAppBase):
    cmd = 'info'
    
    def define_program_options(self, params):
        params.add_string("rawlog", help="Raw log ID")

    def go(self):
        id_rawlog = self.options.rawlog
        rawlog = get_conftools_rawlogs().instance(id_rawlog)

        print(summarize(rawlog))
        
        
        
def summarize(rawlog):
    s = ""
    s += 'Resources:\n'
    for x in rawlog.get_resources():
        s += ' - %s\n' % friendly_path(x)
    s += 'Signals:\n'
    for x, v in rawlog.get_signals().items():
        s += ' - %-30s: %s\n' % (x, v)

    return s
         
        
