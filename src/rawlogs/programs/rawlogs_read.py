from .main import RawlogsMainCmd
from quickapp import QuickAppBase
from rawlogs import get_conftools_rawlogs

__all___ = ['RawLogsInfo']


class RawLogsRead(RawlogsMainCmd, QuickAppBase):
    """ Tries to read a log """
    
    cmd = 'read'
    
    def define_program_options(self, params):
        params.add_string("rawlog", help="Raw log ID")
        params.add_string_list("signals", help="Signals", default=[])

    def go(self):
        id_rawlog = self.options.rawlog
        rawlog = get_conftools_rawlogs().instance(id_rawlog)
        
        signals = self.options.signals
        read_log(rawlog, signals=signals)
        
    
def read_log(rawlog, signals=None, start=None, stop=None):
   
    if not signals:
        signals = list(rawlog.get_signals().keys())
        
    for timestamp, (name, value) in rawlog.read(signals, start, stop):
        print 'reading %s %s' % (timestamp, name)
        
