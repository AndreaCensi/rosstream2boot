from contracts import contract
from rawlogs import RawLog, get_conftools_rawlogs


__all__ = ['RemoveSignals']


class RemoveSignals(RawLog):
    """ Removes the given signals from the log """    

    @contract(rawlog=str, signals='list(str)')
    def __init__(self, rawlog, signals):
        library = get_conftools_rawlogs()
        self.log = library.instance(rawlog)
        self.signals = signals
        
    def get_signals(self):
        signals = self.log.get_signals()
        for s in self.signals:
            if s in signals:
                del signals[s]
        return signals

    def get_time_bounds(self):
        """ Returns a tuple of floats representing approximate start and end times. """ 
        return self.log.get_time_bounds()
    
    def get_resources(self): 
        """ Returns the physical files needed for this log """
        return self.log.get_resources()
    
    def read(self, topics, start=None, stop=None):
        for x in self.log.read(topics, start=start, stop=stop):
            yield x
