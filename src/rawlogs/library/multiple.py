from .logwithannotations import LogWithAnnotations
from contracts import contract
from rawlogs import get_conftools_rawlogs
import heapq

__all__ = ['Multiple']


class Multiple(LogWithAnnotations):    

    @contract(annotations='dict', logs='list(str|code_spec)')
    def __init__(self, logs, annotations={}):
        LogWithAnnotations.__init__(self, annotations)
        library = get_conftools_rawlogs()
        self._logs = [library.instance_smarter(l)[1] for l in logs]
        
    def get_signals(self):
        signals = {}
        for log in self._logs:
            log_signals = log.get_signals()
            for k in log_signals:
                if k in signals:
                    msg = 'Signal %r repeated' % k
                    raise ValueError(msg)
            signals.update(log_signals)
        return signals

    def get_time_bounds(self):
        """ Returns a tuple of floats representing approximate start and end times. """ 
        raise Exception()
    
    def get_resources(self): 
        """ Returns the physical files needed for this log """
        resources = []
        for l in self._logs:
            lres = l.get_resources()
            resources.extend(lres)
        return resources
        
    def read(self, topics, start=None, stop=None):
        found = set()
        iterators = []
        for l in self._logs:
            log_signals = list(set(l.get_signals().keys()) & set(topics))
            found.update(log_signals)
            if log_signals: 
                it = l.read(log_signals, start=start, stop=stop)
#                 iterators.append(_add_ts(it))
                iterators.append(it)
            
        notfound = set(topics) - found
        if notfound:
            msg = 'Could not find signals %s.\nKnown: %s.' % (notfound, self.get_signals().keys())
            raise ValueError(msg)

        it = heapq.merge(*tuple(iterators))
        for x in it:
            yield x

        




