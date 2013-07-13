from abc import abstractmethod
from contracts import ContractsMeta, contract

__all__ = ['RawSignal', 'RawLog']

class RawSignalData(object):
    
    def __init__(self, name, timestamp, value, extra):
        self.name = name
        self.timestamp = timestamp
        self.value = value
        self.extra = extra
        

class RawSignal(object):
    """ Interface for a generic signal stream. """
    
    __metaclass__ = ContractsMeta

    @abstractmethod
    @contract(returns='str')
    def get_signal_type(self):
        """ Returns the type of this signal (string or python class) """        

    @abstractmethod
    @contract(returns='str')
    def get_time_reference(self):
        """ Returns the time reference for this signal """
        
    @abstractmethod
    @contract(returns='list(str)')
    def get_resources(self):
        pass
    
    @abstractmethod
    @contract(returns='list(str)')
    def get_annotations(self):
        """ Returns free-form dict """
        pass

    @abstractmethod
    @contract(returns='tuple(float, float)')
    def get_time_bounds(self):
        """ Returns a tuple of floats representing approximate start and end times. """ 
        pass
    
    def read(self, start=None, stop=None):
        """ Yields a sequence of RawSignalData """
        pass    
    

class RawLog(object):    
    """ 
        A log is a collection of related signals with possibly different
        reference frames.
    """
    
    __metaclass__ = ContractsMeta

    @abstractmethod
    @contract(returns='dict(str:isinstance(GenericSignal))') 
    def get_signals(self):
        """ Returns the signals available """

    @abstractmethod
    @contract(returns='tuple(float, float)')
    def get_time_bounds(self):
        """ Returns a tuple of floats representing approximate start and end times. """ 
        pass
    
    @abstractmethod
    @contract(returns='list(str)')
    def get_resources(self): 
        """ Returns the physical files needed for this log """
        

    @abstractmethod
    def read(self, topics, start=None, stop=None):
        """ 
            Yields a sequence of RawSignalData
        """
        
    @abstractmethod
    @contract(returns='list(str)')
    def get_tags(self):
        """ Returns a list of tags used to organize the logs. """
        
    @abstractmethod
    @contract(returns='list(str)')
    def get_annotations(self):
        """ Returns free-form dict """
        pass

                