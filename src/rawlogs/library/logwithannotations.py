from rawlogs import RawLog
from contracts import contract


__all__ = ['LogWithAnnotations']


class LogWithAnnotations(RawLog):    
    """ Parses annotations from a yaml dictionary """
    
    @contract(annotations='dict')
    def __init__(self, annotations):
        self._annotations = annotations
    
    def get_tags(self):
        return self.annotations.get('tags', [])
        
    def get_annotations(self):
        return self.annotations
