from rosstream2boot import get_rs2b_config
from .rs2b import RS2B
import sys

class List(RS2B.get_sub()):
    
    cmd = 'list'
    
    def define_program_options(self, params):
        params.add_flag('verbose', help='Instances all configuration')        
        config = get_rs2b_config()
        classes = config.get_classes() 
        examples = ', '.join(classes)
        helps = 'Only print one type of objects (%s)' % examples,
        params.add_string('type', help=helps, default=None)
        
    def go(self):
        options = self.get_options()        
        config = get_rs2b_config()
        config.print_summary(sys.stdout, instance=options.verbose,
                             only_type=options.type)
         
