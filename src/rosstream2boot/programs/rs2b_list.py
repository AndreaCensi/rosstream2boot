#!/usr/bin/env python
import sys
from rosstream2boot.programs.rs2b import RS2BCmd, RS2B
from quickapp.library.app_commands.app_with_commands import add_subcommand


class List(RS2BCmd):
    cmd = 'list'
    
    def define_options(self, params):
        params.add_flag('verbose', help='Instances all configuration')        
        config = self.get_rs2b_config()
        classes = config.get_classes() 
        examples = ', '.join(classes)
        params.add_string('type',
                          help='Only print one type of objects (%s)' % examples,
                          default=None)
        
    def define_jobs_context(self, context):
        options = self.get_options()        
        config = self.get_rs2b_config()
        config.print_summary(sys.stdout, instance=options.verbose, only_type=options.type)
        
        
add_subcommand(RS2B, List)
 
