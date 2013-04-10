#!/usr/bin/env python
from quickapp import QuickApp
import sys
from rosstream2boot.config.rbconfig import get_rs2b_config

class JustList(QuickApp):
    
    def define_options(self, params):
        params.add_string_list('config', help='Configuration directory')
        params.add_flag('verbose', help='Instances all configuration')
        
        config = get_rs2b_config()
        classes = config.get_classes() 
        examples = ', '.join(classes)
        params.add_string('type', help='Only print one type of objects (%s)' % examples)
        
    def define_jobs(self):
        options = self.get_options()
        
        config = get_rs2b_config()

        config.load(config.get_default_dir())
        if options.config:
            for d in options.config:
                config.load(d)
        
        config.print_summary(sys.stdout, instance=options.verbose, only_type=options.type)
        
#         for id_explog in R2BConfig.explogs:
#             self.comp(list_explog, config, id_explog)
# 
#         for id_adapter in R2BConfig.adapters:
#             self.comp(list_adapter, config, id_adapter)

# 
# def list_explog(config, id_explog):
#     explog = config.explogs.instance(id_explog)
#     
# 
# def list_adapter(config, id_adapter):
#     adapter = config.id_adapter.instance(id_adapter)
    
    
def main_list():
    
    sys.exit(JustList().main())
    
if __name__ == '__main__':
    main_list()
    
    
