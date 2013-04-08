#!/usr/bin/env python
from .. import logger
from quickapp import QuickApp
from rosstream2boot.config.rbconfig import get_rs2b_config, set_rs2b_config
from rosstream2boot.interfaces.ros_log import MultiLog
import sys

class RS2BConvert(QuickApp):
    
    def define_options(self, params):
        params.add_string_list('config', help='Configuration directory')
        # params.add_flag('verbose', help='Instances all configuration')
        
        # params.add_string_list('sets', help='List of sets (defined in rs2b_batch.yaml)')
        params.add_string('set', help='List of set (defined in rs2b_batch.yaml)',
                          compulsory=True)
        
    def define_jobs(self):
        options = self.get_options()
        
        config = get_rs2b_config()

        config.load(config.get_default_dir())
        if options.config is not None:
            for d in options.config:
                config.load(d)
        
        # config.print_summary(sys.stdout, instance=options.verbose)
        
        self.logger.info('Set: %s' % options.set)

        spec = config.convert_sets[options.set] 
        
        adapters = config.adapters.expand_names(spec['adapters']) 
        logs = config.explogs.expand_names(spec['logs']) 
        
        self.logger.info('Adapters: %s' % adapters)   
        self.logger.info('Logs: %s' % logs)
        
        for id_adapter in adapters: 
            self.comp(convert_job, config, id_adapter, logs)
                
import numpy as np

def convert_job(config, id_adapter, logs):
    set_rs2b_config(config)
    log = MultiLog(logs)
    adapter = config.adapters.instance(id_adapter)
    spec = adapter.get_spec()
    
    topics = set(adapter.get_relevant_topics())
    
    
    # Topic to last message
    topic2last = {}
    last_t = None
    for topic, msg, t, extra in log.read_all(topics=topics):
        if last_t is not None:
            diff = np.abs((t - last_t).to_sec())
#             print diff
        if extra['counter'] % 10 == 0:
            print('percentage: %s' % extra['t_percentage'])
            
        last_t = t 
        
#         print('%s %s' % (t, topic))
        assert topic in topics
        topic2last[topic] = msg
        
        # If we received all topics
        if len(topic2last) == len(topics):
            # then trigger an update
            pass
            # observations = adapter.observations_from_messages(topic2last)
            # commands = adapter.commands_from_messages(topic2last)
            
            
            
            
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
    
    
def main_convert():
    
    sys.exit(RS2BConvert().main())
    
if __name__ == '__main__':
    main_convert()
    
    

 

