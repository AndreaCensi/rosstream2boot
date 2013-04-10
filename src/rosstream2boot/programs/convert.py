from bootstrapping_olympics.interfaces.observations import ObsKeeper
from bootstrapping_olympics.logs.logs_format import LogsFormat
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral
from bootstrapping_olympics.utils import safe_makedirs
from contracts import contract
from quickapp import QuickApp
from rosstream2boot.config import get_rs2b_config, set_rs2b_config
import os
import sys

class RS2BConvert(QuickApp):
    
    def define_options(self, params):
        params.add_string_list('config', help='Configuration directory')
        
        params.add_string('boot_root', default='out-boot_root',
                          help='Output directory for log files')

        params.add_flag('careful', help='Extra careful checking')

        params.add_string('jobs', help='List of set (defined in rs2b_convert_jobs.yaml)',
                          compulsory=True)
        
    def define_jobs(self):
        options = self.get_options()
        
        boot_root = os.path.realpath(options.boot_root)
        self.logger.info('Using root: %s' % boot_root)
        safe_makedirs(boot_root)
        
        config = get_rs2b_config()
        config.load(config.get_default_dir())
        if options.config is not None:
            for d in options.config:
                config.load(d)
                
        self.logger.info('Jobs: %s' % options.jobs)

        jobs = config.convert_jobs.expand_names(options.jobs)
         
        data_central = DataCentral(boot_root)
        
        
        for id_job in jobs:
            convert_job = config.convert_jobs.instance(id_job)
            assert isinstance(convert_job, ConvertJob)
            self.define_jobs_one(config, data_central, convert_job)

    def define_jobs_one(self, config, data_central, convert_job):
        options = self.get_options()
        check_valid_values = options.careful

        logs = config.explogs.expand_names(convert_job.logs)
        ds = data_central.get_dir_structure()
        for id_log in logs:
            id_robot = convert_job.id_robot
            id_stream = '%s%s' % (convert_job.id_episode_prefix, id_log)
            id_agent = None
            id_adapter = convert_job.adapter
            
            filename = ds.get_explog_filename(id_robot=id_robot,
                                                  id_agent=id_agent,
                                                  id_stream=id_stream)
            
            self.comp(do_convert_job,
                config=config,
                id_robot=id_robot,
                id_adapter=id_adapter,
                id_log=id_log,
                id_stream=id_stream,
                filename=filename,
                check_valid_values=check_valid_values)
    

class ConvertJob():
    
    @contract(id_robot='str', adapter='str', logs='list(str)', id_episode_prefix='str')
    def __init__(self, id_robot, adapter, logs, id_episode_prefix):
        self.id_robot = id_robot
        self.adapter = adapter
        self.logs = logs
        self.id_episode_prefix = id_episode_prefix


    
def do_convert_job(config, id_robot, id_adapter, id_log, id_stream, filename,
                   check_valid_values=True):
    set_rs2b_config(config)
    
    log = config.explogs.instance(id_log)
    
    id_environment = log.get_id_environment()
    id_episode = id_log
    
    adapter = config.adapters.instance(id_adapter)
    boot_spec = adapter.get_spec()
    
    topics = set(adapter.get_relevant_topics())
    
    # Topic to last message
    topic2last = {}
    
    logs_format = LogsFormat.get_reader_for(filename)

    keeper = ObsKeeper(boot_spec=boot_spec, id_robot=id_robot,
                       check_valid_values=check_valid_values)

    with logs_format.write_stream(filename=filename, id_stream=id_stream,
                                  boot_spec=boot_spec) as writer:
        
        for topic, msg, t, extra in log.read_all(topics=topics):
            if extra['counter'] % 10 == 0:
                print('percentage: %s' % extra['t_percentage'])
                
            assert topic in topics
            topic2last[topic] = msg
            
            # If we received all topics
            if len(topic2last) == len(topics):
                
                update = adapter.get_observations(topic2last, topic, msg, t)
                if update is None:
                    continue
                
                obs = update                           
                boot_observations = keeper.push(timestamp=obs.timestamp,
                                                observations=obs.observations,
                                                commands=obs.commands,
                                                commands_source=obs.commands_source,
                                                id_episode=id_episode,
                                                id_world=id_environment)

                writer.push_observations(observations=boot_observations, extra=extra)
            
    
    
def main_convert():
    
    sys.exit(RS2BConvert().main())
    
if __name__ == '__main__':
    main_convert()
    
    

 

