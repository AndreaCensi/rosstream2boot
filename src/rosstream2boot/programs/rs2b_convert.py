from bootstrapping_olympics.interfaces.observations import ObsKeeper
from bootstrapping_olympics.logs.logs_format import LogsFormat
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from bootstrapping_olympics.utils import safe_makedirs
from conf_tools.utils.friendly_paths import friendly_path
from contracts import contract
from rosbag_utils import read_bag_stats_progress
from rosstream2boot import logger
from rosstream2boot.config import set_rs2b_config
from rosstream2boot.programs.rs2b import RS2BCmd, RS2Bsub
import os 


class ConvertJob():
    """
        
        id_explog: "log1"
        id_episode_prefix: 'nominal_'
        
        gives:
        
            id_stream = nominal_log1
            id_episode = nominal_log1
            
        
    """
    
    @contract(id_robot='str', adapter='str', logs='list(str)', id_episode_prefix='str')
    def __init__(self, id_robot, adapter, logs, id_episode_prefix):
        self.id_robot = id_robot
        self.adapter = adapter
        self.logs = logs
        self.id_episode_prefix = id_episode_prefix


@RS2Bsub
class RS2BConvertBatch(RS2BCmd):
    cmd = 'convert'
    
    def define_options(self, params):
        params.add_string('boot_root', default='out-boot_root',
                          help='Output directory for log files')

        params.add_flag('careful', help='Extra careful checking')

        params.add_string('jobs',
                          help='List of set (defined in rs2b_convert_jobs.yaml)',
                          compulsory=True)
        
    def define_jobs_context(self, context):
        options = self.get_options()
        
        boot_root = os.path.realpath(options.boot_root)
        self.logger.info('Using root: %s' % boot_root)
        safe_makedirs(boot_root)
        
        config = self.get_rs2b_config()
        self.logger.info('Jobs: %s' % options.jobs)

        jobs = config.convert_jobs.expand_names(options.jobs)
         
        data_central = DataCentral(boot_root)
        
        for id_job in jobs:
            convert_job = config.convert_jobs.instance(id_job)
            assert isinstance(convert_job, ConvertJob)
            self.define_jobs_one(context, config, data_central, convert_job)
            
    def define_jobs_one(self, context, config, data_central, convert_job):
        options = self.get_options()
        check_valid_values = options.careful

        logs = config.explogs.expand_names(convert_job.logs)
        for id_log in logs:
            id_robot = convert_job.id_robot
            id_stream = '%s%s' % (convert_job.id_episode_prefix, id_log)
            id_episode = id_stream
            id_agent = None
            id_adapter = convert_job.adapter
            
            context.comp(do_convert_job,
                data_central=data_central,
                config=config,
                id_robot=id_robot,
                id_adapter=id_adapter,
                id_log=id_log,
                id_stream=id_stream,
                id_episode=id_episode,
                id_agent=id_agent,
                check_valid_values=check_valid_values)




@RS2Bsub
class RS2BConvertOne(RS2BCmd):
    cmd = 'convert-one'
    # Assumes parent.get_rs2b_config()
    
    # TODO: episode_prefix: does it really matter?
    
    def define_options(self, params):
        params.add_string('boot_root', compulsory=True)
        params.add_string('id_explog', compulsory=True)
        params.add_string('id_episode_prefix', default='')
        params.add_string('id_adapter', compulsory=True)
        params.add_string('id_robot', compulsory=True,
                          help='Resulting id_robot (arbitrary)')
        
    def define_jobs_context(self, context):
        options = self.get_options()
        boot_root = options.boot_root
        config = self.get_rs2b_config()
        data_central = DataCentral(boot_root)
        id_robot = options.id_robot
        id_explog = options.id_explog
        id_adapter = options.id_adapter
    
        id_stream = '%s%s' % (options.id_episode_prefix, id_explog)
        id_episode = id_stream
        id_agent = None
        
        return context.comp(do_convert_job,
                data_central=data_central,
                config=config,
                id_robot=id_robot,
                id_adapter=id_adapter,
                id_log=id_explog,
                id_stream=id_stream,
                id_episode=id_episode,
                id_agent=id_agent,
                check_valid_values=False) 
     

def do_convert_job(data_central, config, id_robot, id_adapter, id_log, id_stream, id_episode, id_agent,
                   check_valid_values=True):
    """
        id_episode => 
    """
    ds = data_central.get_dir_structure()
    filename = ds.get_explog_filename(id_robot=id_robot,
                                      id_agent=id_agent,
                                      id_stream=id_stream)
    
    if os.path.exists(filename):
        msg = 'Output file exists: %s\n' % friendly_path(filename)
        msg += 'Delete to force recreating the log.'
        logger.info(msg)
        return

    set_rs2b_config(config)
    
    log = config.explogs.instance(id_log)
    
    id_environment = log.get_id_environment()
    
    adapter = config.adapters.instance(id_adapter)
    boot_spec = adapter.get_spec()

    
    topics = [name for name, _ in adapter.get_relevant_topics()]  
    
    # Topic to last message
    topic2last = {}
    
    logs_format = LogsFormat.get_reader_for(filename)

    keeper = ObsKeeper(boot_spec=boot_spec, id_robot=id_robot,
                       check_valid_values=check_valid_values)

    nvalid = 0
    with logs_format.write_stream(filename=filename, id_stream=id_stream,
                                  boot_spec=boot_spec) as writer:
        
        source = log.read_all(topics=topics)
        show_progress = read_bag_stats_progress(source, logger, interval=5)
        
        for topic, msg, t, ros_extra in show_progress:  # @UnusedVariable
            assert topic in topics
            # print t, topic
            topic2last[topic] = msg
            
            # until we received all topics
            if len(topic2last) != len(topics):
                # print('Not all topicss received (%s instead of %s)'
                #      % (topic2last.keys(), topics))
                continue
                
            update = adapter.get_observations(topic2last, topic, msg, t)
            if update is None:
                # print('skipping')
                continue
                        
            obs = update                           
            boot_observations = keeper.push(timestamp=obs.timestamp,
                                            observations=obs.observations,
                                            commands=obs.commands,
                                            commands_source=obs.commands_source,
                                            id_episode=id_episode,
                                            id_world=id_environment)

            extra = {}
            if obs.robot_pose is not None:
                extra['odom'] = obs.robot_pose.tolist()
            writer.push_observations(observations=boot_observations,
                                     extra=extra)
            
            nvalid += 1
    if nvalid == 0:
        msg = 'No observations could be found in %s' % filename
    return id_episode
