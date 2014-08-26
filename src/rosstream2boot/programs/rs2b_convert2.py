import os
import warnings

from contracts import contract, describe_type

from bootstrapping_olympics import (get_boot_config,
    PassiveRobotInterface)
from boot_manager import DataCentral, LogsFormat
from conf_tools import friendly_path
from quickapp import QuickApp
from rawlogs.configuration import get_conftools_rawlogs
from rosstream2boot import get_rs2b_config, logger
from rosstream2boot.library import ROSRobot

from .rs2b import RS2B


__all__ = ['RS2BConvert2']


class RS2BConvert2(RS2B.sub, QuickApp):  # @UndefinedVariable
    cmd = 'convert2'
    
    def define_options(self, params):
        params.add_string('boot_root')
        params.add_string('id_explog')
        params.add_string('id_robot', help='This should be a LoggedRobot')
        params.add_string('id_robot_res')

        params.add_string('id_episode_prefix', default='')
        
    def define_jobs_context(self, context):
        options = self.get_options()
        boot_root = options.boot_root
        rs2b_config = get_rs2b_config()
        boot_config = get_boot_config()
        data_central = DataCentral(boot_root)
        id_robot = options.id_robot
        id_robot_res = options.id_robot_res
        id_explog = options.id_explog
        id_stream = '%s%s' % (options.id_episode_prefix, id_explog)
        id_episode = id_stream
        id_agent = None
        
        # make sure we have them
        boot_config.robots[id_robot]
        rs2b_config.explogs[id_explog]
  
        ds = data_central.get_dir_structure()
        filename = ds.get_explog_filename(id_robot=id_robot_res,
                                          id_agent=id_agent,
                                          id_stream=id_stream)
      
        return context.comp_config(do_convert_job2,
                            id_robot=id_robot,
                            id_robot_res=id_robot_res,
                            id_explog=id_explog, id_stream=id_stream,
                            id_episode=id_episode,
                            filename=filename) 
     

def do_convert_job2(id_robot,
                    id_robot_res,
                    id_explog, id_stream, id_episode,
                    filename):
    """ This uses explogs """
    rs2b_config = get_rs2b_config()
    boot_config = get_boot_config()

    """
        id_robot: original robot must be a ROSRobot
    """

    if os.path.exists(filename):
        msg = 'Output file exists: %s\n' % friendly_path(filename)
        msg += 'Delete to force recreating the log.'
        logger.info(msg)
        return

    explog = rs2b_config.explogs.instance(id_explog)
    robot = boot_config.robots.instance(id_robot)
    orig_robot = robot.get_inner_components()[-1]

    if not isinstance(orig_robot, ROSRobot):
        msg = 'Expected ROSRobot, got %s' % describe_type(robot)
        raise ValueError(msg)
    
    orig_robot.read_from_log(explog)
    id_environment = explog.get_id_environment()
    write_robot_observations(id_stream, filename, id_robot_res,
                             robot, id_episode, id_environment)



def do_convert_job_rawlog(id_robot,
                    id_robot_res,
                    id_rawlog, id_stream, id_episode,
                    filename):

    

    """
        id_robot: original robot must be a ROSRobot
    """
    
    if os.path.exists(filename):
        msg = 'Output file exists: %s\n' % friendly_path(filename)
        msg += 'Delete to force recreating the log.'
        logger.info(msg)
        return
    
    rawlog = get_conftools_rawlogs().instance(id_rawlog)

    boot_config = get_boot_config()
    robot = boot_config.robots.instance(id_robot)
    orig_robot = robot.get_inner_components()[-1]

    if not isinstance(orig_robot, ROSRobot):
        msg = 'Expected ROSRobot, got %s' % describe_type(robot)
        raise ValueError(msg)
    
    orig_robot.read_from_rawlog(rawlog)
    id_environment = 'not-specified'  # rawlog.get_id_environment()
    write_robot_observations(id_stream, filename, id_robot_res,
                             robot, id_episode, id_environment)
    return 0
         
@contract(robot=PassiveRobotInterface, filename='str', id_robot='str')
def write_robot_observations(id_stream, filename, id_robot,
                             robot, id_episode, id_environment):
    logs_format = LogsFormat.get_reader_for(filename)

    boot_spec = robot.get_spec() 

    keeper = ObsKeeper(boot_spec=boot_spec, id_robot=id_robot,
                       check_valid_values=False)

    boot_spec = robot.get_spec() 

    nvalid = 0
    with logs_format.write_stream(filename=filename, id_stream=id_stream,
                                  boot_spec=boot_spec) as writer:
        for obs in iterate_robot_observations(robot, sleep=0):
            # print('robot_pose: %s' % obs.robot_pose)
            # print('got %s' % obs['timestamp'])
            boot_observations = keeper.push(timestamp=obs.timestamp,
                                            observations=obs.observations,
                                            commands=obs.commands,
                                            commands_source=obs.commands_source,
                                            id_episode=id_episode,
                                            id_world=id_environment)
    
            extra = {}
            warnings.warn('Make this more general')
            if obs.robot_pose is not None:
                extra['robot_pose'] = obs.robot_pose.tolist()
                extra['odom'] = obs.robot_pose.tolist()
            writer.push_observations(observations=boot_observations,
                                     extra=extra)
        
            nvalid += 1
            
    if nvalid == 0:
        msg = 'No observations could be found in %s' % filename
        raise Exception(msg)

       
        
        
        
