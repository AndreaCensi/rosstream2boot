from bootstrapping_olympics.configuration.master import set_boot_config
from bootstrapping_olympics.interfaces.observations import ObsKeeper
from bootstrapping_olympics.interfaces.robot import (RobotObservations,
    PassiveRobotInterface)
from bootstrapping_olympics.library.robots.equiv_robot import EquivRobot
from bootstrapping_olympics.logs.logs_format import LogsFormat
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from conf_tools.utils.friendly_paths import friendly_path
from contracts import contract
from contracts.interface import describe_type
from rosstream2boot import logger
from rosstream2boot.config import set_rs2b_config
from rosstream2boot.library.robot_from_bag import ROSRobot
from rosstream2boot.programs.rs2b import RS2BCmd, RS2Bsub
import os
import time
import warnings


@RS2Bsub
class RS2BConvert2(RS2BCmd):
    cmd = 'convert2'
    # Assumes parent.get_rs2b_config()
    # Assumes parent.get_boot_config()
    
    def define_options(self, params):
        params.add_required_string('boot_root')
        params.add_required_string('id_explog')
        params.add_required_string('id_robot', help='This should be a LoggedRobot')
        params.add_required_string('id_robot_res')

        params.add_string('id_episode_prefix', default='')
        
    def define_jobs_context(self, context):
        options = self.get_options()
        boot_root = options.boot_root
        rs2b_config = self.get_rs2b_config()
        boot_config = self.get_boot_config()
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
      
        return context.comp(do_convert_job2,
                            rs2b_config=rs2b_config, boot_config=boot_config,
                            id_robot=id_robot,
                            id_robot_res=id_robot_res,
                            id_explog=id_explog, id_stream=id_stream,
                            id_episode=id_episode,
                            filename=filename) 
     

def do_convert_job2(rs2b_config, boot_config,
                    id_robot,
                    id_robot_res,
                    id_explog, id_stream, id_episode,
                    filename):
    set_boot_config(boot_config)
    set_rs2b_config(rs2b_config)

    """
        id_robot: original robot must be a RobotFromBag
    """
    
    if os.path.exists(filename):
        msg = 'Output file exists: %s\n' % friendly_path(filename)
        msg += 'Delete to force recreating the log.'
        logger.info(msg)
        return
    
    explog = rs2b_config.explogs.instance(id_explog)
    robot = boot_config.robots.instance(id_robot)
        
    warnings.warn('Make this more general')
    if isinstance(robot, EquivRobot):
        orig_robot = robot.get_original_robot()
    else:
        orig_robot = robot
        
    if not isinstance(orig_robot, ROSRobot):
        msg = 'Expected ROSRobot, got %s' % describe_type(robot)
        raise ValueError(msg)
    
    orig_robot.read_from_log(explog)
    id_environment = explog.get_id_environment()
    write_robot_observations(id_stream, filename, id_robot_res, robot, id_episode, id_environment)
    
    
def bd_sequence_from_robot(id_robot, robot, sleep_wait, id_episode, id_environment,
                           check_valid_values=False):
    """
    
        :param robobs_seq: Sequence of RobotObservations
        
        :returns: iterator of bd array
    """
    
    boot_spec = robot.get_spec() 

    keeper = ObsKeeper(boot_spec=boot_spec, id_robot=id_robot,
                       check_valid_values=check_valid_values)

    for obs in iterate_robot_observations(robot, sleep_wait):
        bd = keeper.push(timestamp=obs.timestamp,
                                        observations=obs.observations,
                                        commands=obs.commands,
                                        commands_source=obs.commands_source,
                                        id_episode=id_episode,
                                        id_world=id_environment)
        yield bd
         
@contract(robot=PassiveRobotInterface, filename='str', id_robot='str')
def write_robot_observations(id_stream, filename, id_robot, robot, id_episode, id_environment):
    logs_format = LogsFormat.get_reader_for(filename)

    boot_spec = robot.get_spec() 

    keeper = ObsKeeper(boot_spec=boot_spec, id_robot=id_robot,
                       check_valid_values=False)

    boot_spec = robot.get_spec() 

    nvalid = 0
    with logs_format.write_stream(filename=filename, id_stream=id_stream,
                                  boot_spec=boot_spec) as writer:
        for obs in iterate_robot_observations(robot):
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

def iterate_robot_observations(robot, sleep=0.1):
    while True:
        try:
            yield robot.get_observations()
        except RobotObservations.NotReady:
            time.sleep(sleep)
            continue
        except RobotObservations.Finished:
            break
        
       
        
        
        
