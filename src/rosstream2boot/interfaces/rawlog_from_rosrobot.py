from blocks import check_timed_named, series
from boot_manager.meat.m_run_simulation import get_all_output_available
from bootstrapping_olympics import get_conftools_robots
from bootstrapping_olympics.library.agents.nuisance_agent_actions import (
    wrap_agent_learner_converter)
from bootstrapping_olympics.library.robots.nuisance_robot import NuisanceRobot
from collections import defaultdict
from contracts import check_isinstance, contract, describe_type
from decent_logs import WithInternalLog
from rawlogs import RawLog
from rosbag_utils.rosbag_flexible_read import resolve_topics
import warnings

__all__ = [
    'make_log_from_adapter',
]

@contract(returns=RawLog, id_ros_robot=str, rawlog=RawLog)
def make_log_from_adapter(id_ros_robot, rawlog):
    """ 
        Creates a RawLog returning 'observations', 'commands'
        from a ROS robot adapter. 
    """
    res = RawLogFromROSRobot(rawlog=rawlog, robot=id_ros_robot)
    return res
 
class RawLogFromROSRobot(RawLog, WithInternalLog):
    
    @contract(rawlog=RawLog, robot=str)
    def __init__(self, rawlog, robot):
        self.rawlog = rawlog
        self.robot = robot 
        self.orig_robot = None

    def get_signals(self):
        """ Returns the signals available """   
        signals0 = self.rawlog.get_signals()
        one = list(signals0)[0]
        self.info('using signal %r for reference' % one)
        bounds = signals0[one].get_time_bounds()
        ref = signals0[one].get_time_reference()
        resources = self.rawlog.get_resources()
        
        class MySignal(object):
            def get_signal_type(self):
                return 'x-object'
            def get_time_reference(self):
                return ref
            def get_resources(self):
                return resources
            def get_time_bounds(self):
                return bounds

        signals = {}
        signals['observations'] = MySignal()
        signals['commands'] = MySignal()
        return signals

    @contract(returns='list(str)')
    def get_resources(self):
        return self.rawlog.get_resources() 
        
    def _instance(self):
        robot_config = get_conftools_robots()
        _,self.robot0 = robot_config.instance_smarter(self.robot)
        self.orig_robot = self.robot0.get_inner_components()[-1]
        from rosstream2boot.library.ros_robot import ROSRobot
        check_isinstance(self.orig_robot, ROSRobot)

    @contract(topics='list(str)', 
              start='None|(float,t)', 
              stop='None|(float,>t)')
    def read(self, topics, start=None, stop=None):
        if self.orig_robot is None:
            self._instance()
            
        from rosstream2boot.library.ros_robot import ROSRobot
        if not isinstance(self.robot0, ROSRobot):
            
            if not isinstance(self.robot0, NuisanceRobot):
                msg = 'not considered %r' % describe_type(self.robot0)
                raise NotImplementedError(msg)


            
        providing = ['observations', 'commands']
        for t in topics:
            if not t in providing:
                msg = 'Not providing %r.' % t
                raise ValueError(msg)
    
        known = list(self.rawlog.get_signals())

        adapter = self.orig_robot.adapter
        all_topics = adapter.get_relevant_topics() + adapter.get_published_topics()
        asked = [name for name, _ in all_topics]          
        resolved, resolved2asked, asked2resolved = resolve_topics(known, asked)
        self.info('resolved: %s' % resolved)
        translator = adapter.get_translator()
          
        source = self.rawlog.read(topics=resolved, start=start, stop=stop)
        
        # Here 
        
        # |log| -> messages -> |translator| -> y,u -> |nuisance| -> y',u'
        if isinstance(self.robot0, NuisanceRobot):
            nuisance = self.robot0.nuisance
            warnings.warn('FIXME: not sure if correct')
            nuisance_box = wrap_agent_learner_converter(nuisance)
            translator = series(translator, nuisance_box)
            
        translator.reset()
        read_signals = defaultdict(lambda: 0)
        translated = defaultdict(lambda: 0) 
        for x in source:
            check_timed_named(x)
            (_, (s, _)) = x
            read_signals[s] += 1
            translator.put(x)
            res, finished = get_all_output_available(translator)  # @UnusedVariable
            for r in res:
                check_timed_named(r)
                (_, (s, _)) = r
                translated[s] += 1
                
#                 if s not in providing:
#                     self.info('Skipping signal %r' % s)
#                 else:
                yield r
                
        # statistics
        read_signals = dict(**read_signals)
        translated = dict(**translated) 
        self.info('      read: %s' % read_signals)        
        self.info('translated: %s' % translated)
        
        if len(translated) == 0:
            msg = 'Something is wrong: no msgs translated.'
            raise Exception(msg)
         

