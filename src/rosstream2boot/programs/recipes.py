from quickapp import ResourceManager
from rawlogs import get_conftools_rawlogs
from rosstream2boot import get_conftools_explogs
from rosstream2boot.programs.rs2b_convert2 import do_convert_job_rawlog

from .rs2b_convert2 import RS2BConvert2


__all__ = ['recipe_episodeready_by_convert2']


def recipe_episodeready_by_convert2(context, boot_root, id_robot=None):
    """
        Provides episode-ready (id_robot, id_episode)
        
    """
    my_id_robot = id_robot
    
    def rp_convert_explogs(c, id_robot, id_episode):
        if my_id_robot is not None and my_id_robot != id_robot:
            msg = ('I only know how to create %r, not %r.' % 
                   (my_id_robot, id_robot))
            raise ResourceManager.CannotProvide(msg)
        id_explog = id_episode
        
        library = get_conftools_explogs()
        if not id_explog in library:
            msg = 'Log %r not found.' % id_explog
            raise ResourceManager.CannotProvide(msg)
        
        return c.subtask(RS2BConvert2,
                           boot_root=boot_root,
                           id_explog=id_explog,
                           id_robot=id_robot,
                           id_robot_res=id_robot,
                           add_job_prefix='')
    
    rm = context.get_resource_manager()        
    rm.set_resource_provider('episode-ready', rp_convert_explogs)
    

def recipe_episodeready_by_convert_rawlog(context, data_central, id_robot=None):
    """
        Provides episode-ready (id_robot, id_episode)
        
    """
    my_id_robot = id_robot

    def rp_convert_rawlogs(c, id_robot, id_episode):
        if my_id_robot is not None and my_id_robot != id_robot:
            msg = ('I only know how to create %r, not %r.' % 
                   (my_id_robot, id_robot))
            raise ResourceManager.CannotProvide(msg)

        id_rawlog = id_episode
        
        # NOTE here
        library = get_conftools_rawlogs()
        if not id_rawlog in library:
            msg = 'RAwLog %r not found.' % id_rawlog
            raise ResourceManager.CannotProvide(msg)

        id_robot_res = id_robot
        id_stream = id_episode
        ds = data_central.get_dir_structure()
        filename = ds.get_explog_filename(id_robot=id_robot_res,
                                          id_agent=None,
                                          id_stream=id_stream)

        res = c.comp_config(do_convert_job_rawlog,
                            id_robot=id_robot,
                            id_robot_res=id_robot,
                            id_rawlog=id_rawlog,
                            id_stream=id_stream,
                            id_episode=id_episode,
                            filename=filename)
        return res
    
    rm = context.get_resource_manager()        
    rm.set_resource_provider('episode-ready', rp_convert_rawlogs)
    
