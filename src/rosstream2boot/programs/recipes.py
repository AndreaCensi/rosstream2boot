from quickapp import ResourceManager
from .rs2b_convert2 import RS2BConvert2

__all__ = ['recipe_episodeready_by_convert2']


def recipe_episodeready_by_convert2(context, boot_root, id_robot=None):
    """
        Provides episode-ready (id_robot, id_episode)
        
    """
    my_id_robot = id_robot
    
    def rp_convert(c, id_robot, id_episode):
        if my_id_robot is not None and my_id_robot != id_robot:
            msg = ('I only know how to create %r, not %r.' % 
                   (my_id_robot, id_robot))
            raise ResourceManager.CannotProvide(msg)
        id_explog = id_episode
        return c.subtask(RS2BConvert2,
                           boot_root=boot_root,
                           id_explog=id_explog,
                           id_robot=id_robot,
                           id_robot_res=id_robot,
                           add_job_prefix='')
    
    rm = context.get_resource_manager()        
    rm.set_resource_provider('episode-ready', rp_convert)
    
