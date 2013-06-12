from .rs2b import RS2B
from rosstream2boot import get_conftools_explogs
import os
import sys

__all__ = ['CheckFiles']

# class CheckLogs(RS2B.get_sub()):
#     """ 
#         Checks that we have a description for the files specified 
#         on the command line. 
#     """
#     
#     cmd = 'check-logs'
#     
#     def define_program_options(self, params):
#         params.accept_extra()
#         
#     def go(self):
#         files = self.options.get_extra()
#         names = [os.path.splitext(os.path.basename(f))[0] for f in files
#                  if '.bag' in f]
#     
#         explogs = get_conftools_explogs()
# 
#         for name in names:
#             if not name in explogs:
#                 self.error('not found: %r' % name)
        
class CheckFiles(RS2B.get_sub()):
    """ 
        Checks that all the files in the command line are claimed by
        some explogs and vice versa.
    """
    
    cmd = 'check-files'
    
    def define_program_options(self, params):
        params.accept_extra()
        
    def go(self):
        files = self.options.get_extra()
        
        from collections import defaultdict
        seen = defaultdict(list)
        for f in files:
            seen[os.path.basename(f)].append(f)
            
        for _, names in seen.items():
            if len(names) > 1:
                self.error('Same file in multiple places:\n' + 
                           '\n'.join('- %s' % f for f in names))
        
        def consider(f):
            for ext in ['.mp4.timestamps', '.orig.bag', '.yaml',
                        '.index_cache']:
                if ext in f:
                    return False
            return True
        files = filter(consider, files)
        known_files = {}
        
        # for all explogs that we know
        explogs_library = get_conftools_explogs()
        explogs_library.make_sure_everything_read()
        
        explogs_library.print_summary(sys.stdout)
        for id_explog in explogs_library:
            explog = explogs_library.instance(id_explog)
            for what, filename in explog.get_files().items():  # @UnusedVariable
                known_files[filename] = id_explog
                if not os.path.exists(filename):
                    self.error('%s: File not found  %s: %s' % 
                               (id_explog, what, filename))
                if not id_explog in filename:
                    self.error('%s: strange filename %s' % 
                               (id_explog, filename))
        for f in sorted(files):
            if not f in known_files:
                self.error('Unclaimed file: %s' % f)
            else:
                # self.info('%s has %s' % (known_files[f], f))
                pass
                







