from math import inf
import psutil

max_usage = inf
_process = psutil.Process()

def get_usage() -> 'float':
    ''' Returns memory usage of current process in MB. '''
    global _process
    return _process.memory_info().rss / (1024*1024)
