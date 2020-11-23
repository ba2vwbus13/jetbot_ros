import numpy as np

def _dancing():
        dancing_pattern = np.concatenate([np.repeat('forward', 10), np.repeat('stop', 2), 
        np.repeat('backward', 10), np.repeat('stop', 2), 
        np.repeat('right', 10), np.repeat('stop', 2), 
        np.repeat('left', 10)])
        for com in dancing_pattern:
            print(com)

_dancing()