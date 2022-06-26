import os
import shutil
from building import *

cwd = GetCurrentDir()
objs = []
list = os.listdir(cwd)

for d in list:
    path = os.path.join(cwd, d)
    if os.path.isfile(os.path.join(path, 'SConscript')):
        objs = objs + SConscript(os.path.join(d, 'SConscript'))

#delate non-used files
try:
    shutil.rmtree(os.path.join(cwd,'legacy'))
except:
    pass

Return('objs')
