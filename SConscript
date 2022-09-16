import os
from building import *

cwd = GetCurrentDir()
objs = []
list = os.listdir(cwd)

if (GetDepend(['PKG_USING_ESP_IDF'])):
    Return('objs')

for d in list:
    path = os.path.join(cwd, d)
    if os.path.isfile(os.path.join(path, 'SConscript')):
        objs = objs + SConscript(os.path.join(d, 'SConscript'))

Return('objs')
