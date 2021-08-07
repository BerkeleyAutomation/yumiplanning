import numpy as np
from yumirws.yumi import YuMi
y=YuMi()
js=(y.left.get_joints())
j=[xd for xd in js]
print(j)
