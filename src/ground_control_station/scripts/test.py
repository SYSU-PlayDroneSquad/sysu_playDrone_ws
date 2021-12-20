from agent import Agents
from environment import Env
import numpy as np


def input_data(p_locs, e_loc):
    #输入p-locs: 无人机的位置，e_loc: 目标位置
    env1 = Env(256, 256)
    env1.input(p_locs, e_loc)
    v = env1.run()
    return v

p = [
    [18.5, 42],
    [21.5, 42],
    [18.5, 37.5],
    [21.5, 37.5],
    [18.5, 33],
    [21.5, 33],
    [18.5, 28.5],
    [21.5, 28.5]
]

e = [20, 50]

vs = []

for j in range(300):
    v, p, e = input_data(p, e, j)
    vs.append(v)






