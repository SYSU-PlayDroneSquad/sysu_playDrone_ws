#!/usr/bin/env python
# coding=utf-8


from agent import Agents
from agent import Agent
from evader import Evaders
from evader import Evader
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D

class Env(object):

    def __init__(self, psize, esize):

        self.psize = psize
        self.esize = esize
        self.evaders = Evaders(esize)
        self.agents = Agents(psize)
        self.cap_r = 0.8

    def init(self):
        self.evaders.init()
        self.agents.init()

    def distance(self, a, b):
        dis = np.linalg.norm(np.array(a - b))
        return dis


    def angle(self, a, o, b):
        ao = np.linalg.norm(np.array(a) - np.array(o))
        bo = np.linalg.norm(np.array(b) - np.array(o))
        ab = np.linalg.norm(np.array(a) - np.array(b))
        cos_theta = (ao ** 2 + bo ** 2 - ab ** 2)/(2 * ao * bo)
        theta = np.arccos(cos_theta)

        return theta


    def plot(self, i):
        fig, ax = plt.subplots()
        ax.cla()
        point = []
        for a in self.agents.agents:
            att_range = patches.Circle(
                a.loc,
                radius=self.cap_r,
                fc='white',
                ec='green',
                ls='-',
                lw=2,
                alpha=0.2)
            ax.add_patch(att_range)
            x = a.loc[0]
            y = a.loc[1]
            if a.state == 0:
                if a.group == 1:
                    ax.scatter(x, y, color='blue', edgecolors='white', marker='o')
                if a.group == 2:
                    ax.scatter(x, y, color='green', edgecolors='white', marker='o')
                if a.group == 3:
                    ax.scatter(x, y, color='orange', edgecolors='white', marker='o')
            if a.state == 1:
                if a.group == 1:
                    ax.scatter(x, y, color='cyan', edgecolors='white', marker='o')
                if a.group == 2:
                    ax.scatter(x, y, color='purple', edgecolors='white', marker='o')
                if a.group == 3:
                    ax.scatter(x, y, color='yellow', edgecolors='white', marker='o')

            point.append(np.array([x, y]))

        for b in self.evaders.evaders:
            ax.plot(b.loc[0], b.loc[1], 'rv')
            #att_range = patches.Circle(
                #b.loc,
                #radius=b.v_range,
                #fc='white',
                #ec='b',
                #ls='-',
                #lw=2,
                #alpha=0.6)
            #ax.add_patch(att_range)
            # ax.plot(rob.loc[0], rob.loc[1], 'ro')
        # for eva in self.evaders.evaders:
        # x = eva.loc[0]
        # y = eva.loc[1]
        # ax.plot(x, y, 'bo')
        # x2 = np.array(eva.rought_x)
        # y2 = np.array(eva.rought_y)
        # plt.plot(x2, y2, '-', 'b')
        #ax.set_title('Pursuers: ' + str(self.psize) + '  ' + 'Evaders: ' + str(self.esize) + '  '+ 'Captured: ' + str(self.esize - len(self.evaders.evaders))
        #                    )
        ax.set_xlim(-15, 55)
        ax.set_ylim(0, 130)
        ax.set_aspect(1)
        plt.savefig('topology' + str(i) +  '.png')

        plt.pause(0.01)

    def run(self):
        #self.plot(i)

        self.agents.get_relative_angle(self.evaders.evaders[0].loc)
        self.agents.whether_in_range(self.evaders.evaders[0].loc)

        self.agents.get_all_gap()
        self.agents.get_v(self.evaders.evaders[0].loc, self.evaders.evaders[0].v)

        v = np.zeros((2, 8), dtype='f8')


        self.agents.move()
        self.evaders.move()

        i = 0
        for a in self.agents.agents:
            v[0][int(a.id)] = a.v[0]
            v[1][int(a.id)] = a.v[1]




        return v


    def input(self, locs, e_loc):

        i = 0
        for p in locs:
            l = np.array([p[0], p[1]])
            a = Agent(i, l, 1, 8)
            self.agents.agents.append(a)
            i = i + 1

        e = Evader(1, e_loc)
        self.evaders.evaders.append(e)



