#!/usr/bin/env python
# coding=utf-8

import numpy as np
import math
import random


class Agent(object):

    def __init__(self, id, loc, group, a_range):

        self.id = id
        self.loc = loc
        self.group = group
        self.vmax = 0.8  # pursuer 速度
        self.a_range = a_range
        self._in_range = 0
        self.state = 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
        self.time_step = 1
        self._relative_angle = 0
        self.gap = 0
        self.dis = 0
        self.v = np.array([0, self.vmax * self.time_step])
        self.too_close = []
        self.repel_v = [0, 0]
        self.left = []
        self.right = []

    def angle(self, a, b):
        v = np.array(b - a)

        x = 1
        y = np.linalg.norm(v)
        z = np.linalg.norm(np.array(v - [1, 0]))

        costheta = (x ** 2 + y ** 2 - z ** 2) / (2 * x * y)

        if v[1] >= 0:
            theta = np.arccos(costheta)
        else:
            theta = 2 * np.pi - np.arccos(costheta)

        return theta

    def whether_in_range(self, e_loc):
        self.dis = np.linalg.norm(np.array(self.loc - e_loc))
        if self.a_range - 0.5 <= self.dis <= self.a_range + 0.5:
            self._in_range = 1
            self.state = 1
        else:
            self._in_range = 0

    def get_relative_angle(self, e_loc):
        self._relative_angle = self.angle(e_loc, self.loc)

    def get_too_close(self, agents):
        for a in agents:
            if a.id != self.id:
                dis = np.linalg.norm(self.loc - a.loc)
                if dis <= 1.5:
                    self.too_close.append(a)

    def get_v(self, e_loc, e_v):
        if self.state == 0:
            self.v = [0, self.vmax]
        if self.state == 1:
            change = self.vmax / self.a_range
            if change > np.abs(self.gap):
                angle1 = self._relative_angle - self.gap / 2
                new_loc = e_loc + self.dis * np.array([np.cos(angle1), np.sin(angle1)])
                self.v = new_loc - self.loc + e_v
            else:
                if self.gap < 0:
                    angle1 = self._relative_angle + change
                else:
                    angle1 = self._relative_angle - change
                new_loc = e_loc + self.dis * np.array([np.cos(angle1), np.sin(angle1)])
                self.v = (new_loc - self.loc ) * 1.2

    def move(self):
        #self.loc = self.loc + self.v
        norm_repel = np.linalg.norm(self.repel_v)
        if norm_repel != 0:
            self.v = self.v + self.repel_v
            vel = np.linalg.norm(self.v)
            self.v = self.v / vel * 0.2
            self.loc = self.loc + self.v
        else:
            self.loc = self.loc + self.v


    def repel(self):
        repel = [0, 0]

        if len(self.too_close) != 0:
            for a in self.too_close:
                dis = np.linalg.norm(self.loc - a.loc)
                dix = self.loc - a.loc
                dis2 = 1.5 - dis
                re = dix / dis2
                repel = repel + re

        self.repel_v = repel

    def center(self):
        l_loc = self.left[0].loc
        r_loc = self.right[0].loc

        l_mid = 0.5 * (self.loc + l_loc)
        r_mid = 0.5 * (self.loc + r_loc)

        l_line = 17

    def get_center(self, p1, p2, p3):
        x21 = p2[0] - p1[0]
        y21 = p2[1] - p1[1]
        x32 = p3[0] - p2[0]
        y32 = p3[1] - p2[1]

        if x21 * y32 - x32 * y21 == 0:
            return None
        xy21 = p2[0] ** 2 - p1[0] ** 2 + p2[1] ** 2 - p1[1] ** 2
        xy32 = p3[0] ** 2 - p2[0] ** 2 + p3[1] ** 2 - p2[1] ** 2

        y0 = 0.5 * (x32 * xy21 - x21 * xy32) * (y21 * x32 - y32 * x21)
        x0 = 0.5 * (xy21 - 2 * y0 * y21) / x21
        r = ((p1[0] - x0) ** 2 + (p1[1] - y0) ** 2) ** 0.5

        return x0, y0, r



class Agents(object):

    def __init__(self, n):

        self.n = n
        self.agents = []
        self.count = 0
        self.split = 2 * np.pi
        self.inrange1 = []
        self.inrange2 = []
        self.inrange3 = []

    def init(self):
        i = 0
        start = [18.5, 42]

        start2 = [15.5, 42]
        start3 = [15.5, 24]

        start4 = [9.5, 42]

        for j in range(4):
            loc1 = np.array([start[0], start[1] - j * 4.5])
            loc2 = np.array([start[0] + 3, start[1] - j * 4.5])
            a1 = Agent(i, loc1, 1, 8)
            a2 = Agent(i + 1, loc2, 1, 8)
            self.agents.append(a1)
            self.agents.append(a2)
            i = i + 2

        for k in range(4):
            loc1 = np.array([start2[0], start2[1] - k * 4.5])
            loc2 = np.array([start2[0] + 9, start2[1] - k * 4.5])
            a1 = Agent(i, loc1, 2, 16)
            a2 = Agent(i + 1, loc2, 2, 16)
            self.agents.append(a1)
            self.agents.append(a2)
            i = i + 2

        for l in range(2):
            loc1 = np.array([start3[0], start3[1] - l * 4.5])
            loc2 = np.array([start3[0] + 3, start3[1] - l * 4.5])
            loc3 = np.array([start3[0] + 6, start3[1] - l * 4.5])
            loc4 = np.array([start3[0] + 9, start3[1] - l * 4.5])
            a1 = Agent(i, loc1, 2, 16)
            a2 = Agent(i + 1, loc2, 2, 16)
            a3 = Agent(i + 2, loc3, 2, 16)
            a4 = Agent(i + 3, loc4, 2, 16)
            self.agents.append(a1)
            self.agents.append(a2)
            self.agents.append(a3)
            self.agents.append(a4)
            i = i + 4

        for m in range(6):
            loc1 = np.array([start4[0], start4[1] - m * 4.5])
            loc2 = np.array([start4[0] + 3, start4[1] - m * 4.5])
            loc3 = np.array([start4[0] + 18, start4[1] - m * 4.5])
            loc4 = np.array([start4[0] + 21, start4[1] - m * 4.5])
            a1 = Agent(i, loc1, 3, 24)
            a2 = Agent(i + 1, loc2, 3, 24)
            a3 = Agent(i + 2, loc3, 3, 24)
            a4 = Agent(i + 3, loc4, 3, 24)
            self.agents.append(a1)
            self.agents.append(a2)
            self.agents.append(a3)
            self.agents.append(a4)
            i = i + 4

    def whether_in_range(self, e_loc):
        inlist1 = []
        inlist2 = []
        inlist3 = []
        for a in self.agents:
            a.whether_in_range(e_loc)
        for a in self.agents:
            if a._in_range == 1:
                if a.group == 1:
                    inlist1.append(np.array([a.id, a._relative_angle]))
                if a.group == 2:
                    inlist2.append(np.array([a.id, a._relative_angle]))
                if a.group == 3:
                    inlist3.append(np.array([a.id, a._relative_angle]))

        self.inrange1 = inlist1
        self.inrange2 = inlist2
        self.inrange3 = inlist3

        if len(self.inrange1) != 0:
            self.inrange1.sort(key=lambda agent: agent[1])
        if len(self.inrange2) != 0:
            self.inrange2.sort(key=lambda agent: agent[1])
        if len(self.inrange3) != 0:
            self.inrange3.sort(key=lambda agent: agent[1])

    def _in_range_count(self):
        count = 0
        for a in self.agents:
            if a._in_range == 1:
                count = count + 1
        self.split = 2 * np.pi / count

    def get_relative_angle(self, e_loc):
        for a in self.agents:
            a.get_relative_angle(e_loc)

    def get_gap(self, inrange):
        length = len(inrange)

        if length == 2:
            gap1 = inrange[0][1] - inrange[1][1]
            self.agents[int(inrange[0][0])].gap = -gap1
            self.agents[int(inrange[1][0])].gap = gap1
        else:
            lefts = []
            rights = []
            rights.append(2 * np.pi - inrange[-1][1] + inrange[0][1])
            for i in range(length - 1):
                left = inrange[i + 1][1] - inrange[i][1]
                lefts.append(left)
                rights.append(left)
            lefts.append(2 * np.pi - inrange[-1][1] + inrange[0][1])
            for j in range(length):
                gap = rights[j] - lefts[j]
                self.agents[int(inrange[j][0])].gap = gap

        if length == 1:
            self.agents[int(inrange[0][0])].gap = 2 * np.pi

    def get_all_gap(self):
        if len(self.inrange1) != 0:
            self.get_gap(self.inrange1)
        if len(self.inrange2) != 0:
            self.get_gap(self.inrange2)
        if len(self.inrange3) != 0:
            self.get_gap(self.inrange3)

    def move(self):
        for a in self.agents:
            a.move()

    def get_v(self, e_loc, e_v):
        for a in self.agents:
            a.get_v(e_loc, e_v)

    def get_too_close(self):
        for a in self.agents:
            a.get_too_close(self.agents)

    def get_repel_v(self):
        for a in self.agents:
            a.repel()

