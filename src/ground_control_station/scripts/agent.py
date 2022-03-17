#!/usr/bin/env python
# coding=utf-8

import numpy as np
import math
import random


class Agent(object):

    def __init__(self, id, loc, group, a_range, state):

        self.id = id
        self.loc = loc
        self.group = group
        self.vmax = 1
        self.vmax2 = 2# pursuer 速度
        self.a_range = a_range
        self._in_range = 0
        self.state = state
        self.time_step = 1
        self._relative_angle = 0
        self.gap = 0
        self.dis = 0
        self.v = np.array([0, self.vmax * self.time_step])
        self.too_close = False
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
        if self.state == 1:
            self.state = 1
        else:
            if self.state == 2:
                if self.a_range - 0.5 <= self.dis <= self.a_range + 0.5:
                    self.state = 1
            else:
                if self.state == 0:
	                if self.a_range + 1.5 <= self.dis <= self.a_range + 6.5:
                         self.state = 2


    def get_relative_angle(self, e_loc):
        self._relative_angle = self.angle(e_loc, self.loc)

    def get_too_close(self, agents):
        for a in agents:
            if a.id != self.id:
                dis = np.linalg.norm(self.loc - a.loc)
                if dis <= 2:
                    self.too_close = True

    def get_v(self, e_loc, e_v):
        if self.state == 0:
            self.v = [0, self.vmax]
        if self.state == 2:
            if self.too_close:
                self.v = [0, self.vmax * 0.5]
            else:
                self.v = [0, self.vmax]
        if self.state == 1:
            change = self.vmax / self.dis
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
                self.v = (new_loc - self.loc) * 1.5

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
        self.sequence = []

    def whether_in_range(self, e_loc):
        inlist1 = []
        inlist2 = []
        inlist3 = []
        for a in self.agents:
            a.whether_in_range(e_loc)
        for a in self.agents:
            if a.state == 1 or a.state == 2:
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

        sqc = []
        for a in self.inrange1:
            sqc.append(np.int(a[0]))
        for a in self.inrange2:
            sqc.append(np.int(a[0]))
        for a in self.inrange3:
            sqc.append(np.int(a[0]))

        self.sequence = sqc

    def whether_complete(self):
        gaps1 = []
        gaps2 = []
        gaps3 = []

        for a in self.agents:
            if a.group == 1:
                gaps1.append(a.gap)
            if a.group == 2:
                gaps2.append(a.gap)
            if a.group == 3:
                gaps3.append(a.gap)

        gap_error1 = np.linalg.norm(gaps1)
        gap_error2 = np.linalg.norm(gaps2)
        gap_error3 = np.linalg.norm(gaps3)

        if gap_error1 <= 0.40 and len(self.inrange1) == 8:
            group1 = True
        else:
            group1 = False

        if gap_error2 <= 0.20 and len(self.inrange2) == 12:
            group2 = True
        else:
            group2 = False

        if gap_error2 <= 0.20 and len(self.inrange3) == 16:
            group3 = True
        else:
            group3 = False
        

        errors = np.array([gap_error1, len(self.inrange1), group1, gap_error2, len(self.inrange2), group2, gap_error3, len(self.inrange3), group3])

   
        if group1 and group2 and group3:
            return True, errors, self.sequence
        else:
            return False, errors, self.sequence


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

        key = self.agents[int(inrange)[0][0]].group

        if key == 1:
            if length == 1:
                if self.agents[int(inrange[0][0])].id < 4.5:
                    self.agents[int(inrange[0][0])].gap = -np.pi
                else:
                    self.agents[int(inrange[0][0])].gap = np.pi
            if length > 1 and length < 8:
                if self.agents[int(inrange[0][0])].id < 4.5:
                    self.agents[int(inrange[0][0])].gap = -np.abs(self.agents[int(inrange[0][0])].gap)
                else:
                    self.agents[int(inrange[0][0])].gap = np.abs(self.agents[int(inrange[0][0])].gap)

        if key == 2:
            if length == 1:
                if self.agents[int(inrange[0][0])].id < 16.5:
                    self.agents[int(inrange[0][0])].gap = -np.pi
                else:
                    self.agents[int(inrange[0][0])].gap = np.pi
            if length >= 1 and length < 16.5:
                if self.agents[int(inrange[0][0])].id < 16.5:
                    self.agents[int(inrange[0][0])].gap = -np.abs(self.agents[int(inrange[0][0])].gap)
                else:
                    self.agents[int(inrange[0][0])].gap = np.abs(self.agents[int(inrange[0][0])].gap)


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

    def in_range_list(self):
        l = []
        for a in self.agents:
            l.append(a.state)
        return l
