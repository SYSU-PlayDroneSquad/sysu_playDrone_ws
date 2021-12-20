#!/usr/bin/env python
# coding=utf-8
import random
import numpy as np


class PSO(object):
    def __init__(self, pn, dim, max_iter, aa):
        self.w = 0.73
        self.c1 = 1.5
        self.c2 = 1.5
        self.r1 = np.random.random(1)
        self.r2 = np.random.random(1)
        self.pn = pn  # 粒子数量
        self.dim = dim  # 搜索维度
        self.max_iter = max_iter  # 迭代次数

        self.pa_lim = aa
        self.pav_lim = 0.3 * self.pa_lim

        self.x = np.zeros((self.pn, self.dim))  # 所有粒子的位置和速度
        self.v = np.zeros((self.pn, self.dim))
        self.p_best = np.zeros((self.pn, self.dim))  # 个体经历的最佳位置和全局最佳位置
        self.g_best = np.zeros((1, self.dim))
        self.p_fit = np.zeros(self.pn)  # 每个个体的历史最佳适应值
        self.fit = 1e10  # 全局最佳适应值

    # ---------------------e目标函数-----------------------------
    @staticmethod
    def p_func(a, r0, oex, oey, x, y, vx, vy, vex, vey, dt):
        nex = x + vx * dt + 0.5 * vex * a * dt**2
        ney = y + vy * dt + 0.5 * vey * a * dt**2
        dis = np.sqrt((oex-nex)**2+(oey-ney)**2)
        err = (r0 - dis)**2
        return err

    # ---------------------e初始化种群----------------------------------
    def p_init(self, r0, oex, oey, x, y, vx, vy, vex, vey, dt):
        for i in range(self.pn):
            for j in range(self.dim):
                self.x[i][j] = random.uniform(-self.pa_lim, self.pa_lim)
                self.v[i][j] = random.uniform(-self.pav_lim, self.pav_lim)
            self.p_best[i] = self.x[i]
            tmp = self.p_func(self.x[i], r0, oex, oey, x, y, vx, vy, vex, vey, dt)  # 初始化目标函数
            self.p_fit[i] = tmp
            if tmp < self.fit:
                self.fit = tmp
                self.g_best = self.x[i]

    # ----------------------e更新粒子位置----------------------------------
    def p_iter(self, r0, oex, oey, x, y, vx, vy, vex, vey, dt):
        for k in range(self.max_iter):
            for i in range(self.pn):  # 更新g_best\p_best
                temp = self.p_func(self.x[i], r0, oex, oey, x, y, vx, vy, vex, vey, dt)  # 目标函数迭代
                if temp < self.p_fit[i]:  # 更新个体最优
                    self.p_fit[i] = temp
                    self.p_best[i] = self.x[i]
                    if self.p_fit[i] < self.fit:  # 更新全局最优
                        self.g_best = self.x[i]
                        self.fit = self.p_fit[i]
            for i in range(self.pn):
                self.v[i] = self.w * self.v[i] + self.c1 * self.r1 * (self.p_best[i] - self.x[i]) + \
                            self.c2 * self.r2 * (self.g_best - self.x[i])  # 速度更新
                self.x[i] = self.x[i] + self.v[i]  # 位置更新
                if self.v[i] < -self.pav_lim:  # 速度边界调整
                    self.v[i] = -self.pav_lim
                if self.v[i] > self.pav_lim:
                    self.v[i] = self.pav_lim
                if self.x[i] < -self.pa_lim:  # 位置边界调整
                    self.x[i] = -self.pa_lim
                if self.x[i] > self.pa_lim:
                    self.x[i] = self.pa_lim
        return self.g_best
