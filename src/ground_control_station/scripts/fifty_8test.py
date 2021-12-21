# coding: utf-8
from fiftyPSO_8test import PSO
import numpy as np

# 导入本代码：from fifty_8test import ST8

# st8 = ST8()
# vxy = st8.op_vol(pxy)


class ST8(object):
    def __init__(self):

        self.dt = 0.3  # 时步/秒

        self.ve = 0.0  # 目标速度
        self.vp = 0.0  # 无人机速度

        self.r0 = 2  # 目标圈半径
        self.r1 = 6  # 初始圈半径

        self.n = 8  # 无人机数目
        self.a = 0.15  # 采用的固定加速度（收缩起始阶段）
        self.amx = 0.15  # 最大加速度限制（收缩控制阶段）

        self.a_bri = 0.15  # 撤退上升加速度
        self.a_bre = 0.15  # 撤退加速
        self.v_bre = 0.4  # 匀速限制
        self.x_bre = 2.0  # 撤退减速距离
        self.d_bre = 5.0  # 撤退触发距离

        self.px, self.py = np.zeros(self.n), np.zeros(self.n)
        self.n_px, self.n_py = np.zeros(self.n), np.zeros(self.n)  # 存储中间值
        # self.phx, self.phy = np.zeros((self.step, self.n)), np.zeros((self.step, self.n))

        self.vp_x, self.vp_y = self.vp * np.ones(self.n), np.zeros(self.n)
        self.vpx_sum, self.vpy_sum = np.zeros(self.n), np.zeros(self.n)
        # self.vp_xh, self.vp_yh = self.vp * np.ones((self.step, self.n)), np.zeros((self.step, self.n))

        self.det_x, self.det_y = np.zeros(self.n), np.zeros(self.n)
        self.ver_xi, self.ver_yi = np.zeros(self.n), np.zeros(self.n)

        self.dis, self.err = np.zeros(self.n), np.zeros(self.n)

        self.vxy = np.zeros((3, self.n))
        self.pxy = np.zeros((3, self.n))

        self.p_xy = np.zeros((3, self.n))

        self.der = 0.2
        self.jud = self.r1 * np.ones(2)

        self.rn = 2  # 两个撤退过程
        self.zt = 6  # 目标高度
        self.z = 4  # 定高
        self.pz = self.z * np.ones(self.n)
        self.pvz = np.zeros(self.n)
        self.n_pz = np.zeros(self.n)
        self.n_pvz = np.zeros(self.n)
        self.juz = self.n * np.ones(self.rn)
        self.tar_x, self.tar_y = [0, 0], [0, 10]
        # self.tar_x, self.tar_y = [-5, -5], [5, 5]
        self.exx, self.eyy = np.zeros(self.rn), np.zeros(self.rn)
        self.n_exx, self.n_eyy = np.zeros(self.rn), np.zeros(self.rn)
        self.vxx, self.vyy = self.ve * np.ones(self.rn), np.zeros(self.rn)
        self.ehx, self.ehy = 0, 0

        self.ct = np.zeros(self.rn)  # 计数器
        self.cc = [0, 0]  # 计数对比
        self.ox, self.oy = np.zeros(self.rn), np.zeros(self.rn)
        self.oxx, self.oyy = np.zeros(self.rn), np.zeros(self.rn)
        self.dis_re, self.dis_te = np.zeros(self.rn), np.zeros(self.rn)
        self.nxs = np.zeros(self.rn)
        self.juz_sym = np.zeros(self.rn)
        self.attack = np.zeros(self.n)  # td信号初始为0
        self.land = np.zeros(self.n)  # 降落信号初始为0
        self.sx, self.sy = 0, 0
        self.ver_xx, self.ver_yy = np.zeros(self.rn), np.zeros(self.rn)

    def op_vol(self, para_xy):  # --------------------------------API: 无人机实时坐标para_xy
        oex, oey = 0, 24  # ---------------------------------API: 获取目标坐标
        self.pxy = para_xy
        ter, cou = np.zeros(2), np.zeros(2)
        jz = np.zeros(self.rn)
        self.ox, self.oy = np.zeros(self.rn), np.zeros(self.rn)
        self.ct = np.zeros(self.rn)
        for i in range(len(self.pxy[0, :])):
            self.px[i] = self.pxy[0, i]
            self.py[i] = self.pxy[1, i]
            self.pz[i] = self.pxy[2, i]

        for k in range(self.n):
            if k % 2 == 0:
                dm = 0
                if self.jud[0] <= 2 * self.der:
                    self.attack[k] = 1
                    bn0 = 0
                    self.juz_sym[bn0] = 1
                    ST8.back(self, k, oex, oey, bn0, jz)
                    self.ehx, self.ehy = self.exx[bn0], self.eyy[bn0]
                else:
                    cou[0] += 1
                    ST8.srk(self, k, oex, oey, ter, dm)
            else:
                dm = 1
                if self.jud[0] <= 2 * self.der:
                    if self.jud[1] <= 2 * self.der and self.nxs[0] == 1:
                        self.attack[k] = 1
                        bn1 = 1
                        self.juz_sym[bn1] = 1
                        ST8.back(self, k, oex, oey, bn1, jz)
                        self.ehx, self.ehy = self.exx[bn1], self.eyy[bn1]
                    else:
                        cou[1] += 1
                        ST8.srk(self, k, oex, oey, ter, dm)
                else:
                    ST8.rem(self, k, oex, oey, ter, dm)

        self.vxy[0, :] = self.vp_x
        self.vxy[1, :] = self.vp_y
        self.vxy[2, :] = self.pvz

        self.p_xy[0, :] = self.n_px
        self.p_xy[1, :] = self.n_py
        self.p_xy[2, :] = self.pz

        for q in range(2):
            if cou[q] != 0:
                self.jud[q] = abs(ter[q]) / cou[q]
        for p in range(self.rn):
            if self.juz_sym[p] == 1:
                self.juz[p] = jz[p] / 4
        # print('vol_x:', self.vxy[0, :])
        # print('')
        # print('vol_y:', self.vxy[1, :])
        # print('')
        # print('vol_z:', self.vxy[2, :])
        # print('')
        # print('px:', self.px)
        # print('')
        print('jud:', self.jud)
        print('')
        print('juz:', self.juz)
        print('')
        # print('pz:', self.pz)
        # print('')
        return self.vxy
        # return self.p_xy, self.ehx, self.ehy

    def back(self, k, oex, oey, bm, jz):
        if self.juz[bm] > self.der:
            if self.pz[k] < self.z + (self.zt - self.z) / 2:  # if self.pz[k] < (self.z + (self.zt - self.z) / 2):
                if self.pvz[k] < (self.v_bre/2):
                    az = self.a_bri
                else:
                    az = 0
            elif self.pz[k] >= self.z + (self.zt - self.z) / 2 and (self.pz[k] < self.zt):
                if self.pz[k] >= (self.zt - (self.v_bre/2)**2 / (2 * self.a_bri)) and (self.pz[k] < self.zt):
                    az = -self.a_bri
                else:
                    az = 0
            else:
                az = 0
                self.pvz[k] = 0
            self.n_pz[k] = self.pz[k] + self.pvz[k] * self.dt + 0.5 * az * self.dt ** 2
            self.n_pvz[k] = self.pvz[k] + az * self.dt
            jz[bm] += (self.zt - self.n_pz[k])
            self.exx[bm], self.eyy[bm] = oex, oey
            self.sx, self.sy = self.exx[bm], self.eyy[bm]
            self.pz[k] = self.n_pz[k]
            self.pvz[k] = self.n_pvz[k]
            self.n_px[k] = self.px[k] + self.ve * self.dt
            self.n_py[k] = self.py[k]
            self.vp_x[k] = self.ve
            self.vp_y[k] = 0
        else:
            self.pvz[k] = 0
            self.ct[bm] += 1
            self.ox[bm] = self.ox[bm] + self.px[k]
            self.oy[bm] = self.oy[bm] + self.py[k]
            # print('px:', self.px)
            # print('')
            # print('ox:', self.ox)
            # print('')

            if self.ct[bm] % 4 == 0:  # if self.cc[bm] != self.ct

                self.dis_re[bm] = np.sqrt((self.tar_x[bm] - self.oxx[bm]) ** 2 + (self.tar_y[bm] - self.oyy[bm]) ** 2)
                self.dis_te[bm] = np.sqrt((self.tar_x[bm] - self.sx) ** 2 + (self.tar_y[bm] - self.sy) ** 2)
                self.ver_xx[bm] = (self.tar_x[bm] - self.sx) / self.dis_te[bm]
                self.ver_yy[bm] = (self.tar_y[bm] - self.sy) / self.dis_te[bm]

                if abs(self.tar_y[bm] - self.oyy[bm]) > 1:
                    self.oxx[bm] += 0 * self.ver_xx[bm] * self.dt
                    self.oyy[bm] += 0.5 * self.ver_yy[bm] * self.dt
                else:
                    self.oxx[bm] += 0 * self.ver_xx[bm] * self.dt
                    self.oyy[bm] += 0 * self.ver_yy[bm] * self.dt
                # print('oxx:', self.oxx)
                # print('')

                # vxy = np.sqrt(self.vxx[bm] ** 2 + self.vyy[bm] ** 2)

                # if (self.dis_te[bm] - self.dis_re[bm]) >= 5:
                # if abs(self.tar_x[0] - self.px[0]) >= 2:
                #     self.nxs[0] = 1
                # print('dis_re', self.dis_re)
                # print('')

                # print('dis_te:', dis_te)
                # print('')
                # if (vxy > 0.2) and (self.vxx[bm] < 0) and (dis_re > 1):
                #     ar = 0
                #     print('1ar:', ar)
                # else:
                #     if dis_re <= 1:
                #         ar = -(self.v_bre**2 / (2 * self.x_bre))
                #         print('21ar:', ar)
                #     else:
                #         ar = 0.1
                #         print('22ar:', ar)
                # if abs(self.tar_x[bm] - self.exx[bm]) < 0.1:
                #     ar = 0
                #     self.vxx[bm], self.vyy[bm] = 0, 0
                #     print('3ar:', ar)
                # if self.vxx[bm] > 0:
                #     self.ver_xx[bm] = -1
                #     self.ver_yy[bm] = 0
                # self.exx[bm] += self.vxx[bm] * self.dt + 0.5 * self.ver_xx[bm] * ar * self.dt ** 2
                # self.eyy[bm] += self.vyy[bm] * self.dt + 0.5 * self.ver_yy[bm] * ar * self.dt ** 2
                # self.vxx[bm] += self.ver_xx[bm] * ar * self.dt
                # self.vyy[bm] += self.ver_yy[bm] * ar * self.dt
                # self.vxx[bm] = -0.2
                # self.vyy[bm] = 0
                # self.n_exx[bm] = self.exx[bm] + self.vxx[bm] * self.dt
                # self.n_eyy[bm] = self.eyy[bm] + self.vyy[bm] * self.dt
                # if abs(self.tar_x[bm] - self.n_exx[bm]) < 1:
                #     self.vxx[bm], self.vyy[bm] = 0, 0
                # self.exx[bm] = self.n_exx[bm]
                # self.eyy[bm] = self.n_eyy[bm]

            # self.cc[bm] = self.ct
            if bm == 0:
                yc = self.py[0]
                d = 1
            else:
                yc = self.py[1]
                d = 0.5
            if abs(self.tar_y[bm] - yc) <= abs(self.sy)-8:
                self.nxs[bm] = 1
            if abs(self.tar_y[bm] - yc) < d:
                self.vp_x[k], self.vp_y[k] = 0, 0
                self.land[k] = 1
            else:
                self.vp_x[k] = 0 * self.ver_xx[bm]
                self.vp_y[k] = 0.5 * self.ver_yy[bm]
            self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
            self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt

    def rem(self, k, oex, oey, ter, dm):
        self.dis[k] = np.sqrt((oex - self.px[k]) ** 2 + (oey - self.py[k]) ** 2)
        self.err[k] = self.r0 - self.dis[k]
        self.n_px[k] = self.px[k] + self.vp * self.dt
        self.n_py[k] = self.py[k]
        ter[dm] += self.err[k]

    def srk(self, k, oex, oey, ter, dm):
        self.dis[k] = np.sqrt((oex - self.px[k]) ** 2 + (oey - self.py[k]) ** 2)
        self.err[k] = self.r0 - self.dis[k]
        self.ver_xi[k] = (oex - self.px[k]) / np.sqrt((oex - self.px[k]) ** 2 + (oey - self.py[k]) ** 2)
        self.ver_yi[k] = (oey - self.py[k]) / np.sqrt((oex - self.px[k]) ** 2 + (oey - self.py[k]) ** 2)
        self.det_x[k] = self.vp_x[k] - self.ve
        self.det_y[k] = self.vp_y[k]
        jud_x = self.ver_xi[k] * self.det_x[k]
        ter[dm] += self.err[k]

        x = self.px[k]
        y = self.py[k]
        vx = self.vp_x[k]
        vy = self.vp_y[k]
        vex = self.ver_xi[k]
        vey = self.ver_yi[k]
        # pso参数
        pn_ = 20
        dim_ = 1
        max_iter_ = 20

        if self.dis[k] <= self.r1 + self.der and (self.dis[k] > (self.r0 + (self.r1 - self.r0) / 2)):
            # if jud_x >= 0:
            # aa = self.a
            # else:
            #     aa = -self.a
            if np.sqrt(vx**2 + vy**2) < self.v_bre:
                aa = self.a
            else:
                aa = 0
        elif self.dis[k] <= (self.r0 + (self.r1 - self.r0) / 2) and (self.dis[k] > self.r0 + self.der):
            if jud_x > 0:
                if self.dis[k] <= (self.r0 + self.v_bre**2 / (2 * self.a)) and (self.dis[k] > self.r0):
                    aa = -self.a
                else:
                    aa = 0
            elif jud_x < 0:
                aa = self.a
            else:
                aa = 0
        else:
            # 进入优化程序
            p_pso = PSO(pn_, dim_, max_iter_, self.amx)
            p_pso.p_init(self.r0, oex, oey, x, y, vx, vy, vex, vey, self.dt)
            aa = p_pso.p_iter(self.r0, oex, oey, x, y, vx, vy, vex, vey, self.dt)

        self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt + 0.5 * self.ver_xi[k] * aa * self.dt ** 2
        self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt + 0.5 * self.ver_yi[k] * aa * self.dt ** 2
        self.vpx_sum[k] = self.vp_x[k] + self.ver_xi[k] * aa * self.dt
        self.vpy_sum[k] = self.vp_y[k] + self.ver_yi[k] * aa * self.dt

        if (self.dis[k] - self.r0) > self.der:
            self.vp_x[k] = self.vpx_sum[k]
            self.vp_y[k] = self.vpy_sum[k]
        else:
            self.vp_x[k] = self.ve
            self.vp_y[k] = 0

        return ter
