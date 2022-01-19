# coding: utf-8
from fiftyPSO_8test import PSO
import numpy as np
import math

# 导入本代码：from fifty_8test import ST8

# st8 = ST8()
# vxy = st8.op_vol(pxy)


class ST8(object):
    def __init__(self):

        self.dt = 0.3  # 时步/秒

        self.ve = 0.0  # 目标速度
        self.vp = 0.0  # 无人机速度

        self.r0 = 3.2  # ---------------------------------------------------------[目标圈半径_需实际修改]
        self.r1 = 8  # ---------------------------------------------------------[初始圈半径_需实际修改]

        self.n = 8  # 无人机数目
        self.sn = 4  # 单组架次
        self.a = 0.15  # 采用的固定加速度（收缩起始阶段）
        self.amx = 0.15  # 最大加速度限制（收缩控制阶段）

        self.a_bri = 0.15  # 撤退上升加速度
        self.v_bre = 0.8  # ----------------------------------------------------[匀速限制可调]

        self.oex, self.oey = 0, 0  # 需要初始化给定目标起始位置
        self.ver_ex, self.ver_ey = 0, 0

        self.px, self.py = np.zeros(self.n), np.zeros(self.n)
        self.n_px, self.n_py = np.zeros(self.n), np.zeros(self.n)  # 存储中间值
        # self.phx, self.phy = np.zeros((self.step, self.n)), np.zeros((self.step, self.n))

        self.vp_x, self.vp_y = np.zeros(self.n), np.zeros(self.n)
        self.vpx_sum, self.vpy_sum = np.zeros(self.n), np.zeros(self.n)
        self.vx_t, self.vy_t = np.zeros(self.n), np.zeros(self.n)
        # self.vp_xh, self.vp_yh = self.vp * np.ones((self.step, self.n)), np.zeros((self.step, self.n))

        # self.det_x, self.det_y = np.zeros(self.n), np.zeros(self.n)
        self.ver_xi, self.ver_yi = np.zeros(self.n), np.zeros(self.n)

        self.dis, self.err = np.zeros(self.n), np.zeros(self.n)

        self.vxy = np.zeros((3, self.n))
        self.pxy = np.zeros((3, self.n))

        self.p_xy = np.zeros((3, self.n))

        self.tN = 2  # 总共多少个过程（48时是18）
        self.der = 0.3  # -----------------------------------------------------[误差限可调]
        self.ter, self.cou = np.zeros(self.tN), np.zeros(self.tN)
        self.jud = self.r1 * np.ones(2)
        self.d_sym, self.d_jud = np.zeros(self.n), np.zeros(self.tN)

        self.rn = 2  # 两个撤退过程（48时是12）
        self.zt = 7  # 目标高度<这个很重要>
        self.z = 5  # 定高<其实不重要了>
        self.pz = self.z * np.ones(self.n)  # 初始化，后面会覆盖
        self.pvz = np.zeros(self.n)
        self.n_pz = np.zeros(self.n)
        self.n_pvz = np.zeros(self.n)
        self.jz = np.zeros(self.rn)
        self.juz = self.n * np.ones(self.rn)
        self.juz_sym = np.zeros(self.rn)

        self.sx, self.sy = 0, 0
        self.ct = np.zeros(self.rn)  # 计数器
        self.ox, self.oy = np.zeros(self.rn), np.zeros(self.rn)
        self.oxx, self.oyy = np.zeros(self.rn), np.zeros(self.rn)
        self.ver_xx, self.ver_yy = np.zeros(self.rn), np.zeros(self.rn)
        self.dis_re, self.dis_rt, self.dis_te = np.zeros(self.rn), np.zeros(self.rn), np.zeros(self.rn)

        self.nxs = np.zeros(self.rn)
        self.tar_x, self.tar_y = [-2.5, 7.5], [15, 15]  # --------------------------[撤离点坐标_需实际修改]
        self.attack = np.zeros(self.n)  # td信号初始为0
        self.land = np.zeros(self.n)  # 降落信号初始为0

    def op_vol(self, para_xy, txy):  # --------------------------------API: 无人机实时坐标para_xy，目标物实时坐标txy
        ex, ey = txy[0], txy[1]
        # d_exy = np.sqrt((ex - self.oex) ** 2 + (ey - self.oey) ** 2)
        # if d_exy != 0:
        #     self.ver_ex = (ex - self.oex) / d_exy
        #     self.ver_ey = (ey - self.oey) / d_exy
        self.oex, self.oey = ex, ey
        self.pxy = para_xy
        self.jz = np.zeros(self.rn)
        self.ct = np.zeros(self.rn)
        self.attack = np.zeros(self.n)  # gj指令复位
        self.d_jud = np.zeros(self.tN)
        self.ter, self.cou = np.zeros(self.tN), np.zeros(self.tN)
        self.ox, self.oy = np.zeros(self.rn), np.zeros(self.rn)
        for i in range(len(self.pxy[0, :])):
            self.px[i] = self.pxy[0, i]
            self.py[i] = self.pxy[1, i]
            self.pz[i] = self.pxy[2, i]

        for k in range(self.n):
            if k % 2 == 0:
                dm = 0  # 第一步
                if self.jud[0] <= self.der:
                    self.attack[k] = 1
                    bn0 = 0  # 第一次撤离
                    self.juz_sym[bn0] = 1
                    ST8.back(self, k, bn0)
                else:
                    self.cou[dm] += 1
                    ST8.srk(self, k, dm)
            else:
                dm = 1
                if self.jud[0] <= self.der and self.nxs[0] == 1:
                    if self.jud[1] <= self.der:
                        self.attack[k] = 1
                        bn1 = 1
                        self.juz_sym[bn1] = 1
                        ST8.back(self, k, bn1)
                    else:
                        self.cou[dm] += 1
                        ST8.srk(self, k, dm)
                else:
                    ST8.rem(self, k, dm)

        self.vxy[0, :] = self.vp_x
        self.vxy[1, :] = self.vp_y
        self.vxy[2, :] = self.pvz

        self.p_xy[0, :] = self.n_px
        self.p_xy[1, :] = self.n_py
        self.p_xy[2, :] = self.pz

        for q in range(self.tN):
            if self.cou[q] != 0:  # and self.d_jud[q] / self.sn == 1
                self.jud[q] = abs(self.ter[q]) / self.cou[q]
        for p in range(self.rn):
            if self.juz_sym[p] == 1:
                self.juz[p] = abs(self.jz[p]) / self.sn
        # print('vol_x:', self.vxy[0, :])
        # print('')
        # print('vol_y:', self.vxy[1, :])
        # print('')
        # print('vol_z:', self.vxy[2, :])
        # print('')
        print('jud:', self.jud)
        print('')
        print('juz:', self.juz)
        print('')
        # print('rd:', self.dis - self.r0*np.ones(self.n))
        # print('')
        # print('d_jud:', self.d_jud)
        # print('')
        # print('d_sym:', self.d_sym)
        # print('')

        # return self.vxy, self.land
        return self.vxy

    def back(self, k, bm):
        if self.juz[bm] > self.der:
            if self.pz[k] < self.zt - self.der:  # 直接加速变匀速，无减速过程
                if self.pvz[k] < (self.v_bre/2):
                    az = self.a_bri
                else:
                    az = 0
            else:  # 直接输出0速度控制
                az = 0
                self.pvz[k] = 0
            self.n_pz[k] = self.pz[k] + self.pvz[k] * self.dt + 0.5 * az * self.dt ** 2
            self.n_pvz[k] = self.pvz[k] + az * self.dt
            self.jz[bm] += (self.zt - self.n_pz[k])

            self.pz[k] = self.n_pz[k]
            self.pvz[k] = self.n_pvz[k]

            self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
            self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt
            self.vp_x[k] = self.ve * self.ver_ex
            self.vp_y[k] = self.ve * self.ver_ey

            self.sx, self.sy = self.oex, self.oey
        else:
            self.pvz[k] = 0
            self.ct[bm] += 1
            self.ox[bm] += self.px[k]
            self.oy[bm] += self.py[k]
            # print('ox:', self.ox)
            # print('')
            # print('oy:', self.oy)
            # print('')

            if self.ct[bm] % self.sn == 0:
                self.oxx[bm] = self.ox[bm] / self.sn
                self.oyy[bm] = self.oy[bm] / self.sn
                # print('oxx:', self.oxx)
                # print('')
                # print('oyy:', self.oyy)
                # print('')

                self.dis_re[bm] = np.sqrt((self.tar_x[bm] - self.oxx[bm]) ** 2 + (self.tar_y[bm] - self.oyy[bm]) ** 2)
                self.dis_rt[bm] = np.sqrt((self.sx - self.oxx[bm]) ** 2 + (self.sy - self.oyy[bm]) ** 2)
                self.dis_te[bm] = np.sqrt((self.tar_x[bm] - self.sx) ** 2 + (self.tar_y[bm] - self.sy) ** 2)
                self.ver_xx[bm] = (self.tar_x[bm] - self.sx) / self.dis_te[bm]
                self.ver_yy[bm] = (self.tar_y[bm] - self.sy) / self.dis_te[bm]

            if self.dis_rt[bm] > self.r0+1:  # 撤离间隔与触发
                self.nxs[bm] = 1

            if self.dis_re[bm] < 1:
                self.vp_x[k], self.vp_y[k] = 0, 0
                self.land[k] = 1
            else:
                self.vp_x[k] = self.v_bre * self.ver_xx[bm]
                self.vp_y[k] = self.v_bre * self.ver_yy[bm]
            self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
            self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt

    def rem(self, k, dm):
        self.dis[k] = np.sqrt((self.oex - self.px[k]) ** 2 + (self.oey - self.py[k]) ** 2)
        self.err[k] = self.r0 - self.dis[k]
        self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
        self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt
        self.vp_x[k] = self.ve * self.ver_ex
        self.vp_y[k] = self.ve * self.ver_ey
        self.ter[dm] += self.err[k]

    def srk(self, k, dm):
        self.dis[k] = np.sqrt((self.oex - self.px[k]) ** 2 + (self.oey - self.py[k]) ** 2)
        self.err[k] = self.r0 - self.dis[k]
        self.ver_xi[k] = (self.oex - self.px[k]) / self.dis[k]
        self.ver_yi[k] = (self.oey - self.py[k]) / self.dis[k]
        self.ter[dm] += self.err[k]

        jud_x = self.ver_xi[k] * (self.vp_x[k] - self.ve)
        # if self.dis[k] <= self.r0 + self.der:
        #     self.d_sym[k] = 1
        # self.d_jud[dm] = self.d_jud[dm] + self.d_sym[k]

        x = self.px[k]
        y = self.py[k]
        vx = self.vp_x[k]
        vy = self.vp_y[k]
        vex = self.ver_xi[k]
        vey = self.ver_yi[k]
        v_sum = np.sqrt(vx**2 + vy**2)

        # ver_yt = -self.ver_xi[k]/(self.ver_xi[k]**2+self.ver_yi[k]**2)
        # ver_xt = -ver_yt*self.ver_yi[k]/self.ver_xi[k]
        # dis_l = np.sqrt((self.oex - self.px[k-2]) ** 2 + (self.oey - self.py[k-2]) ** 2)
        # dis_u = np.sqrt((self.oex - self.px[(k+2) % 8]) ** 2 + (self.oey - self.py[(k+2) % 8]) ** 2)
        # ver_x_l = (self.oex - self.px[k-2]) / dis_l
        # ver_y_l = (self.oey - self.py[k-2]) / dis_l
        # ver_x_u = (self.oex - self.px[(k+2) % 8]) / dis_u
        # ver_y_u = (self.oey - self.py[(k+2) % 8]) / dis_u
        # theta_i_l = math.acos((vex*ver_x_l+vey*ver_y_l)/(self.dis[k]*dis_l))
        # theta_i_u = math.acos((vex*ver_x_u+vey*ver_y_u)/(self.dis[k]*dis_u))
        # gap_lu = theta_i_u - theta_i_l
        # crs = ver_xt*self.ver_yi[k] - self.ver_xi[k]*ver_yt
        # if crs > 0:
        #     vt = 0.05 * gap_lu / abs(gap_lu)
        # else:
        #     vt = -0.05 * gap_lu / abs(gap_lu)

        # pso参数
        pn_ = 20
        dim_ = 1
        max_iter_ = 20

        if self.dis[k] > self.r0 + self.der:
            if v_sum < self.v_bre:
                aa = self.a
            else:
                aa = 0
        elif self.dis[k] <= (self.r0 + self.v_bre**2 / (2 * self.a)):
            if jud_x > 0:
                aa = -0.8 * self.a  # 带一点点减速度
            elif jud_x < 0:
                aa = 0.8 * self.a  # 用于修正位置
            else:
                aa = 0
        else:
            # 进入优化程序
            p_pso = PSO(pn_, dim_, max_iter_, self.amx)
            p_pso.p_init(self.r0, self.oex, self.oey, x, y, vx, vy, vex, vey, self.dt)
            aa = p_pso.p_iter(self.r0, self.oex, self.oey, x, y, vx, vy, vex, vey, self.dt)

        vxx = self.ver_xi[k] * aa
        vyy = self.ver_yi[k] * aa
        self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt + 0.5 * vxx * self.dt ** 2
        self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt + 0.5 * vyy * self.dt ** 2
        self.vpx_sum[k] = self.vp_x[k] + vxx * self.dt
        self.vpy_sum[k] = self.vp_y[k] + vyy * self.dt

        if (self.dis[k] - self.r0) >= self.der:
            self.vp_x[k] = self.vpx_sum[k]
            self.vp_y[k] = self.vpy_sum[k]
        else:
            self.vp_x[k] = self.ve * self.ver_ex
            self.vp_y[k] = self.ve * self.ver_ey
