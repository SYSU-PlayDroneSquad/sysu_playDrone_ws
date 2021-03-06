# coding: utf-8
# from fiftyPSO_48test import PSO
import numpy as np
import math

# 导入本代码：from fifty_48test import ST8

# st8 = ST8()
# vxy = st8.op_vol(pxy)


class ST8(object):
    def __init__(self):

        self.dt = 0.3  # 时步/秒

        self.ve = 0.0  # 目标速度
        self.vp = 0  # 无人机速度

        self.r0 = 3  # ---------------------------------------------------------[目标圈半径_需实际修改]
        self.r1 = 8  # ---------------------------------------------------------[初始圈半径_需实际修改]
        self.r2 = 16  # ---------------------------------------------------------[中间圈半径_需实际修改]
        self.r3 = 24  # ---------------------------------------------------------[外部圈半径_需实际修改]

        self.n = 48  # 无人机数目
        self.sn = 4  # 单组架次
        self.a = 0.15  # 采用的固定加速度（收缩起始阶段）
        self.amx = 0.15  # 最大加速度限制（收缩控制阶段）

        self.a_bri = 0.15  # 撤退上升加速度
        self.v_up = 1
        self.v_bre = 1.5  # ----------------------------------------------------[匀速限制可调]
        self.v_ret = 2

        self.oex, self.oey = 0, 0  # 需要初始化给定目标起始位置
        self.ver_ex, self.ver_ey = 0, 0  # 目标方向向量初始化

        self.px, self.py = np.zeros(self.n), np.zeros(self.n)
        self.n_px, self.n_py = np.zeros(self.n), np.zeros(self.n)  # 存储中间值
        # self.phx, self.phy = np.zeros((self.step, self.n)), np.zeros((self.step, self.n))

        self.vp_x, self.vp_y = np.zeros(self.n), self.vp * np.ones(self.n)
        self.vpx_sum, self.vpy_sum = np.zeros(self.n), np.zeros(self.n)
        self.vx_t, self.vy_t = np.zeros(self.n), np.zeros(self.n)
        # self.vp_xh, self.vp_yh = self.vp * np.ones((self.step, self.n)), np.zeros((self.step, self.n))

        # self.det_x, self.det_y = np.zeros(self.n), np.zeros(self.n)
        self.ver_xi, self.ver_yi = np.zeros(self.n), np.zeros(self.n)

        self.dis, self.err = np.zeros(self.n), np.zeros(self.n)

        self.vxy = np.zeros((3, self.n))
        self.pxy = np.zeros((3, self.n))

        self.p_xy = np.zeros((3, self.n))

        self.tN = 18  # 总共多少个过程（48时是18）
        self.der = 0.3  # -----------------------------------------------------[误差限可调]
        self.ter, self.cou = np.zeros(self.tN), np.zeros(self.tN)
        self.jud = self.r1 * np.ones(self.tN)
        self.d_sym, self.d_jud = np.zeros(self.n), np.zeros(self.tN)

        self.rn = 12  # 两个撤退过程（48时是12）
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
        self.dis_re, self.dis_rt, self.dis_te = self.n * np.ones(self.rn), np.zeros(self.rn), np.zeros(self.rn)

        self.tar_x, self.tar_y = [], []  # --------------------------[撤离点坐标_需实际修改]
        rex, rey = np.zeros((3, 3)), np.zeros((3, 3))  # -----------36 (3x3)
        d_x, d_y = -10, 10
        rx, ry = 0-1*d_x, -50  # 撤退目标点矩阵的第一点
        for row in range(3):
            rey[row, :] = row * d_y + ry
            for col in range(3):
                rex[row, col] = col * d_x + rx
                self.tar_x.append(rex[row, col])
                self.tar_y.append(rey[row, col])
        self.nxs = np.zeros(self.rn)
        self.pla, self.pat = [], []
        self.attack_jud = np.zeros(self.n)  # gj指令复位
        self.land_jud = np.zeros(self.n)
        self.attack = np.zeros(self.n)  # td信号初始为0
        self.attack_past = np.zeros(self.n)
        self.land = np.zeros(self.n)  # 降落信号初始为0
        self.land_past = np.zeros(self.n)
        self.land_sym = np.zeros(self.n)
        self.ic = np.zeros((2, self.n))  # ip_check table
        self.ic[0, :] = np.arange(0, self.n)
        self.ic[1, :] = np.arange(0, self.n)
        # self.ic[1, :] = np.ones(self.n) * (-1)

        # self.g4 = [[0, 2, 4, 6],
        #            [1, 3, 5, 7],
        #            [8, 12, 16, 20],
        #            [10, 14, 18, 22],
        #            [9, 13, 17, 21],
        #            [11, 15, 19, 23],
        #            [24, 30, 36, 42],
        #            [27, 33, 39, 45],
        #            [25, 31, 37, 43],
        #            [28, 34, 40, 46],
        #            [26, 32, 38, 44],
        #            [29, 35, 41, 47]]
        # self.g8 = [[8, 10, 12, 14, 16, 18, 20, 22],
        #            [9, 11, 13, 15, 17, 19, 21, 23],
        #            [24, 27, 30, 33, 36, 39, 42, 45],
        #            [25, 28, 31, 34, 37, 40, 43, 46],
        #            [26, 29, 32, 35, 38, 41, 44, 47]]
        # self.g16 = [25, 26, 28, 29, 31, 32, 34, 35, 37, 38, 40, 41, 43, 44, 46, 47]

        self.g4 = [[0, 2, 4, 6],  # 1
                   [1, 3, 5, 7],
                   [9, 12, 15, 18],  # 2
                   [10, 13, 16, 19],
                   [8, 11, 14, 17],
                   [20, 24, 28, 32],  # 3
                   [22, 26, 30, 34],
                   [21, 25, 29, 33],
                   [23, 27, 31, 35]]
        self.g8 = [[9, 10, 12, 13, 15, 16, 18, 19],
                   [20, 22, 24, 26, 28, 30, 32, 34],
                   [21, 23, 25, 27, 29, 31, 33, 35]]
        self.g16 = []

    def op_vol(self, para_xy, txy, seq):  # --------------------------------API: 无人机实时坐标para_xy，目标物实时坐标txy
        ex, ey = txy[0], txy[1]
        d_exy = np.sqrt((ex - self.oex) ** 2 + (ey - self.oey) ** 2)
        if d_exy != 0:
            self.ver_ex = (ex - self.oex) / d_exy
            self.ver_ey = (ey - self.oey) / d_exy
        self.oex, self.oey = ex, ey
        self.pxy = para_xy
        self.jz = np.zeros(self.rn)
        self.ct = np.zeros(self.rn)
        self.d_jud = np.zeros(self.tN)
        self.ter, self.cou = np.zeros(self.tN), np.zeros(self.tN)
        self.ox, self.oy = np.zeros(self.rn), np.zeros(self.rn)
        if len(seq) != 0: bi = min(seq)
        else: bi = 0
        self.ic[1, bi:(bi+len(seq))] = seq
        for i in range(len(self.pxy[0, :])):
            p_ip = int(self.ic[1, i])
            self.px[i] = self.pxy[0, p_ip]
            self.py[i] = self.pxy[1, p_ip]
            self.pz[i] = self.pxy[2, p_ip]

        # print(self.ic[0:24])
        # print para_xy[0, 24:48]
        # print(self.px[0:24])

        for k in range(self.n):

            if k < 8:  # 内圈操作
                if k % 2 == 0:
                    if self.jud[0] <= self.der:
                        ST8.attack(self, k)
                        bn = 0  # 第一次撤离
                        self.juz_sym[bn] = 1
                        ST8.back(self, k, bn)
                    else:
                        dm = 0  # 第一步
                        self.cou[dm] += 1
                        lr = self.r0
                        ST8.srk(self, lr, k, dm)
                else:
                    if self.jud[0] <= self.der and self.nxs[0] == 1:
                        if self.jud[1] <= self.der:
                            ST8.attack(self, k)
                            bn = 1
                            self.juz_sym[bn] = 1
                            ST8.back(self, k, bn)
                        else:
                            dm = 1
                            self.cou[dm] += 1
                            lr = self.r0
                            ST8.srk(self, lr, k, dm)
                    else:
                        ST8.rem(self, k)

            elif k < 20:  # 中圈操作
                if self.jud[1] <= self.der:
                    if k % 3 != 2:  # 36
                        if self.jud[2] > self.der:
                            dm = 2
                            self.cou[dm] += 1
                            lr = self.r1
                            ST8.srk(self, lr, k, dm)
                        else:
                            if k % 3 == 0:  # 36
                                if self.jud[3] <= self.der:
                                    ST8.attack(self, k)
                                    bn = 2
                                    self.juz_sym[bn] = 1
                                    ST8.back(self, k, bn)
                                else:
                                    dm = 3
                                    self.cou[dm] += 1
                                    lr = self.r0
                                    ST8.srk(self, lr, k, dm)
                            else:
                                if self.jud[3] <= self.der and self.nxs[2] == 1:
                                    if self.jud[4] <= self.der:
                                        ST8.attack(self, k)
                                        bn = 3
                                        self.juz_sym[bn] = 1
                                        ST8.back(self, k, bn)
                                    else:
                                        dm = 4
                                        self.cou[dm] += 1
                                        lr = self.r0
                                        ST8.srk(self, lr, k, dm)
                                else:
                                    ST8.rem(self, k)
                    else:
                        if self.jud[4] <= self.der:
                            if self.jud[5] > self.der:
                                dm = 5
                                self.cou[dm] += 1
                                lr = self.r1
                                ST8.srk(self, lr, k, dm)
                            else:
                                if k % 3 == 2:  # 36
                                    if self.jud[6] <= self.der:
                                        ST8.attack(self, k)
                                        bn = 4
                                        self.juz_sym[bn] = 1
                                        ST8.back(self, k, bn)
                                    else:
                                        dm = 6
                                        self.cou[dm] += 1
                                        lr = self.r0
                                        ST8.srk(self, lr, k, dm)
                                else:
                                    if self.jud[6] <= self.der and self.nxs[4] == 1 and False:  # 36-----shadow
                                        if self.jud[7] <= self.der:
                                            ST8.attack(self, k)
                                            bn = 5
                                            self.juz_sym[bn] = 1
                                            ST8.back(self, k, bn)
                                        else:
                                            dm = 7
                                            self.cou[dm] += 1
                                            lr = self.r0
                                            ST8.srk(self, lr, k, dm)
                                    else:
                                        ST8.rem(self, k)
                        else:
                            ST8.rem(self, k)
                else:
                    ST8.rem(self, k)

            elif k < 36:  # 外圈操作  36
                if self.jud[5] <= self.der:  # 36
                    # print("ok", k)
                    if k % 2 == 0:  # 36
                        if self.jud[8] > self.der:
                            dm = 8
                            self.cou[dm] += 1
                            lr = self.r1
                            ST8.srk(self, lr, k, dm)
                        else:
                            if self.jud[9] <= self.der:
                                if k % 4 == 0:  # 36
                                    if self.jud[10] <= self.der:
                                        ST8.attack(self, k)
                                        bn = 5  # 36
                                        self.juz_sym[bn] = 1
                                        ST8.back(self, k, bn)
                                    else:
                                        dm = 10
                                        self.cou[dm] += 1
                                        lr = self.r0
                                        ST8.srk(self, lr, k, dm)
                                else:
                                    if self.jud[10] <= self.der and self.nxs[5] == 1:  # 36
                                        if self.jud[11] <= self.der:
                                            ST8.attack(self, k)
                                            bn = 6  # 36
                                            self.juz_sym[bn] = 1
                                            ST8.back(self, k, bn)
                                        else:
                                            dm = 11
                                            self.cou[dm] += 1
                                            lr = self.r0
                                            ST8.srk(self, lr, k, dm)
                                    else:
                                        ST8.rem(self, k)
                            else:
                                ST8.rem(self, k)
                    else:
                        if self.jud[8] <= (self.r2 - self.r1 + self.der):
                            if self.jud[9] > self.der:
                                dm = 9
                                self.cou[dm] += 1
                                lr = self.r2
                                ST8.srk(self, lr, k, dm)
                            else:
                                if self.jud[11] <= self.der:
                                    if k % 2 == 1:  # 36
                                        if self.jud[12] > self.der:
                                            dm = 12
                                            self.cou[dm] += 1
                                            lr = self.r1
                                            ST8.srk(self, lr, k, dm)
                                        else:
                                            if k % 4 == 1:  # 36
                                                if self.jud[13] <= self.der:
                                                    ST8.attack(self, k)
                                                    bn = 7  # 36
                                                    self.juz_sym[bn] = 1
                                                    ST8.back(self, k, bn)
                                                else:
                                                    dm = 13
                                                    self.cou[dm] += 1
                                                    lr = self.r0
                                                    ST8.srk(self, lr, k, dm)
                                            else:
                                                if self.jud[13] <= self.der and self.nxs[7] == 1:  # 36
                                                    if self.jud[14] <= self.der:
                                                        ST8.attack(self, k)
                                                        bn = 8  # 36
                                                        self.juz_sym[bn] = 1
                                                        ST8.back(self, k, bn)
                                                    else:
                                                        dm = 14
                                                        self.cou[dm] += 1
                                                        lr = self.r0
                                                        ST8.srk(self, lr, k, dm)
                                                else:
                                                    ST8.rem(self, k)
                                    else:
                                        if self.jud[14] <= self.der and False:  # 36---------------------shadow
                                            if self.jud[15] > self.der:
                                                dm = 15
                                                self.cou[dm] += 1
                                                lr = self.r1
                                                ST8.srk(self, lr, k, dm)
                                            else:
                                                if k % 6 == 2:
                                                    if self.jud[16] <= self.der:
                                                        ST8.attack(self, k)
                                                        bn = 10
                                                        self.juz_sym[bn] = 1
                                                        ST8.back(self, k, bn)
                                                    else:
                                                        dm = 16
                                                        self.cou[dm] += 1
                                                        lr = self.r0
                                                        ST8.srk(self, lr, k, dm)
                                                else:
                                                    if self.jud[16] <= self.der and self.nxs[10] == 1:
                                                        if self.jud[17] <= self.der:
                                                            ST8.attack(self, k)
                                                            bn = 11
                                                            self.juz_sym[bn] = 1
                                                            ST8.back(self, k, bn)
                                                        else:
                                                            dm = 17
                                                            self.cou[dm] += 1
                                                            lr = self.r0
                                                            ST8.srk(self, lr, k, dm)
                                                    else:
                                                        ST8.rem(self, k)
                                        else:
                                            ST8.rem(self, k)
                                else:
                                    ST8.rem(self, k)
                        else:
                            ST8.rem(self, k)
                else:
                    ST8.rem(self, k)

        for ip in range(self.n):
            f_ip = int(self.ic[0, ip])
            t_ip = int(self.ic[1, ip])
            self.vxy[0, t_ip] = self.vx_t[f_ip]
            self.vxy[1, t_ip] = self.vy_t[f_ip]
            self.vxy[2, t_ip] = self.pvz[f_ip]

        # self.p_xy[0, :] = self.n_px  # comp. sim.
        # self.p_xy[1, :] = self.n_py
        # self.p_xy[2, :] = self.pz

        for q in range(self.tN):
            if self.cou[q] != 0:
                self.jud[q] = abs(self.ter[q]) / self.cou[q]
        for p in range(self.rn):
            if self.juz_sym[p] == 1:
                self.juz[p] = abs(self.jz[p]) / self.sn
        
        # land calc.
        for lj in range(self.n):
            self.land_jud[lj] = int(self.land[lj]) ^ int(self.land_past[lj])
        if sum(self.land_jud) != 0:
            lp = np.nonzero(self.land_jud)
            land_ip = list(lp[0]+np.ones(lp[0].shape))
            land_str = "landList#"+str(int(land_ip[0]))+"#"+str(int(land_ip[1]))+"#"+str(int(land_ip[2]))+"#"+str(int(land_ip[3]))
            self.pla.append(land_str)
        else:
            land_ip = np.zeros(4)
            land_str = " "
        for ld in range(self.n):
            self.land_past[ld] = self.land[ld]
        
        # attack calc
        for aj in range(self.n):
            self.attack_jud[aj] = int(self.attack[aj]) ^ int(self.attack_past[aj])
        if sum(self.attack_jud) != 0:
            ap = np.nonzero(self.attack_jud)
            attack_ip = list(ap[0]+np.ones(ap[0].shape))
            attack_str = "attackList#"+str(int(attack_ip[0]))+"#"+str(int(attack_ip[1]))+"#"+str(int(attack_ip[2]))+"#"+str(int(attack_ip[3]))
            self.pat.append(attack_str)
        else:
            attack_ip = np.zeros(4)
            attack_str = " "
        for ak in range(self.n):
            self.attack_past[ak] = self.attack[ak]
        
        # print "land:", self.pla
        # print ""
        # print "attack:", self.pat
        # print ""
        
        # print('vol_x:', self.vxy[0, :])
        # print('')
        # print('vol_y:', self.vxy[1, :])
        # print('')
        # print('vol_z:', self.vxy[2, :])
        # print('')
        print('jud:', self.jud)
        print('')
        # print('juz:', self.juz)
        # print('')
        # print('rd:', self.dis - self.r0*np.ones(self.n))
        # print('')
        # print('d_jud:', self.d_jud)
        # print('')
        # print('juz_sym:', self.juz_sym)
        # print('')
        
        # print('dis_re:', self.dis_re)
        # print("")

        # print("out_vx:", self.vxy[0, 0:24])
        # print("")
        # print("out_vy:", self.vxy[1, 0:24])
        # print("")
        # print("out_vz:", self.vxy[2, 24:48])
        # print("")

        # self.px = self.n_px  # comp. sim.
        # self.py = self.n_py

        # return self.p_xy
        return self.vxy, land_str, attack_str
        # return self.vxy

    def attack(self, k):
        a_ip = int(self.ic[1, k])
        self.attack[a_ip] = 1
    
    def land(self, k):
        l_ip = int(self.ic[1, k])
        self.land[l_ip] = 1
        self.land_sym[k] = 1
    
    def back(self, k, bm):
        if self.juz[bm] > self.der and self.juz_sym[bm] == 1:
            if self.pz[k] < self.zt - self.der:  # 直接加速变匀速，无减速过程
                if self.pvz[k] < self.v_up:
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
            self.vx_t[k] = self.vp_x[k]
            self.vy_t[k] = self.vp_y[k]

            self.sx, self.sy = self.oex, self.oey
        else:
            
            self.pvz[k] = 0
            lr = self.r0
            
            self.ct[bm] += 1
            self.ox[bm] += self.px[k]
            self.oy[bm] += self.py[k]
            # print('ox:', self.ox)
            # print('')
            # print('oy:', self.oy)
            # print('')
            
            sxx, syy = self.sx, self.sy
            self.dis_te[bm] = np.sqrt((self.tar_x[bm] - sxx) ** 2 + (self.tar_y[bm] - syy) ** 2)
            self.ver_xx[bm] = (self.tar_x[bm] - sxx) / self.dis_te[bm]
            self.ver_yy[bm] = (self.tar_y[bm] - syy) / self.dis_te[bm]
            
            if self.ct[bm] > 4:
                # mt_x, mt_y = ST8.mt(self, lr, k)
                mt_x, mt_y = 0, 0
            else:
                mt_x, mt_y = 0, 0
            
            if self.dis_re[bm] < 3 or self.land_sym[k] == 1:
                if self.vp_x[k] == 0 and self.vp_y[k] == 0:
                    ST8.land(self, k)
                    self.juz_sym[bm] = 0
                self.vp_x[k], self.vp_y[k] = 0, 0
            else:
                self.vp_x[k] = self.v_ret * self.ver_xx[bm] + mt_x
                self.vp_y[k] = self.v_ret * self.ver_yy[bm] + mt_y

            if self.ct[bm] % self.sn == 0:
                self.oxx[bm] = self.ox[bm] / self.sn
                self.oyy[bm] = self.oy[bm] / self.sn
                # print('oxx:', self.oxx)
                # print('')
                # print('oyy:', self.oyy)
                # print('')

                self.dis_re[bm] = np.sqrt((self.tar_x[bm] - self.oxx[bm]) ** 2 + (self.tar_y[bm] - self.oyy[bm]) ** 2)
                self.dis_rt[bm] = np.sqrt((self.sx - self.oxx[bm]) ** 2 + (self.sy - self.oyy[bm]) ** 2)

            if self.dis_rt[bm] > self.r0+1:  # 撤离间隔与触发
                self.nxs[bm] = 1
            
            self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
            self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt
            self.vx_t[k] = round(self.vp_x[k], 2)
            self.vy_t[k] = round(self.vp_y[k], 2)
            # print("vy", self.ver_yy[0:2])
            # # print("vy", self.ver_yy)
            # print("")
            # print("vpy", self.vy_t[0:8])
            # # print("vpy", self.vy_t[0:24])
            # print("")

    def srk(self, lr, k, dm):
        self.dis[k] = np.sqrt((self.oex - self.px[k]) ** 2 + (self.oey - self.py[k]) ** 2)
        self.err[k] = lr - self.dis[k]
        self.ver_xi[k] = (self.oex - self.px[k]) / self.dis[k]
        self.ver_yi[k] = (self.oey - self.py[k]) / self.dis[k]
        self.ter[dm] += self.err[k]

        # jud_x = self.ver_xi[k] * (self.vp_x[k] - self.ve)

        # x = self.px[k]
        # y = self.py[k]
        vx = self.vp_x[k]
        vy = self.vp_y[k]
        v_sum = np.sqrt(vx**2 + vy**2)
        
        vt, ver_xt, ver_yt = ST8.angle(self, lr, k, dm)  # 36

        # 无夹角均分机制时的参数
        # vt = 0
        # ver_xt, ver_yt = 0, 0

        # # pso参数
        # pn_ = 20
        # dim_ = 1
        # max_iter_ = 20

        # if k > 24 and (k < 48):
        #     print(self.dis[k])

        if self.dis[k] > lr + self.der:
            if v_sum < self.v_bre:
                aa = self.a
                # print("1")
            else:
                aa = 0
                # print("2")
        # elif self.dis[k] <= lr + self.v_bre**2 / (2 * self.a):
        #     if jud_x > 0:
        #         aa = -0.8 * self.a  # 带一点点减速度
        #     elif jud_x < 0:
        #         aa = 0.8 * self.a  # 用于修正位置
        #     else:
        #         aa = 0
        else:
            # # 进入优化程序
            # p_pso = PSO(pn_, dim_, max_iter_, self.amx)
            # p_pso.p_init(lr, self.oex, self.oey, x, y, vx, vy, vex, vey, self.dt)
            # aa = p_pso.p_iter(lr, self.oex, self.oey, x, y, vx, vy, vex, vey, self.dt)
            aa = 0
            # print("3")

        vxx = self.ver_xi[k] * aa
        vyy = self.ver_yi[k] * aa
        # self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt + 0.5 * vxx * self.dt ** 2
        # self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt + 0.5 * vyy * self.dt ** 2
        self.n_px[k] = self.px[k] + vt * ver_xt * self.dt + self.vp_x[k] * self.dt + 0.5 * vxx * self.dt ** 2
        self.n_py[k] = self.py[k] + vt * ver_yt * self.dt + self.vp_y[k] * self.dt + 0.5 * vyy * self.dt ** 2
        self.vpx_sum[k] = self.vp_x[k] + vxx * self.dt
        self.vpy_sum[k] = self.vp_y[k] + vyy * self.dt

        if (self.dis[k] - lr) >= self.der:
            self.vp_x[k] = self.vpx_sum[k]
            self.vp_y[k] = self.vpy_sum[k]
        else:
            self.vp_x[k] = self.ve * self.ver_ex
            self.vp_y[k] = self.ve * self.ver_ey

        self.vx_t[k] = self.vp_x[k] + vt * ver_xt
        self.vy_t[k] = self.vp_y[k] + vt * ver_yt
        
        # print(self.vp_x[24:48])
        # print(self.vp_y[24:48])
    
    def angle(self, lr, k, dm):  # 36
        # 夹角均分机制
        if lr == self.r0 or dm == 5:  # 36
            index = list([m, xx.index(k)] for m, xx in enumerate(self.g4) if k in xx)
            lin = self.g4[index[0][0]][index[0][1] - 1]
            uin = self.g4[index[0][0]][(index[0][1] + 1) % len(self.g4[index[0][0]])]
        elif lr == self.r1 or dm == 9:  # 36
            index = list([m, xx.index(k)] for m, xx in enumerate(self.g8) if k in xx)
            lin = self.g8[index[0][0]][index[0][1] - 1]
            uin = self.g8[index[0][0]][(index[0][1] + 1) % len(self.g8[index[0][0]])]
        else:
            index = self.g16.index(k)
            lin = self.g16[index - 1]
            uin = self.g16[(index + 1) % len(self.g16)]
        ver_yt = -self.ver_xi[k]/(self.ver_xi[k]**2+self.ver_yi[k]**2)
        ver_xt = -ver_yt*self.ver_yi[k]/self.ver_xi[k]
        dis_l = np.sqrt((self.oex - self.px[lin]) ** 2 + (self.oey - self.py[lin]) ** 2)
        dis_u = np.sqrt((self.oex - self.px[uin]) ** 2 + (self.oey - self.py[uin]) ** 2)
        ver_x_l = (self.oex - self.px[lin]) / dis_l

        ver_y_l = (self.oey - self.py[lin]) / dis_l
        ver_x_u = (self.oex - self.px[uin]) / dis_u
        ver_y_u = (self.oey - self.py[uin]) / dis_u
        theta_i_l = math.acos(self.ver_xi[k]*ver_x_l+self.ver_yi[k]*ver_y_l)
        theta_i_u = math.acos(self.ver_xi[k]*ver_x_u+self.ver_yi[k]*ver_y_u)
        gap_lu = theta_i_u - theta_i_l
        crs = ver_xt*self.ver_yi[k] - self.ver_xi[k]*ver_yt
        if crs > 0:
            vt = 0.1 * gap_lu * self.dis[k] / self.dt
        else:
            vt = -0.1 * gap_lu * self.dis[k] / self.dt
        
        return vt, ver_xt, ver_yt

    def mt(self, lr, k, dm):
        self.dis[k] = np.sqrt((self.oxx - self.px[k]) ** 2 + (self.oyy - self.py[k]) ** 2)
        self.err[k] = lr - self.dis[k]
        self.ver_xi[k] = (self.oxx - self.px[k]) / self.dis[k]
        self.ver_yi[k] = (self.oyy - self.py[k]) / self.dis[k]
        
        if self.err[k] < -self.der/2:
            vo = 1
        elif self.err[k] > self.der/2:
            vo = -1
        else:
            vo = 0
        
        vt, ver_xt, ver_yt = ST8.angle(self, lr, k, dm)
        mt_x = vo * self.ver_xi[k] + vt * ver_xt
        mt_y = vo * self.ver_yi[k] + vt * ver_yt
        
        return mt_x, mt_y

    def rem(self, k):
        self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
        self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt
        self.vp_x[k] = self.ve * self.ver_ex
        self.vp_y[k] = self.ve * self.ver_ey
        self.vx_t[k] = self.vp_x[k]
        self.vy_t[k] = self.vp_y[k]

