# coding: utf-8

# from fiftyPSO_48AR import PSO
import numpy as np
import math

# 导入代码
# from fifty_48AR import ST8

# 调用代码
# st8 = ST8()  # 初始化一次即可
# vel_xy, land_cmd, attack_cmd = st8.op_vol(loc_arr, txy, seq)  # 循环调用


class ST8(object):
    def __init__(self):

        self.dt = 0.3  # 时步/秒
        self.ve = 0  # 目标速度(可预设/也可传入)
        self.vp = 0  # 无人机速度

        self.r0 = 3  # ---------------------------------------------------------[目标圈半径_需实际修改]
        self.r1 = 10  # ---------------------------------------------------------[初始圈半径_需实际修改]
        self.r2 = 20  # --------------------------------------------------------[中间圈半径_需实际修改]
        self.r3 = 24  # --------------------------------------------------------[外部圈半径_需实际修改]

        self.n = 48  # 无人机数目
        self.sn = 4  # 单组架次
        self.a = 0.15  # 采用的固定加速度_收缩阶段
        self.amx = 0.15
        self.a_bri = 0.15  # 撤退上升加速度
        self.v_up = 1  # -----------------------------------------------------[上升匀速限制可调]
        self.v_bre = 1  # ----------------------------------------------------[收缩匀速限制可调]
        self.v_ret = 2  # ----------------------------------------------------[撤退匀速限制可调]

        self.oex, self.oey = 0, 0
        self.ver_ex, self.ver_ey = 0, 0

        self.px, self.py = np.zeros(self.n), np.zeros(self.n)
        self.n_px, self.n_py = np.zeros(self.n), np.zeros(self.n)

        self.vp_x, self.vp_y = np.zeros(self.n), self.vp * np.ones(self.n)
        self.vpx_sum, self.vpy_sum = np.zeros(self.n), np.zeros(self.n)
        self.vx_t, self.vy_t = np.zeros(self.n), np.zeros(self.n)

        self.ver_xi, self.ver_yi = np.zeros(self.n), np.zeros(self.n)

        self.dis, self.err = np.zeros(self.n), np.zeros(self.n)

        self.vxy = np.zeros((3, self.n))
        self.pxy = np.zeros((3, self.n))

        self.p_xy = np.zeros((3, self.n))

        self.tN = 18
        self.der = 0.3  # -------------------------------------------------------[误差限可调]
        self.ter, self.cou = np.zeros(self.tN), np.zeros(self.tN)
        self.jud = self.r1 * np.ones(self.tN)
        self.d_sym, self.d_jud = np.zeros(self.n), np.zeros(self.tN)

        self.rn = 12
        self.zt = 6  # -----------------------------------------------------[上升目标高度可调]
        self.z = 5
        self.rou = np.zeros(self.rn)
        self.pz = self.z * np.ones(self.n)
        self.pvz = np.zeros(self.n)
        self.n_pz = np.zeros(self.n)
        self.n_pvz = np.zeros(self.n)
        self.jz = np.zeros(self.rn)
        self.juz = self.n * np.ones(self.rn)
        self.juz_sym = np.zeros(self.rn)

        self.sx, self.sy = 0, 0
        self.ct = np.zeros(self.rn)
        self.ox, self.oy = np.zeros(self.rn), np.zeros(self.rn)
        self.oxx, self.oyy = np.zeros(self.rn), np.zeros(self.rn)
        self.ver_xx, self.ver_yy = np.zeros(self.rn), np.zeros(self.rn)
        self.dis_re, self.dis_rt, self.dis_te = self.n * np.ones(self.rn), np.zeros(self.rn), np.zeros(self.rn)

        self.tar_x, self.tar_y = [], []
        rx0, ry0 = 0, -30  # ---------------------------------------------[撤离点坐标_需实际修改]
        r_row, r_col = 3, 4
        rex, rey = np.zeros((r_row, r_col)), np.zeros((r_row, r_col))
        d_x, d_y = -10, 10
        rx, ry = rx0-(len(rex[0, :])-1)*d_x/2, ry0  # 撤退目标点矩阵的第一点
        for row in range(r_row):
            rey[row, :] = row * d_y + ry
            for col in range(r_col):
                rex[row, col] = col * d_x + rx
                self.tar_x.append(rex[row, col])
                self.tar_y.append(rey[row, col])

        self.nxs = np.zeros(self.rn)
        self.pla, self.pat = [], []
        self.attack_jud = np.zeros(self.n)
        self.land_jud = np.zeros(self.n)
        self.attack = np.zeros(self.n)
        self.attack_past = np.zeros(self.n)
        self.land = np.zeros(self.n)
        self.land_past = np.zeros(self.n)
        self.land_sym = 0

        self.ic = np.zeros((2, self.n))
        self.ic[0, :] = np.arange(0, self.n)
        self.ic[1, :] = np.arange(0, self.n)

        self.g4 = [[0, 2, 4, 6],
                   [1, 3, 5, 7],
                   [8, 12, 16, 20],
                   [10, 14, 18, 22],
                   [9, 13, 17, 21],
                   [11, 15, 19, 23],
                   [24, 30, 36, 42],
                   [27, 33, 39, 45],
                   [25, 31, 37, 43],
                   [28, 34, 40, 46],
                   [26, 32, 38, 44],
                   [29, 35, 41, 47]]
        self.g8 = [[8, 10, 12, 14, 16, 18, 20, 22],
                   [9, 11, 13, 15, 17, 19, 21, 23],
                   [24, 27, 30, 33, 36, 39, 42, 45],
                   [25, 28, 31, 34, 37, 40, 43, 46],
                   [26, 29, 32, 35, 38, 41, 44, 47]]
        self.g16 = [25, 26, 28, 29, 31, 32, 34, 35, 37, 38, 40, 41, 43, 44, 46, 47]

    def op_vol(self, para_xy, txy, seq, ve):  # -----------------API: 无人机实时坐标para_xy，目标物实时坐标txy，排序序列seq
        ex, ey = txy[0], txy[1]
        self.ve = np.sqrt(ve[0]**2+ve[1]**2)
        d_exy = np.sqrt((ex - self.oex) ** 2 + (ey - self.oey) ** 2)
        if d_exy != 0:
            self.ver_ex = (ex - self.oex) / d_exy
            self.ver_ey = (ey - self.oey) / d_exy
        self.oex, self.oey = ex, ey
        self.pxy = para_xy
        self.jz = np.zeros(self.rn)
        self.ct = np.zeros(self.rn)
        self.d_jud = np.zeros(self.tN)
        self.rou = np.zeros(self.rn)
        self.ter, self.cou = np.zeros(self.tN), np.zeros(self.tN)
        self.ox, self.oy = np.zeros(self.rn), np.zeros(self.rn)
        if len(seq) != 0:
            bi = min(seq)
        else:
            bi = 0
        self.ic[1, bi:(bi+len(seq))] = seq
        for i in range(len(self.pxy[0, :])):
            p_ip = int(self.ic[1, i])
            self.px[i] = self.pxy[0, p_ip]
            self.py[i] = self.pxy[1, p_ip]
            self.pz[i] = self.pxy[2, p_ip]

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
                            lr = self.r0
                            ST8.srk(self, lr, k, dm)
                    else:
                        ST8.rem(self, k)

            elif k < 24:  # 中圈操作
                if self.jud[1] <= self.der:
                    if k % 2 == 0:
                        if self.jud[2] > self.der:
                            dm = 2
                            lr = self.r1
                            ST8.srk(self, lr, k, dm)
                        else:
                            if k % 4 == 0:
                                if self.jud[3] <= self.der:
                                    ST8.attack(self, k)
                                    bn = 2
                                    self.juz_sym[bn] = 1
                                    ST8.back(self, k, bn)
                                else:
                                    dm = 3
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
                                        lr = self.r0
                                        ST8.srk(self, lr, k, dm)
                                else:
                                    ST8.rem(self, k)
                    else:
                        if self.jud[4] <= self.der:
                            if self.jud[5] > self.der:
                                dm = 5
                                lr = self.r1
                                ST8.srk(self, lr, k, dm)
                            else:
                                if k % 4 == 1:
                                    if self.jud[6] <= self.der:
                                        ST8.attack(self, k)
                                        bn = 4
                                        self.juz_sym[bn] = 1
                                        ST8.back(self, k, bn)
                                    else:
                                        dm = 6
                                        lr = self.r0
                                        ST8.srk(self, lr, k, dm)
                                else:
                                    if self.jud[6] <= self.der and self.nxs[4] == 1:
                                        if self.jud[7] <= self.der:
                                            ST8.attack(self, k)
                                            bn = 5
                                            self.juz_sym[bn] = 1
                                            ST8.back(self, k, bn)
                                        else:
                                            dm = 7
                                            lr = self.r0
                                            ST8.srk(self, lr, k, dm)
                                    else:
                                        ST8.rem(self, k)
                        else:
                            ST8.rem(self, k)
                else:
                    ST8.rem(self, k)

            else:  # 外圈操作
                if self.jud[6] <= self.der and False:
                    if k % 3 == 0:
                        if self.jud[8] > self.der:
                            dm = 8
                            lr = self.r1
                            ST8.srk(self, lr, k, dm)
                        else:
                            if self.jud[9] <= self.der:
                                if k % 6 == 0:
                                    if self.jud[10] <= self.der:
                                        ST8.attack(self, k)
                                        bn = 6
                                        self.juz_sym[bn] = 1
                                        ST8.back(self, k, bn)
                                    else:
                                        dm = 10
                                        lr = self.r0
                                        ST8.srk(self, lr, k, dm)
                                else:
                                    if self.jud[10] <= self.der and self.nxs[6] == 1:
                                        if self.jud[11] <= self.der:
                                            ST8.attack(self, k)
                                            bn = 7
                                            self.juz_sym[bn] = 1
                                            ST8.back(self, k, bn)
                                        else:
                                            dm = 11
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
                                lr = self.r2
                                ST8.srk(self, lr, k, dm)
                            else:
                                if self.jud[11] <= self.der:
                                    if k % 3 == 1:
                                        if self.jud[12] > self.der:
                                            dm = 12
                                            lr = self.r1
                                            ST8.srk(self, lr, k, dm)
                                        else:
                                            if k % 6 == 1:
                                                if self.jud[13] <= self.der:
                                                    ST8.attack(self, k)
                                                    bn = 8
                                                    self.juz_sym[bn] = 1
                                                    ST8.back(self, k, bn)
                                                else:
                                                    dm = 13
                                                    lr = self.r0
                                                    ST8.srk(self, lr, k, dm)
                                            else:
                                                if self.jud[13] <= self.der and self.nxs[8] == 1:
                                                    if self.jud[14] <= self.der:
                                                        ST8.attack(self, k)
                                                        bn = 9
                                                        self.juz_sym[bn] = 1
                                                        ST8.back(self, k, bn)
                                                    else:
                                                        dm = 14
                                                        lr = self.r0
                                                        ST8.srk(self, lr, k, dm)
                                                else:
                                                    ST8.rem(self, k)
                                    else:
                                        if self.jud[14] <= self.der:
                                            if self.jud[15] > self.der:
                                                dm = 15
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

        # land calc.
        for lj in range(self.n):
            self.land_jud[lj] = int(self.land[lj]) ^ int(self.land_past[lj])
        if sum(self.land_jud) != 0:
            lp = np.nonzero(self.land_jud)
            land_ip = list(lp[0]+np.ones(lp[0].shape))
            land_str = "landList#"+str(int(land_ip[0]))+"#"+str(int(land_ip[1])) + \
                       "#"+str(int(land_ip[2]))+"#"+str(int(land_ip[3]))
            self.pla.append(land_str)
        else:
            land_str = " "
        for ld in range(self.n):
            self.land_past[ld] = self.land[ld]

        # attack calc
        for aj in range(self.n):
            self.attack_jud[aj] = int(self.attack[aj]) ^ int(self.attack_past[aj])
        if sum(self.attack_jud) != 0:
            ap = np.nonzero(self.attack_jud)
            attack_ip = list(ap[0]+np.ones(ap[0].shape))
            attack_str = "attackList#"+str(int(attack_ip[0]))+"#"+str(int(attack_ip[1])) + \
                         "#"+str(int(attack_ip[2]))+"#"+str(int(attack_ip[3]))
            self.pat.append(attack_str)
        else:
            attack_str = " "
        for ak in range(self.n):
            self.attack_past[ak] = self.attack[ak]

        for q in range(self.tN):
            if self.cou[q] != 0:
                self.jud[q] = abs(self.ter[q]) / self.cou[q]
        for p in range(self.rn):
            if self.rou[p] != 0:
                self.juz[p] = abs(self.jz[p]) / self.rou[p]

        for ip in range(self.n):
            f_ip = int(self.ic[0, ip])
            t_ip = int(self.ic[1, ip])
            self.vxy[0, t_ip] = self.vx_t[f_ip]
            self.vxy[1, t_ip] = self.vy_t[f_ip]
            self.vxy[2, t_ip] = self.pvz[f_ip]

        return self.vxy, land_str, attack_str

    def attack(self, k):
        a_ip = int(self.ic[1, k])
        self.attack[a_ip] = 1

    def land(self, k):
        l_ip = int(self.ic[1, k])
        self.land[l_ip] = 1

    def back(self, k, bm):
        self.rou[bm] += 1
        if self.juz[bm] > self.der and self.juz_sym[bm] == 1:
            if self.pz[k] < self.zt - self.der:
                if self.pvz[k] < self.v_up:
                    az = self.a_bri
                else:
                    az = 0
            else:
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

            self.ct[bm] += 1
            self.ox[bm] += self.px[k]
            self.oy[bm] += self.py[k]

            sxx, syy = self.sx, self.sy
            self.dis_te[bm] = np.sqrt((self.tar_x[bm] - sxx) ** 2 + (self.tar_y[bm] - syy) ** 2)
            self.ver_xx[bm] = (self.tar_x[bm] - sxx) / self.dis_te[bm]
            self.ver_yy[bm] = (self.tar_y[bm] - syy) / self.dis_te[bm]

            if abs(self.oxx[bm] - self.tar_x[bm]) <= self.der or abs(self.oyy[bm] - self.tar_y[bm]) <= self.der:
                if self.vp_x[k] == 0 and self.vp_y[k] == 0:
                    ST8.land(self, k)
                    self.juz_sym[bm] = 0
                self.vp_x[k], self.vp_y[k] = 0, 0
                self.land_sym = 1
            else:
                self.vp_x[k] = self.v_ret * self.ver_xx[bm]
                self.vp_y[k] = self.v_ret * self.ver_yy[bm]

            if self.ct[bm] % self.sn == 0:
                if self.land_sym == 0:
                    self.oxx[bm] = self.ox[bm] / self.rou[bm]
                    self.oyy[bm] = self.oy[bm] / self.rou[bm]
                self.dis_rt[bm] = np.sqrt((self.sx - self.oxx[bm]) ** 2 + (self.sy - self.oyy[bm]) ** 2)

            if self.dis_rt[bm] > self.r0+1:
                self.nxs[bm] = 1

            self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
            self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt
            self.vx_t[k] = round(self.vp_x[k], 2)
            self.vy_t[k] = round(self.vp_y[k], 2)

    def srk(self, lr, k, dm):
        self.dis[k] = np.sqrt((self.oex - self.px[k]) ** 2 + (self.oey - self.py[k]) ** 2)
        self.err[k] = lr - self.dis[k]
        self.ver_xi[k] = (self.oex - self.px[k]) / self.dis[k]
        self.ver_yi[k] = (self.oey - self.py[k]) / self.dis[k]
        self.ter[dm] += self.err[k]
        self.cou[dm] += 1

        pn_ = 20
        dim_ = 1
        max_iter_ = 20
        x = self.px[k]
        y = self.py[k]
        vex = self.ver_xi[k]
        vey = self.ver_yi[k]
        vx = self.vpx_sum[k]
        vy = self.vpy_sum[k]
        v_sum = np.sqrt(vx**2 + vy**2)

        vt, ver_xt, ver_yt = ST8.angle(self, lr, k)

        if self.dis[k] > lr + self.der:
            if v_sum < self.v_bre:
                aa = self.a
            else:
                aa = 0
        else:
            aa = 0
            # p_pso = PSO(pn_, dim_, max_iter_, self.amx)
            # p_pso.p_init(self.r0, self.oex, self.oey, x, y, vx, vy, vex, vey, self.dt)
            # aa = p_pso.p_iter(self.r0, self.oex, self.oey, x, y, vx, vy, vex, vey, self.dt)

        vxx = self.ver_xi[k] * aa
        vyy = self.ver_yi[k] * aa
        self.n_px[k] = self.px[k] + vt * ver_xt * self.dt + self.vp_x[k] * self.dt + 0.5 * vxx * self.dt ** 2
        self.n_py[k] = self.py[k] + vt * ver_yt * self.dt + self.vp_y[k] * self.dt + 0.5 * vyy * self.dt ** 2
        self.vpx_sum[k] = self.vpx_sum[k] + vxx * self.dt
        self.vpy_sum[k] = self.vpy_sum[k] + vyy * self.dt

        if (self.dis[k] - lr) > self.der:
            self.vp_x[k] = self.vpx_sum[k] + self.ve * self.ver_ex
            self.vp_y[k] = self.vpy_sum[k] + self.ve * self.ver_ey
        else:
            self.vp_x[k] = self.ve * self.ver_ex
            self.vp_y[k] = self.ve * self.ver_ey

        self.vx_t[k] = self.vp_x[k] + vt * ver_xt
        self.vy_t[k] = self.vp_y[k] + vt * ver_yt

    def angle(self, lr, k):
        # 夹角均分机制
        if lr == self.r0:
            index = list([m, xx.index(k)] for m, xx in enumerate(self.g4) if k in xx)
            lin = self.g4[index[0][0]][index[0][1] - 1]
            uin = self.g4[index[0][0]][(index[0][1] + 1) % len(self.g4[index[0][0]])]
        elif lr == self.r1:
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

    def rem(self, k):
        self.n_px[k] = self.px[k] + self.vp_x[k] * self.dt
        self.n_py[k] = self.py[k] + self.vp_y[k] * self.dt
        self.vp_x[k] = self.ve * self.ver_ex
        self.vp_y[k] = self.ve * self.ver_ey
        self.vx_t[k] = self.vp_x[k]
        self.vy_t[k] = self.vp_y[k]