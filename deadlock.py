# chống kẹt (đặc tả mục 7) - phần ĐX còn để trống (IV.4), đóng góp của nhóm
# phát hiện: wait[k] tăng 1 mỗi bước robot k bị từ chối OCCUPIED, blocker[k] = robot đang chiếm ô k muốn vào
# xử lý khi wait[k] >= deadlockK, j = blocker[k]:
#   R0  : j đã hỏng -> k lập lại đường (OCCUPIED, replan)
#   R1  : có ô né -> robot né được dời bằng bước forced + KHÓA với robot kia
#   R2a : ko ai né được, j rảnh, k đang giữ đích g -> k nhả g (tabu), chuyển g cho j, k nhận ABORT
#   R2b : còn lại -> k nhả đích (tabu), nhận ABORT
# chỉ xử lý khi j hỏng, j đứng yên (STAY hoặc bị LOCKED), hoặc k nằm trong CHU TRÌNH chặn nhau
# (đối đầu = chu trình độ dài 2). j đang tự di chuyển -> chờ thêm
# note bổ sung so với đặc tả (đã ghi trong README):
#   - "đối đầu" mở rộng thành chu trình bất kỳ (3 robot chặn vòng tròn cũng kẹt vĩnh viễn)
#   - j bị từ chối LOCKED cũng coi là đứng yên (ko thì j và robot khóa nó có thể chờ nhau tới hết lockMaxSteps)

from .protocol import (Kind, Mode, Reason, Intent, RESP_OCCUPIED_REPLAN, RESP_ABORT, RESP_YIELDED)
from .grid_utils import bfsPathTo, pathFootprint
from .conflict import moveKey


class DeadlockManager:
    def __init__(self, shared, config):
        self.shared = shared
        self.config = config

        self.wait = {}               # {id: số bước liên tiếp bị OCCUPIED}
        self.blocker = {}            # {id: robot đang chặn}, chỉ của bước vừa xét
        self.yieldPlan = {}          # {id: [ô, ...]} robot đang bị dời (nhường đường)
        self.forcedBlocked = {}      # {id: số bước forced bị chặn liên tiếp}
        self.locks = {}              # {y: (o, bước bắt đầu)}: y đã nhường cho o
        self.pending = {}            # {id: Response} ghi đè response ở lần resume tới

        self.thongKe = {'R0': 0, 'R1': 0, 'R2a': 0, 'R2b': 0, 'huy_nhuong_duong': 0}

    def isYielding(self, k):
        return k in self.yieldPlan

    # ---------------- thực hiện nhường đường (mục 7.3) ----------------

    # bước forced tới ô tiếp theo của kế hoạch né, ưu tiên cao nhất (moveKey)
    def forcedIntent(self, y):
        keHoach = self.yieldPlan[y]
        return Intent(robotId = y, kind = Kind.TRAVERSE, target = keHoach[0], mode = Mode.WAIT,
                      v = float(len(keHoach)), theta = 0.0, goal = None,
                      path = tuple(keHoach), idle = False, forced = True)

    # gọi ở pha 4 cho mỗi robot đang bị dời
    def onForcedResult(self, y, chapNhan):
        if chapNhan:
            self.yieldPlan[y].pop(0)
            self.forcedBlocked[y] = 0
            if len(self.yieldPlan[y]) == 0:
                del self.yieldPlan[y]
                self.pending[y] = RESP_YIELDED
        else:
            self.forcedBlocked[y] = self.forcedBlocked.get(y, 0) + 1
            if self.forcedBlocked[y] >= self.config.deadlockK:
                del self.yieldPlan[y]
                self.forcedBlocked[y] = 0
                self.pending[y] = RESP_YIELDED
                self.thongKe['huy_nhuong_duong'] = self.thongKe['huy_nhuong_duong'] + 1

    # ---------------- khóa nhường đường (mục 7.4) ----------------

    # dấu chân đường sắp đi của robot o: target + path (+ ô cạnh bước chéo nếu strictDiagonal)
    def _footprint(self, o, it):
        if it is None or it.kind == Kind.STAY:
            return set()
        duongDi = list(it.path) if len(it.path) > 0 else [it.target]
        return pathFootprint(self.shared.positions[o], duongDi, self.config.strictDiagonal) | {it.target}

    # ô bị khóa với y = target + path HIỆN TẠI của o (chỉ khi o đang TRAVERSE)
    def lockedCells(self, intents):
        ketQua = {}
        for y, (o, _) in self.locks.items():
            it = intents.get(o)
            if it is None or it.kind != Kind.TRAVERSE:
                continue
            ketQua[y] = self._footprint(o, it)
        return ketQua

    # gọi sau pha 6: mở khóa khi o ko còn TRAVERSE, 1 bên hỏng, hoặc quá lockMaxSteps
    def releaseLocks(self, intents):
        for y in list(self.locks):
            o, tBatDau = self.locks[y]
            if y in self.shared.failed or o in self.shared.failed:
                del self.locks[y]
            elif self.shared.t - tBatDau > self.config.lockMaxSteps:
                del self.locks[y]
            elif o in self.yieldPlan:
                continue                                  # o đang bị dời: vẫn là TRAVERSE
            else:
                it = intents.get(o)
                if it is None or it.kind != Kind.TRAVERSE:
                    del self.locks[y]

    # ---------------- phát hiện (mục 7.1) ----------------

    # gọi ở pha 5 với Intent đã thực thi và kết quả resolveMoves của bước này
    def updateWaits(self, intents, ketQuaMoves):
        self.blocker = {}
        for k in intents:
            it = intents[k]
            kq = ketQuaMoves.get(k)
            if not it.forced and kq is not None and not kq[0] and kq[1] == Reason.OCCUPIED:
                self.wait[k] = self.wait.get(k, 0) + 1
                self.blocker[k] = kq[2]
            else:
                self.wait[k] = 0

    # chu trình chặn nhau chứa k: [k, blocker[k], blocker[blocker[k]], ...] quay về k, ko có -> None
    def _findCycle(self, k):
        chuoi = [k]
        x = self.blocker.get(k)
        while x is not None and x not in chuoi:
            chuoi.append(x)
            x = self.blocker.get(x)
        if x == k and len(chuoi) >= 2:
            return chuoi
        return None

    # R1: tìm kế hoạch né ngắn nhất trong các ứng viên (y né cho o)
    # ô CẤM = vị trí o + path của o + target của o (+ ô cạnh bước chéo nếu strictDiagonal)
    # y ko đứng trên ô cấm -> bỏ qua ứng viên
    # BFS từ y trên traversable() trừ vị trí các robot khác, tới ô gần nhất ko bị cấm
    # hòa: robot rảnh né trước, rồi ID lớn hơn né
    def _findBestDodge(self, danhSachUngVien, intents):
        shared = self.shared
        totNhat = None
        for (y, o) in danhSachUngVien:
            if y in shared.failed or o in shared.failed:
                continue
            itO = intents.get(o)
            oCam = {shared.positions[o]} | self._footprint(o, itO)
            viTriY = shared.positions[y]
            if viTriY not in oCam:
                continue
            viTriKhac = {shared.positions[x] for x in shared.positions if x != y}
            vungDuocDi = shared.traversable() - viTriKhac
            # góc: mặc định chỉ cần ô cạnh đã phủ (đúng hợp đồng); strictDiagonal thì ô cạnh còn phải ko có robot
            vungGoc = vungDuocDi if self.config.strictDiagonal else shared.traversable()
            duongDi = bfsPathTo(viTriY, lambda c: c not in oCam, vungDuocDi, vungGoc)
            if duongDi is None or len(duongDi) == 0:
                continue
            itY = intents.get(y)
            yRanh = itY is not None and itY.kind == Kind.STAY
            khoa = (len(duongDi), 0 if yRanh else 1, -y)
            if totNhat is None or khoa < totNhat[0]:
                totNhat = (khoa, y, o, duongDi)
        if totNhat is None:
            return None
        return totNhat[1], totNhat[2], totNhat[3]

    # ---------------- thang xử lý (mục 7.2) ----------------

    # gọi ở pha 5 sau updateWaits
    def resolve(self, intents, ketQuaMoves):
        shared = self.shared
        K = self.config.deadlockK
        daXuLy = set()

        danhSachKet = [k for k in self.blocker
                       if self.wait.get(k, 0) >= K and k not in shared.failed and k not in self.yieldPlan]
        # robot ưu tiên THẤP xử lý trước -> khi phải bỏ đích thì robot ưu tiên thấp bỏ
        danhSachKet.sort(key = lambda k: moveKey(intents[k], self.config.thetaEps), reverse = True)

        for k in danhSachKet:
            if k in daXuLy:
                continue
            j = self.blocker[k]

            # R0: bị robot HỎNG chặn
            if j in shared.failed:
                self.pending[k] = RESP_OCCUPIED_REPLAN
                self.wait[k] = 0
                daXuLy.add(k)
                self.thongKe['R0'] = self.thongKe['R0'] + 1
                continue

            if j in daXuLy or j in self.yieldPlan:
                continue
            itJ = intents.get(j)
            kqJ = ketQuaMoves.get(j)
            jDungYen = (itJ is not None and not itJ.forced and
                        (itJ.kind == Kind.STAY or (kqJ is not None and kqJ[1] == Reason.LOCKED)))
            chuTrinh = None if jDungYen else self._findCycle(k)
            if not jDungYen and chuTrinh is None:
                continue                                   # j đang tự di chuyển -> chờ thêm
            if chuTrinh is not None and any(x in daXuLy or x in self.yieldPlan for x in chuTrinh):
                continue

            # R1: nhường đường
            if jDungYen:
                danhSachUngVien = [(j, k)]
            else:
                danhSachUngVien = []
                for i in range(len(chuTrinh)):
                    y = chuTrinh[(i + 1) % len(chuTrinh)]  # y chặn chuTrinh[i]
                    danhSachUngVien.append((y, chuTrinh[i]))
            keHoach = self._findBestDodge(danhSachUngVien, intents)
            if keHoach is not None:
                y, o, duongDi = keHoach
                if ketQuaMoves.get(y, (False,))[0]:
                    raise RuntimeError("robot %s vua di chuyen ma lai bi chon nhuong duong" % (y,))
                self.yieldPlan[y] = list(duongDi)
                self.forcedBlocked[y] = 0
                self.locks[y] = (o, shared.t)
                self.pending.pop(y, None)
                self.wait[k] = 0
                daXuLy.update({k, y, o})
                self.thongKe['R1'] = self.thongKe['R1'] + 1
                continue

            # R2a: chuyển đích cho robot rảnh đang chắn đường
            g = shared.reserved.get(k)
            if jDungYen and itJ.idle and g is not None:
                shared.release(k)
                shared.addTabu(k, g, self.config.tabuSteps)
                shared.reserve(j, g)
                self.pending[k] = RESP_ABORT
                self.wait[k] = 0
                daXuLy.update({k, j})
                self.thongKe['R2a'] = self.thongKe['R2a'] + 1
                continue

            # R2b: bỏ đích
            if g is not None:
                shared.release(k)
                shared.addTabu(k, g, self.config.tabuSteps)
            self.pending[k] = RESP_ABORT
            self.wait[k] = 0
            daXuLy.add(k)
            self.thongKe['R2b'] = self.thongKe['R2b'] + 1

    # ---------------- robot hỏng (mục 8.3) ----------------

    # hủy kế hoạch né, khóa, pending liên quan tới k
    def cancel(self, k):
        self.yieldPlan.pop(k, None)
        self.forcedBlocked.pop(k, None)
        self.pending.pop(k, None)
        self.wait.pop(k, None)
        self.blocker.pop(k, None)
        self.locks.pop(k, None)
        for y in list(self.locks):
            if self.locks[y][0] == k:
                del self.locks[y]
                if y in self.yieldPlan:
                    del self.yieldPlan[y]
                    self.pending[y] = RESP_YIELDED
