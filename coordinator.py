# vòng lặp điều phối một bước (đặc tả mục 5), khớp 5 bước flowchart ĐX
# step() gồm 7 pha:
#   PHA 1  thu đề xuất                          (ĐX Bước 1)
#   PHA 2  xung đột đích X4 + đặt chỗ           (ĐX Bước 2-4)  -> resolveGoals
#   PHA 3  xung đột di chuyển X1, X3/X2, X2'    (ĐX Bước 2-4)  -> resolveMoves
#   PHA 4  cập nhật đồng thời                   (ĐX Bước 5)
#   PHA 5  hậu xử lý: chống kẹt, soonCovered, bất biến
#   PHA 6  resume: robot tính đề xuất cho bước sau
#   PHA 7  kiểm tra dừng (M14)
# vì sao đúng (mục 5.2): giữa pha 6 bước t và pha 4 bước t+1 ko ai đổi covered, positions, đặt chỗ
# -> mọi robot đề xuất trên CÙNG một trạng thái (thay cho ảnh chụp đầu bước [UAV 9.2])

from .protocol import (Kind, Mode, Reason, ContractError, Response, isNeighbor8,
                       RESP_OK, RESP_X1_LOST, RESP_OCCUPIED, RESP_CROSSING, RESP_LOCKED)
from .config import CoordConfig
from .grid_utils import getFreeCells, getNeighbors4, bfsReach
from .conflict import validateIntent, resolveGoals, resolveMoves
from .deadlock import DeadlockManager
from . import invariants

# lý do bị từ chối di chuyển -> Response (replan = False)
PHAN_HOI_TU_CHOI = {
    Reason.X1_LOST: RESP_X1_LOST,
    Reason.OCCUPIED: RESP_OCCUPIED,
    Reason.CROSSING: RESP_CROSSING,
    Reason.LOCKED: RESP_LOCKED,
}


class Coordinator:
    # grid: bản đồ thật (chỉ dùng để kiểm tra hợp đồng + bất biến, KHÔNG dùng để ra quyết định)
    # shared: SharedState đã tạo với vị trí xuất phát
    # danhSachRobot: list robot tuân thủ RobotProtocol (có robotId, pos, run(), forceStep())
    def __init__(self, grid, shared, danhSachRobot, config = None):
        if config is None:
            config = CoordConfig()
        self.grid = grid
        self.shared = shared
        self.config = config
        self.robots = {}
        for robot in danhSachRobot:
            if robot.robotId in self.robots:
                raise ValueError("trung robotId %s" % (robot.robotId,))
            self.robots[robot.robotId] = robot
        if set(self.robots) != set(shared.positions):
            raise ValueError("danh sach robot khong khop shared.positions")
        for k, robot in self.robots.items():
            if tuple(robot.pos) != shared.positions[k]:
                raise ValueError("robot %s: pos %s khac shared.positions %s" % (k, robot.pos, shared.positions[k]))

        self.deadlock = DeadlockManager(shared, config)
        self.oTrong = getFreeCells(grid)
        for k, o in shared.positions.items():
            if o not in self.oTrong:
                raise ValueError("robot %s xuat phat tren vat can %s" % (k, o))

        self.gens = {}                 # {id: generator}
        self.intents = {}              # {id: Intent} đề xuất mới nhất (cho bước sau)
        self.daBatDau = False
        self.daDung = False
        self.coveredTruoc = None       # cho bất biến I3

        # bộ đếm
        self.thongKe = {
            'X1': 0, 'X2_cheo': 0, 'X3': 0, 'LOCKED': 0,
            'X4_GOAL_TAKEN': 0, 'GOAL_DEAD': 0,
            'cap_neo_lai': 0, 'robot_hong': 0,
        }
        self.soBuocDuocDi = {k: 0 for k in self.robots}     # số bước di chuyển được chấp nhận
        self.soBuocCho = {k: 0 for k in self.robots}        # số bước đứng yên (kể cả bị từ chối)

    # ---------------- khởi động ----------------

    def start(self):
        if self.daBatDau:
            return
        self.daBatDau = True
        for k in sorted(self.robots):
            gen = self.robots[k].run()
            self.gens[k] = gen
            try:
                it = next(gen)
            except StopIteration:
                raise ContractError("robot %s: generator ket thuc ngay khi bat dau" % (k,))
            validateIntent(it, k, self.shared, self.grid, self.config.thetaEps)
            self.intents[k] = it
            self.shared.modes[k] = it.mode
        if self.config.checkInvariants:
            invariants.check(self.shared, self.oTrong, None)
            self.coveredTruoc = set(self.shared.covered)
        self.daDung = self._checkTermination()

    # ---------------- một bước ----------------

    def step(self):
        if not self.daBatDau:
            self.start()
        if self.daDung:
            return
        shared = self.shared
        shared.t = shared.t + 1
        shared.clearExpiredTabu()
        dsSong = shared.liveRobots()

        # PHA 1: thu đề xuất
        intents = {}
        for k in dsSong:
            if self.deadlock.isYielding(k):
                intents[k] = self.deadlock.forcedIntent(k)
            else:
                if k in self.deadlock.pending:
                    raise RuntimeError("robot %s con pending o pha 1 -> Intent da cu" % (k,))
                intents[k] = self.intents[k]

        # PHA 2: X4 + đặt chỗ
        intents, phanHoiDich = resolveGoals(intents, shared, self.config)
        for k, resp in phanHoiDich.items():
            if resp.reason == Reason.GOAL_TAKEN:
                self.thongKe['X4_GOAL_TAKEN'] = self.thongKe['X4_GOAL_TAKEN'] + 1
            else:
                self.thongKe['GOAL_DEAD'] = self.thongKe['GOAL_DEAD'] + 1

        # PHA 3: X1, X3/X2, X2', khóa
        khoaO = self.deadlock.lockedCells(intents)
        ketQuaMoves = resolveMoves(intents, shared, khoaO, self.config)

        # PHA 4: cập nhật đồng thời (theo thứ tự id)
        phanHoi = {}
        for k in dsSong:
            it = intents[k]
            kq = ketQuaMoves.get(k)
            if kq is not None and kq[0]:
                o = it.target
                if not isNeighbor8(shared.positions[k], o):
                    raise RuntimeError("robot %s nhay %s -> %s (Intent cu?)" % (k, shared.positions[k], o))
                shared.positions[k] = o
                if it.kind == Kind.UNCOVERED:                          # (18)(44)(M9)
                    shared.covered.add(o)
                    shared.coveredBy[k].add(o)
                    shared.deletePairsWithCu(o)
                if it.forced:
                    self.robots[k].forceStep(o)
                    self.deadlock.onForcedResult(k, True)
                self.soBuocDuocDi[k] = self.soBuocDuocDi[k] + 1
                phanHoi[k] = RESP_OK
            else:
                self.soBuocCho[k] = self.soBuocCho[k] + 1
                if kq is not None:
                    self._demTuChoi(kq[1])
                    if it.forced:
                        self.deadlock.onForcedResult(k, False)
                    phanHoi[k] = PHAN_HOI_TU_CHOI[kq[1]]
                elif k in phanHoiDich:
                    phanHoi[k] = phanHoiDich[k]
                else:
                    phanHoi[k] = RESP_OK                               # STAY như dự định
            shared.modes[k] = it.mode

        # PHA 5: hậu xử lý
        self.deadlock.updateWaits(intents, ketQuaMoves)
        self.deadlock.resolve(intents, ketQuaMoves)
        if self.config.filterNearCover:
            shared.soonCovered = self._computeSoonCovered(intents, ketQuaMoves)
        else:
            shared.soonCovered = set()
        if self.config.checkInvariants:
            invariants.check(shared, self.oTrong, self.coveredTruoc)
            self.coveredTruoc = set(shared.covered)

        # PHA 6: resume robot (sống, ko đang bị dời), theo thứ tự id
        for k in dsSong:
            if self.deadlock.isYielding(k):
                continue
            resp = self.deadlock.pending.pop(k, None)
            if resp is None:
                resp = phanHoi[k]
            self._resumeRobot(k, resp)
        self.deadlock.releaseLocks(self.intents)

        # PHA 7: kiểm tra dừng
        self.daDung = self._checkTermination()

    # gửi Response cho robot k, nhận Intent cho bước sau, kiểm tra hợp đồng
    def _resumeRobot(self, k, resp):
        try:
            it = self.gens[k].send(resp)
        except StopIteration:
            raise ContractError("robot %s: generator ket thuc (robot khong duoc return)" % (k,))
        validateIntent(it, k, self.shared, self.grid, self.config.thetaEps)
        self.intents[k] = it

    def _demTuChoi(self, lyDo):
        if lyDo == Reason.X1_LOST:
            self.thongKe['X1'] = self.thongKe['X1'] + 1
        elif lyDo == Reason.CROSSING:
            self.thongKe['X2_cheo'] = self.thongKe['X2_cheo'] + 1
        elif lyDo == Reason.OCCUPIED:
            self.thongKe['X3'] = self.thongKe['X3'] + 1
        elif lyDo == Reason.LOCKED:
            self.thongKe['LOCKED'] = self.thongKe['LOCKED'] + 1

    # (Mức 2) ô sắp được phủ: phần còn lại của đoạn thẳng của robot đang quét vừa đi được
    def _computeSoonCovered(self, intents, ketQuaMoves):
        ketQua = set()
        for k, it in intents.items():
            kq = ketQuaMoves.get(k)
            if it.kind != Kind.UNCOVERED or it.mode not in (Mode.COVER, Mode.HOLE):
                continue
            if kq is None or not kq[0]:
                continue
            for o in it.path:
                if o not in self.shared.covered:
                    ketQua.add(o)
        return ketQua

    # ---------------- chạy cả thuật toán ----------------

    # failAt = {id: b}: robot id hỏng ngay sau khi bước thứ b chạy xong (b = 0: hỏng trước bước đầu)
    def run(self, failAt = None):
        if failAt is None:
            failAt = {}
        for k in failAt:
            if k not in self.robots:
                raise ValueError("failAt co robot %s khong ton tai" % (k,))
        self.start()
        while True:
            for k in sorted(failAt):
                if failAt[k] == self.shared.t and k not in self.shared.failed:
                    self.failRobot(k)
            if self.daDung:
                break
            if self.shared.t >= self.config.maxSteps:
                raise RuntimeError("vuot maxSteps = %d" % (self.config.maxSteps,))
            self.step()
        return self.getResult()

    # ---------------- robot hỏng (mục 8.3), gọi GIỮA hai bước ----------------

    def failRobot(self, k):
        shared = self.shared
        if k in shared.failed:
            return
        if not self.daBatDau:
            self.start()
        o = shared.positions[k]
        shared.failed.add(k)
        shared.failedCells.add(o)
        shared.modes[k] = Mode.FAILED
        shared.release(k)
        self.deadlock.cancel(k)
        gen = self.gens.pop(k, None)
        if gen is not None:
            gen.close()
        self.intents.pop(k, None)
        self.thongKe['robot_hong'] = self.thongKe['robot_hong'] + 1

        # bổ sung: cặp có c_b ngay ô robot hỏng -> neo lại vào ô kề cạnh khác của c_u (nếu đã phủ)
        # ko đánh dấu cặp mất ở đây (mục 8.2)
        capMoi = []
        for (c_b, c_u) in shared.activePairs():
            if c_b != o:
                continue
            for c in getNeighbors4(c_u):
                if c != o and c in shared.covered and c not in shared.failedCells:
                    capMoi.append((c, c_u))
        self.thongKe['cap_neo_lai'] = self.thongKe['cap_neo_lai'] + len(shared.addPairs(None, capMoi))

        # robot đang né cho k bị hủy kế hoạch -> có pending YIELDED, Intent cũ của nó đã CŨ (vị trí đã đổi)
        # -> resume ngay để lấy Intent mới (giống pha 6), ko được dùng Intent cũ ở pha 1 bước sau
        for y in sorted(self.deadlock.pending):
            if y not in shared.failed and not self.deadlock.isYielding(y):
                self._resumeRobot(y, self.deadlock.pending.pop(y))
        self.deadlock.releaseLocks(self.intents)

        self.daDung = self._checkTermination()

    # cặp mở có c_b KHÔNG tới được từ robot sống (BFS 8 hướng ko cắt góc trên traversable())
    def unreachablePairs(self, oToiDuoc = None):
        if oToiDuoc is None:
            oToiDuoc = self._reachableCells()
        return [cap for cap in self.shared.activePairs() if cap[0] not in oToiDuoc]

    def _reachableCells(self):
        shared = self.shared
        viTriSong = [shared.positions[k] for k in shared.liveRobots()]
        return bfsReach(viTriSong, shared.traversable())

    # điều kiện dừng (mục 8.1, M14)
    # dừng <=> ko robot nào đang bị dời, ko còn pending
    #          và mọi robot SỐNG đề xuất idle = True
    #          và mọi cặp còn mở đều KHÔNG tới được
    # cặp mất CHỈ kết luận ở đây (mục 8.2)
    def _checkTermination(self):
        shared = self.shared
        if len(self.deadlock.yieldPlan) > 0 or len(self.deadlock.pending) > 0:
            return False
        for k in shared.liveRobots():
            if not self.intents[k].idle:
                return False

        oToiDuoc = self._reachableCells()
        capKhongToi = self.unreachablePairs(oToiDuoc)
        if len(capKhongToi) < len(shared.activePairs()):
            return False                    # còn cặp tới được (vd đang bị tabu) -> chạy tiếp

        # bổ sung: trước khi kết luận mất, thử neo lại cặp vào ô đã phủ tới được kề cạnh c_u
        capMoi = []
        for (c_b, c_u) in capKhongToi:
            for c in getNeighbors4(c_u):
                if c in oToiDuoc:
                    capMoi.append((c, c_u))
        soCapThem = len(shared.addPairs(None, capMoi))
        if soCapThem > 0:
            self.thongKe['cap_neo_lai'] = self.thongKe['cap_neo_lai'] + soCapThem
            return False

        for cap in capKhongToi:
            shared.lostPairs.add(cap)
        for k in list(shared.reserved):
            shared.release(k)
        return True

    # ---------------- kết quả ----------------

    def getResult(self):
        shared = self.shared
        return {
            'steps': shared.t,
            'covered': set(shared.covered),
            'covered_by': {k: set(v) for k, v in shared.coveredBy.items()},
            'lost_pairs': set(shared.lostPairs),
            'failed': set(shared.failed),
            'stats': dict(self.thongKe),
            'deadlock_stats': dict(self.deadlock.thongKe),
            'moves': dict(self.soBuocDuocDi),
            'waits': dict(self.soBuocCho),
        }
