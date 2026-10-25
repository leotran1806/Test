# robot GIẢ LẬP tuân thủ đúng 10 quy tắc hợp đồng (protocol.py, đặc tả mục 3.5 + 11)
# dùng để test tầng điều phối KHI CHƯA CÓ tầng robot ACM, đồng thời là VÍ DỤ MẪU cho acm_robot.py
# hành vi:
#   quét: bước vào ô kề 4 hướng chưa phủ (ưu tiên giữ hướng, rồi góc quay nhỏ nhất)
#   ghi cặp: mọi ô kề 4 hướng trống chưa phủ -> cặp (pos, ô đó)
#   quay lui: BFS trên shared.traversable() tới c_b gần nhất trong planningPairs, đi từng ô,
#             tới nơi thì bước vào c_u của firstPairAt(c_b)
# note:
# self.pos / self.goc: ô đang đứng + góc hướng (radian, Đông = 0 giống ACM)
# self.path: list cấu hình (dòng, cột, góc) giống ACM_LOS_FINAL.py -> tính L(P), NT(P)
# self.duongDi: các ô còn lại phải đi trên vùng đã phủ; self.dich: c_b đang nhắm tới

import math

from ..protocol import (Kind, Mode, Intent, makeStayIntent, getStepAngle)
from ..grid_utils import getNeighbors4, isFreeCell, bfsPathTo, pathLength, normalizeAngle, turnAngle

# giống ACM_LOS_FINAL.py
BUOC_DI = {
    'N': (-1, 0),
    'E': (0, 1),
    'S': (1, 0),
    'W': (0, -1),
}
GOC_CUA_HUONG = {}
for _h in BUOC_DI:
    GOC_CUA_HUONG[_h] = math.atan2(BUOC_DI[_h][0], BUOC_DI[_h][1])
# thứ tự xét khi hòa góc quay
THU_TU_HUONG = ['N', 'E', 'S', 'W']


class SimpleRobot:
    # robotId: ID; grid: bản đồ (giả lập cảm biến); shared: SharedState
    # huongBanDau: 'N'/'E'/'S'/'W'; tamNhin: số ô tối đa khi đếm v (giống floor(rho))
    # rng (chỉ để stress test): nếu có thì chọn hướng quét / đích quay lui NGẪU NHIÊN thay vì tốt nhất
    #     -> tạo nhiều kiểu xung đột hơn cho tầng điều phối, vẫn đúng hợp đồng
    def __init__(self, robotId, grid, shared, huongBanDau = 'N', tamNhin = 3, rng = None):
        self.robotId = robotId
        self.grid = grid
        self.shared = shared
        self.pos = shared.positions[robotId]
        self.goc = GOC_CUA_HUONG[huongBanDau]
        self.tamNhin = tamNhin
        self.rng = rng
        self.path = [(self.pos[0], self.pos[1], self.goc)]

        self.duongDi = []
        self.dich = None

        # bộ đếm
        self.soLanQuayLui = 0
        self.soLanLapLai = 0

    # ---------------- cảm biến giả lập ----------------

    def isFreeUncovered(self, o):
        return isFreeCell(self.grid, o) and o not in self.shared.covered

    # đếm số ô trống chưa phủ liên tiếp từ viTri theo bước buoc (tối đa tamNhin) -> v
    def measureRun(self, viTri, buoc):
        soO = 0
        o = viTri
        danhSachO = []
        for i in range(self.tamNhin):
            o = (o[0] + buoc[0], o[1] + buoc[1])
            if self.isFreeUncovered(o):
                soO = soO + 1
                danhSachO.append(o)
            else:
                break
        return soO, danhSachO

    # ghi cặp quay lui tại ô đang đứng (quy tắc 7: gọi SAU khi bước được chấp nhận)
    def findBacktrackPairs(self):
        capMoi = []
        for oKe in getNeighbors4(self.pos):
            if self.isFreeUncovered(oKe):
                capMoi.append((self.pos, oKe))
        if len(capMoi) > 0:
            self.shared.addPairs(self.robotId, capMoi)

    # ---------------- di chuyển riêng (KHÔNG đụng covered / cặp - quy tắc 4) ----------------

    def updatePose(self, o):
        gocMoi = getStepAngle(self.pos, o)
        if abs(normalizeAngle(gocMoi - self.goc)) > 1e-9:
            self.path.append((self.pos[0], self.pos[1], gocMoi))
        self.goc = gocMoi
        self.pos = o
        self.path.append((o[0], o[1], gocMoi))

    # quy tắc 9: điều phối dời robot, chỉ cập nhật pos, heading, đường đi riêng, KHÔNG yield
    def forceStep(self, o):
        self.updatePose(o)

    def clearPlan(self):
        self.duongDi = []
        self.dich = None

    # ---------------- đề xuất ----------------

    # bước quét: ô kề 4 hướng chưa phủ, ưu tiên góc quay nhỏ nhất (giữ hướng trước)
    def proposeCover(self):
        danhSach = []
        for i, ten in enumerate(THU_TU_HUONG):
            buoc = BUOC_DI[ten]
            oKe = (self.pos[0] + buoc[0], self.pos[1] + buoc[1])
            if self.isFreeUncovered(oKe):
                gocQuay = turnAngle(GOC_CUA_HUONG[ten], self.goc)
                danhSach.append((round(gocQuay, 9), i, ten, oKe))
        if len(danhSach) == 0:
            return None
        danhSach.sort()
        if self.rng is not None and self.rng.random() < 0.3:
            gocQuay, _, ten, oKe = self.rng.choice(danhSach)
        else:
            gocQuay, _, ten, oKe = danhSach[0]
        soO, danhSachO = self.measureRun(self.pos, BUOC_DI[ten])
        return Intent(robotId = self.robotId, kind = Kind.UNCOVERED, target = oKe, mode = Mode.COVER,
                      v = float(soO), theta = gocQuay, goal = None, path = tuple(danhSachO))

    # bước vào nhánh c_b -> c_u (43)(44), goal vẫn là c_b
    def proposeEnterBranch(self, c_u):
        buoc = (c_u[0] - self.pos[0], c_u[1] - self.pos[1])
        soO, danhSachO = self.measureRun(self.pos, buoc)
        gocQuay = turnAngle(getStepAngle(self.pos, c_u), self.goc)
        return Intent(robotId = self.robotId, kind = Kind.UNCOVERED, target = c_u, mode = Mode.BACKTRACK,
                      v = float(max(soO, 1)), theta = gocQuay, goal = self.dich, path = tuple(danhSachO))

    # bước đi trên vùng đã phủ: v = độ dài Euclid còn lại, theta = góc phải quay
    def proposeTraverse(self):
        oKe = self.duongDi[0]
        gocQuay = turnAngle(getStepAngle(self.pos, oKe), self.goc)
        return Intent(robotId = self.robotId, kind = Kind.TRAVERSE, target = oKe, mode = Mode.BACKTRACK,
                      v = pathLength(self.pos, self.duongDi), theta = gocQuay,
                      goal = self.dich, path = tuple(self.duongDi))

    def propose(self):
        shared = self.shared
        for _ in range(4):
            # 1. đang quay lui tới c_b
            if self.dich is not None:
                if self.dich in shared.failedCells or not shared.goalAlive(self.dich):
                    self.clearPlan()
                    continue
                if self.pos == self.dich:
                    cap = shared.firstPairAt(self.dich)
                    if cap is None:
                        self.clearPlan()
                        continue
                    return self.proposeEnterBranch(cap[1])
                if len(self.duongDi) == 0:
                    self.clearPlan()
                    continue
                return self.proposeTraverse()

            # 2. còn ô kề chưa phủ -> quét
            it = self.proposeCover()
            if it is not None:
                return it

            # 3. lập kế hoạch quay lui tới c_b gần nhất
            tapDich = {c_b for (c_b, c_u) in shared.planningPairs(self.robotId)}
            if len(tapDich) == 0:
                break
            duongDi = None
            if self.rng is not None and self.rng.random() < 0.5:
                duongDi = bfsPathTo(self.pos, {self.rng.choice(sorted(tapDich))}, shared.traversable())
            if duongDi is None:
                duongDi = bfsPathTo(self.pos, tapDich, shared.traversable())
            if duongDi is None:
                break
            self.soLanQuayLui = self.soLanQuayLui + 1
            self.duongDi = list(duongDi)
            self.dich = duongDi[-1] if len(duongDi) > 0 else self.pos
        # quy tắc 10: ko tới được đích nào -> STAY idle
        self.clearPlan()
        return makeStayIntent(self.robotId, Mode.WAIT, idle = True)

    # ---------------- xử lý phản hồi (bảng 3.4) ----------------

    def handleResponse(self, it, resp):
        if resp.ok and it.kind != Kind.STAY:
            self.updatePose(it.target)
            if it.kind == Kind.TRAVERSE:
                if len(self.duongDi) > 0 and self.duongDi[0] == it.target:
                    self.duongDi.pop(0)
                else:
                    self.clearPlan()
            elif it.mode == Mode.BACKTRACK:
                self.clearPlan()                 # vào nhánh xong -> quay lại quét
        if resp.replan:
            self.soLanLapLai = self.soLanLapLai + 1
            self.clearPlan()
        # X1_LOST / OCCUPIED / CROSSING / LOCKED (replan = False): giữ kế hoạch, lần sau đề xuất lại
        # với UNCOVERED bị X1_LOST: ô đó đã bị phủ nên propose() tự chọn hướng khác

    # ---------------- generator (quy tắc 1, 2) ----------------

    def run(self):
        while True:
            self.findBacktrackPairs()
            it = self.propose()
            resp = yield it
            self.handleResponse(it, resp)
