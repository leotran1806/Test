# robot GIẢ LẬP đi từ A -> B trên vùng đã phủ (đặc tả mục 11)
# dùng để test đối đầu, nhường đường, khóa, ngõ cụt
# laQuayLui = True: dich là c_b của 1 cặp quay lui -> mode BACKTRACK, goal = dich,
#                   tới nơi thì bước vào c_u của firstPairAt(dich) rồi rảnh
# laQuayLui = False: mode RETURN (giống quay về q_resume), goal = None
# nhận ABORT / GOAL_TAKEN / GOAL_DEAD -> bỏ cuộc (rảnh); YIELDED / OCCUPIED+replan -> lập lại đường

from ..protocol import (Kind, Mode, Reason, Intent, makeStayIntent, getStepAngle)
from ..grid_utils import bfsPathTo, pathLength, normalizeAngle, turnAngle


class GoalRobot:
    def __init__(self, robotId, shared, dich, laQuayLui = False, goc = 0.0):
        self.robotId = robotId
        self.shared = shared
        self.pos = shared.positions[robotId]
        self.dich = tuple(dich)
        self.laQuayLui = laQuayLui
        self.goc = goc
        self.path = [(self.pos[0], self.pos[1], self.goc)]
        self.duongDi = []

        self.daToi = False        # đã tới đích
        self.boCuoc = False       # đã bỏ cuộc (ABORT, GOAL_TAKEN, GOAL_DEAD, ko có đường)
        self.soLanBiDoi = 0       # số lần bị dời (forceStep)

    def updatePose(self, o):
        gocMoi = getStepAngle(self.pos, o)
        if abs(normalizeAngle(gocMoi - self.goc)) > 1e-9:
            self.path.append((self.pos[0], self.pos[1], gocMoi))
        self.goc = gocMoi
        self.pos = o
        self.path.append((o[0], o[1], gocMoi))

    def forceStep(self, o):
        self.soLanBiDoi = self.soLanBiDoi + 1
        self.updatePose(o)

    def idleIntent(self):
        return makeStayIntent(self.robotId, Mode.WAIT, idle = True)

    def propose(self):
        if self.daToi or self.boCuoc:
            return self.idleIntent()
        modeDi = Mode.BACKTRACK if self.laQuayLui else Mode.RETURN
        goal = self.dich if self.laQuayLui else None

        if self.pos == self.dich:
            if not self.laQuayLui:
                self.daToi = True
                return self.idleIntent()
            cap = self.shared.firstPairAt(self.dich)
            if cap is None:
                self.daToi = True
                return self.idleIntent()
            c_u = cap[1]
            gocQuay = turnAngle(getStepAngle(self.pos, c_u), self.goc)
            return Intent(robotId = self.robotId, kind = Kind.UNCOVERED, target = c_u, mode = Mode.BACKTRACK,
                          v = 1.0, theta = gocQuay, goal = goal, path = (c_u,))

        if len(self.duongDi) == 0:
            duongDi = bfsPathTo(self.pos, {self.dich}, self.shared.traversable())
            if duongDi is None:
                self.boCuoc = True
                return self.idleIntent()
            self.duongDi = list(duongDi)
        oKe = self.duongDi[0]
        gocQuay = turnAngle(getStepAngle(self.pos, oKe), self.goc)
        return Intent(robotId = self.robotId, kind = Kind.TRAVERSE, target = oKe, mode = modeDi,
                      v = pathLength(self.pos, self.duongDi), theta = gocQuay,
                      goal = goal, path = tuple(self.duongDi))

    def handleResponse(self, it, resp):
        if resp.ok and it.kind != Kind.STAY:
            self.updatePose(it.target)
            if it.kind == Kind.UNCOVERED:
                self.daToi = True
            elif len(self.duongDi) > 0 and self.duongDi[0] == it.target:
                self.duongDi.pop(0)
            else:
                self.duongDi = []
        if resp.reason in (Reason.ABORT, Reason.GOAL_TAKEN, Reason.GOAL_DEAD):
            self.boCuoc = True
        elif resp.replan:
            self.duongDi = []

    def run(self):
        while True:
            it = self.propose()
            resp = yield it
            self.handleResponse(it, resp)
