# HỢP ĐỒNG GIAO TIẾP giữa tầng điều phối (Kiên) và tầng robot (Phát) - đặc tả mục 3
# cả 2 tầng cùng import file này, KHÔNG đặt tên types.py vì trùng thư viện chuẩn
# quy ước giống ACM_LOS_FINAL.py:
#   1 ô viết dạng tuple o = (dòng, cột), dòng tăng khi đi xuống
#   góc: hướng Đông là 0, goc = atan2(dDong, dCot) -> E = 0, S = pi/2, W = pi, N = -pi/2
# note:
# Intent: robot gửi lên mỗi bước (yield)
# Response: điều phối trả về (send)
# 1 lần send = 1 bước thời gian

import math
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple


# Robot tự báo đang làm gì (bảng 3.1). FAILED chỉ điều phối đặt
class Mode(Enum):
    COVER = 'COVER'
    HOLE = 'HOLE'
    BACKTRACK = 'BACKTRACK'
    RETURN = 'RETURN'          # quay về q_resume sau khi phủ hole
    WAIT = 'WAIT'
    FAILED = 'FAILED'


# loại hành động
# STAY: đứng yên 1 bước
# UNCOVERED: bước vào ô CHƯA phủ (quét, vào nhánh c_u (43)(44), vào hole (28))
# TRAVERSE: bước sang ô ĐÃ phủ (quay lui (39)-(42), về q_resume, quay lui cục bộ trong hole)
class Kind(Enum):
    STAY = 'STAY'
    UNCOVERED = 'UNCOVERED'
    TRAVERSE = 'TRAVERSE'


# lý do bị từ chối
# 4 lý do đầu: không cần lập lại kế hoạch (replan = False)
# 4 lý do sau: đi kèm replan = True (bảng 3.4)
class Reason(Enum):
    X1_LOST = 'X1_LOST'
    OCCUPIED = 'OCCUPIED'
    CROSSING = 'CROSSING'
    LOCKED = 'LOCKED'
    GOAL_TAKEN = 'GOAL_TAKEN'
    GOAL_DEAD = 'GOAL_DEAD'
    ABORT = 'ABORT'
    YIELDED = 'YIELDED'


# lỗi tích hợp: robot gửi Intent sai hợp đồng (lỗi code tầng robot, ko phải lỗi thuật toán)
# đặt ở đây để shared_state.py cũng dùng được mà ko phải import conflict.py (tránh vòng import)
class ContractError(Exception):
    pass


# robot gửi lên mỗi bước (bảng 3.2)
# robotId: ID robot gửi
# kind: loại hành động
# target: ô sẽ đứng ở t+1, None khi và chỉ khi STAY
# mode: trạng thái hiện tại
# v, theta: bộ ba ưu tiên [ĐX] (bảng 3.3), v >= 0, theta trong [0, pi]
# goal: c_b đang nhắm tới, giữ nguyên trên MỌI bước của 1 lần quay lui (kể cả bước vào nhánh c_b -> c_u)
#       None ở mọi chế độ khác
# path: các ô CÒN LẠI sẽ đi qua, gồm cả target, ko gồm ô hiện tại (bắt buộc khi TRAVERSE)
# idle: True khi và chỉ khi robot hết việc (ko hướng quét, planningPairs ko cho đích nào tới được)
# forced: chỉ điều phối đặt (bước nhường đường)
@dataclass(frozen=True)
class Intent:
    robotId: int
    kind: Kind
    target: Optional[Tuple[int, int]]
    mode: Mode
    v: float = 0.0
    theta: float = 0.0
    goal: Optional[Tuple[int, int]] = None
    path: Tuple[Tuple[int, int], ...] = ()
    idle: bool = False
    forced: bool = False


# điều phối trả về (bảng 3.4)
# quy tắc chung: replan = True -> bỏ TOÀN BỘ kế hoạch di chuyển đang dở, tính lại từ self.pos, bất kể ok
@dataclass(frozen=True)
class Response:
    ok: bool
    reason: Optional[Reason] = None
    replan: bool = False


# các Response hay dùng
RESP_OK = Response(True)
RESP_X1_LOST = Response(False, Reason.X1_LOST, False)
RESP_OCCUPIED = Response(False, Reason.OCCUPIED, False)
RESP_CROSSING = Response(False, Reason.CROSSING, False)
RESP_LOCKED = Response(False, Reason.LOCKED, False)
RESP_GOAL_TAKEN = Response(False, Reason.GOAL_TAKEN, True)
RESP_GOAL_DEAD = Response(False, Reason.GOAL_DEAD, True)
RESP_OCCUPIED_REPLAN = Response(False, Reason.OCCUPIED, True)     # bị robot HỎNG chặn (R0)
RESP_ABORT = Response(False, Reason.ABORT, True)
RESP_YIELDED = Response(False, Reason.YIELDED, True)


# tạo Intent đứng yên
def makeStayIntent(robotId, mode, idle = False, goal = None):
    return Intent(robotId = robotId, kind = Kind.STAY, target = None, mode = mode,
                  v = 0.0, theta = 0.0, goal = goal, path = (), idle = idle)


# 2 ô kề chéo
def isDiagonal(A, B):
    return abs(A[0] - B[0]) == 1 and abs(A[1] - B[1]) == 1

# 2 ô kề cạnh (N4, công thức (5) bản thảo)
def isNeighbor4(A, B):
    return abs(A[0] - B[0]) + abs(A[1] - B[1]) == 1

# 2 ô kề 8 hướng (khác nhau)
def isNeighbor8(A, B):
    return A != B and abs(A[0] - B[0]) <= 1 and abs(A[1] - B[1]) <= 1

# góc của bước A -> B, cùng quy ước với ACM_LOS_FINAL.py (Đông = 0)
def getStepAngle(A, B):
    return math.atan2(B[0] - A[0], B[1] - A[1])


# 10 quy tắc bắt buộc với tầng robot (mục 3.5). Lớp này chỉ để ĐỌC + làm mẫu, ko bắt kế thừa
# 1. run() là generator: điều phối gọi next(gen) 1 lần lấy Intent đầu, sau đó mỗi bước gen.send(resp) đúng 1 lần
# 2. mỗi lần nhận send phải yield NGAY Intent cho bước kế tiếp
# 3. chỉ đọc shared.covered, shared.positions, shared.planningPairs(k), shared.traversable(),
#    shared.firstPairAt(), shared.goalAlive(); ghi duy nhất shared.addPairs(k, danhSachCap)
# 4. ok = True: KHÔNG tự thêm covered, KHÔNG tự xóa cặp (điều phối đã làm)
# 5. UNCOVERED: bước 4 hướng tới ô chưa phủ; TRAVERSE: tới ô đã phủ, kề 8 hướng, chéo ko cắt góc
# 6. Theta* tính trên shared.traversable(), tách từng đoạn thành dãy ô kề nhau
# 7. gọi findBacktrackPairs SAU khi bước được chấp nhận (đầu lần chạy tiếp theo)
# 8. sau bước chéo phải đưa heading về 1 trong 4 hướng trục trước khi quét tiếp
# 9. forceStep(o): chỉ cập nhật pos, heading, đường đi riêng. KHÔNG yield, ko phát hiện cặp
# 10. ko bao giờ xóa sạch danh sách cặp; ko tới được đích nào -> yield STAY với idle = True
class RobotProtocol:
    robotId = None          # int
    pos = None              # ô đang đứng

    def run(self):          # generator: yield Intent, nhận Response
        raise NotImplementedError

    def forceStep(self, o):  # điều phối dời robot sang ô o (nhường đường)
        raise NotImplementedError
