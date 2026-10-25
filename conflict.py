# xung đột và ưu tiên (đặc tả mục 6)
# 2 nhận xét làm bài toán đơn giản (mục 6.1):
#   1. mọi robot luôn đứng trên ô ĐÃ phủ -> ô chưa phủ ko bao giờ có robot
#      -> bước UNCOVERED chỉ xung đột với bước UNCOVERED khác vào CÙNG ô (X1)
#   2. luật bảo thủ "ko vào ô đang có robot" tự bao xung đột đổi chỗ (X2)
# khóa ưu tiên (M8): nhỏ hơn thắng
#   moveKey = (0 nếu forced ngược lại 1, -v, theta, ID)
#   goalKey = (-v, theta, ID) khi goalRule = 'paper' (đúng ĐX)
#           = (khoảng cách, ID) khi goalRule = 'nearest'

import math

from .protocol import (Kind, Mode, Reason, Intent, ContractError, makeStayIntent,
                       isDiagonal, isNeighbor8,
                       RESP_GOAL_TAKEN, RESP_GOAL_DEAD)
from .grid_utils import turnAngle, isFreeCell, diagonalSideCells

# để ai import ContractError từ conflict.py (như đặc tả) vẫn dùng được
__all__ = ['ContractError', 'validateIntent', 'moveKey', 'goalKey', 'turnAngle',
           'resolveGoals', 'resolveMoves']


# làm tròn số thực theo eps để so sánh ổn định
def roundByEps(x, eps):
    return int(round(x / eps))

# khóa ưu tiên khi xét di chuyển (M8). Bước nhường đường (forced) luôn đứng đầu
def moveKey(it, thetaEps = 1e-9):
    return (0 if it.forced else 1,
            -roundByEps(it.v, thetaEps),
            roundByEps(it.theta, thetaEps),
            it.robotId)

# khóa ưu tiên khi 2 robot tranh 1 đích (X4)
# 'nearest': robot đang đứng ngay tại c_b (bước vào nhánh) có khoảng cách 0
def goalKey(it, goalRule = 'paper', thetaEps = 1e-9):
    if goalRule == 'paper':
        return (-roundByEps(it.v, thetaEps), roundByEps(it.theta, thetaEps), it.robotId)
    khoangCach = it.v if it.kind == Kind.TRAVERSE else 0.0
    return (roundByEps(khoangCach, thetaEps), it.robotId)


# kiểm tra Intent có đúng hợp đồng ko (mục 6.5), sai -> ContractError
# viTri: ô robot k đang đứng; grid (nếu có): kiểm tra thêm target là ô trống
def validateIntent(it, k, shared, grid = None, thetaEps = 1e-9):
    if not isinstance(it, Intent):
        raise ContractError("robot %s yield %r, khong phai Intent" % (k, it))
    if it.robotId != k:
        raise ContractError("robot %s gui Intent mang robotId = %s" % (k, it.robotId))
    if it.forced:
        raise ContractError("robot %s tu dat forced = True (chi dieu phoi duoc dat)" % (k,))
    if not isinstance(it.kind, Kind) or not isinstance(it.mode, Mode):
        raise ContractError("robot %s: kind/mode sai kieu" % (k,))
    if it.mode == Mode.FAILED:
        raise ContractError("robot %s tu bao FAILED" % (k,))
    if not (isinstance(it.v, (int, float)) and math.isfinite(it.v) and it.v >= 0):
        raise ContractError("robot %s: v = %r khong hop le (phai >= 0)" % (k, it.v))
    if not (isinstance(it.theta, (int, float)) and -thetaEps <= it.theta <= math.pi + 1e-7):
        raise ContractError("robot %s: theta = %r ngoai [0, pi]" % (k, it.theta))
    if it.idle and it.kind != Kind.STAY:
        raise ContractError("robot %s: idle = True nhung khong STAY" % (k,))

    # goal: bắt buộc khi đang quay lui và di chuyển, None ở chế độ khác
    if it.mode != Mode.BACKTRACK and it.goal is not None:
        raise ContractError("robot %s: goal != None khi mode = %s" % (k, it.mode.name))
    if it.mode == Mode.BACKTRACK and it.kind != Kind.STAY and it.goal is None:
        raise ContractError("robot %s: dang BACKTRACK ma goal = None" % (k,))

    if it.kind == Kind.STAY:
        if it.target is not None:
            raise ContractError("robot %s: STAY nhung target != None" % (k,))
        return

    o = it.target
    if not (isinstance(o, tuple) and len(o) == 2):
        raise ContractError("robot %s: target = %r sai dang (dong, cot)" % (k, o))
    viTri = shared.positions[k]
    if not isNeighbor8(viTri, o):
        raise ContractError("robot %s: %s -> %s khong ke 8 huong" % (k, viTri, o))
    if grid is not None and not isFreeCell(grid, o):
        raise ContractError("robot %s: buoc vao vat can / ngoai ban do %s" % (k, o))
    if len(it.path) > 0 and tuple(it.path[0]) != o:
        raise ContractError("robot %s: path[0] = %s khac target %s" % (k, it.path[0], o))

    if it.kind == Kind.UNCOVERED:
        if isDiagonal(viTri, o):
            raise ContractError("robot %s: buoc UNCOVERED di cheo %s -> %s" % (k, viTri, o))
        if o in shared.covered:
            raise ContractError("robot %s: buoc UNCOVERED vao o da phu %s" % (k, o))
        return

    # TRAVERSE
    if o not in shared.covered:
        raise ContractError("robot %s: buoc TRAVERSE vao o chua phu %s" % (k, o))
    if isDiagonal(viTri, o):
        oCanh1 = (o[0], viTri[1])
        oCanh2 = (viTri[0], o[1])
        if oCanh1 not in shared.covered or oCanh2 not in shared.covered:
            raise ContractError("robot %s: buoc cheo %s -> %s cat goc" % (k, viTri, o))
    if len(it.path) == 0:
        raise ContractError("robot %s: TRAVERSE phai co path" % (k,))
    for i in range(len(it.path) - 1):
        if not isNeighbor8(it.path[i], it.path[i + 1]):
            raise ContractError("robot %s: path dut quang tai %s -> %s" % (k, it.path[i], it.path[i + 1]))


# PHA 2: xung đột đích X4 + đặt chỗ (mục 6.3)
# trả về (intentsMoi, phanHoi): robot bị từ chối -> Intent đổi thành STAY, phanHoi[k] có replan = True
def resolveGoals(intents, shared, config):
    intentsMoi = dict(intents)
    phanHoi = {}
    ungVien = {}                     # {c_b: [id, ...]}

    for k in sorted(intents):
        it = intents[k]
        if it.forced:
            continue
        g = it.goal
        cur = shared.reserved.get(k)

        # robot ko còn quay lui -> nhả đích
        if g is None:
            if cur is not None:
                shared.release(k)
            continue

        # vẫn giữ đích cũ -> kiểm tra đích còn sống và ko nằm trên ô robot hỏng
        if g == cur:
            if not shared.goalAlive(g) or g in shared.failedCells:
                shared.release(k)
                phanHoi[k] = RESP_GOAL_DEAD
            continue

        # đích mới: robot đã bỏ đích cũ
        if cur is not None:
            shared.release(k)
        if not shared.goalAlive(g) or g in shared.failedCells:
            phanHoi[k] = RESP_GOAL_DEAD
        elif shared.goalOwner.get(g) is not None:
            phanHoi[k] = RESP_GOAL_TAKEN
        elif shared.isTabu(k, g):
            phanHoi[k] = RESP_GOAL_TAKEN
        else:
            ungVien.setdefault(g, []).append(k)

    # nhiều robot cùng nhắm 1 đích mới tại cùng t -> so goalKey
    for g in sorted(ungVien):
        danhSach = sorted(ungVien[g], key = lambda k: goalKey(intents[k], config.goalRule, config.thetaEps))
        shared.reserve(danhSach[0], g)
        for k in danhSach[1:]:
            phanHoi[k] = RESP_GOAL_TAKEN

    for k in phanHoi:
        intentsMoi[k] = makeStayIntent(k, intents[k].mode)
    return intentsMoi, phanHoi


# PHA 3: xung đột di chuyển X1, X3/X2, X2', khóa (mục 6.4)
# khoaO: {id: set ô bị khóa với robot đó} (từ deadlock.lockedCells)
# trả về {id: (chapNhan, lyDo, blocker)} cho mọi robot có kind != STAY
# duyệt tham lam theo moveKey toàn cục: robot chỉ bị từ chối vì 1 robot ưu tiên cao hơn CÓ xung đột với nó
# config.strictDiagonal (bổ sung): bước chéo còn bị chặn khi ô cạnh có robot đứng (OCCUPIED),
#     có robot sắp vào, hoặc ô đích là ô cạnh của 1 bước chéo đã nhận (CROSSING)
def resolveMoves(intents, shared, khoaO, config):
    oDangCo = {}                                 # {ô: id}, MỌI robot kể cả robot hỏng
    for k, o in shared.positions.items():
        oDangCo[o] = k

    danhSachDi = [k for k in intents if intents[k].kind != Kind.STAY]
    danhSachDi.sort(key = lambda k: moveKey(intents[k], config.thetaEps))

    ketQua = {}
    oDaNhan = {}                                 # {ô đích đã chấp nhận: id}
    buocCheoDaNhan = []                          # [(a, b, id)]
    oCanhDaNhan = {}                             # {ô cạnh của bước chéo đã nhận: id} (strictDiagonal)
    for k in danhSachDi:
        it = intents[k]
        a = shared.positions[k]
        b = it.target

        if not it.forced and b in khoaO.get(k, ()):
            ketQua[k] = (False, Reason.LOCKED, None)
            continue
        if b in oDangCo and oDangCo[b] != k:                       # X3, X2
            ketQua[k] = (False, Reason.OCCUPIED, oDangCo[b])
            continue
        if b in oDaNhan:                                           # X1
            ketQua[k] = (False, Reason.X1_LOST, oDaNhan[b])
            continue
        if config.strictDiagonal:
            oCanh = diagonalSideCells(a, b)
            nguoiCanh = [oDangCo[c] for c in oCanh if c in oDangCo and oDangCo[c] != k]
            if len(nguoiCanh) > 0:
                ketQua[k] = (False, Reason.OCCUPIED, nguoiCanh[0])
                continue
            nguoiCat = [oDaNhan[c] for c in oCanh if c in oDaNhan]
            if b in oCanhDaNhan:
                nguoiCat.append(oCanhDaNhan[b])
            if len(nguoiCat) > 0:
                ketQua[k] = (False, Reason.CROSSING, nguoiCat[0])
                continue
        biCat = None
        if isDiagonal(a, b):                                       # X2': 2 đường chéo cùng khối 2x2
            for (c, d, j) in buocCheoDaNhan:
                cungTam = (a[0] + b[0] == c[0] + d[0]) and (a[1] + b[1] == c[1] + d[1])
                if cungTam and {a, b} != {c, d}:
                    biCat = j
                    break
        if biCat is not None:
            ketQua[k] = (False, Reason.CROSSING, biCat)
            continue

        ketQua[k] = (True, None, None)
        oDaNhan[b] = k
        if isDiagonal(a, b):
            buocCheoDaNhan.append((a, b, k))
            for c in diagonalSideCells(a, b):
                oCanhDaNhan[c] = k
    return ketQua
