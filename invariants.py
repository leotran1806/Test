# bất biến kiểm tra mỗi bước khi checkInvariants = True (đặc tả mục 9)
# vi phạm -> InvariantError: lỗi LOGIC, phải dừng sửa ngay
# I1: ko 2 robot cùng 1 ô (M13)
# I2: mọi robot đứng trên ô đã phủ (mục 6.1)
# I3: covered ko giảm (tăng đơn điệu, dùng trong Định lý 5.4)
# I4: covered là tập con của ô trống bản đồ
# I5: mọi cặp hoạt động: c_b đã phủ, c_u chưa phủ, kề 4 hướng, ko trùng (35) / Bổ đề 5.1
# I6: reserved và goalOwner khớp nhau, mỗi đích tối đa 1 robot, ko robot hỏng nào giữ đích

from .protocol import isNeighbor4


class InvariantError(Exception):
    pass


# oTrong: tập ô trống của bản đồ; coveredTruoc: covered ở lần kiểm tra trước (None -> bỏ qua I3)
def check(shared, oTrong, coveredTruoc = None):
    # I1
    viTri = list(shared.positions.values())
    if len(set(viTri)) != len(viTri):
        raise InvariantError("I1: 2 robot cung 1 o, t = %d, positions = %s" % (shared.t, shared.positions))

    # I2
    for k, o in shared.positions.items():
        if o not in shared.covered:
            raise InvariantError("I2: robot %s dung tren o chua phu %s" % (k, o))

    # I3
    if coveredTruoc is not None and not coveredTruoc <= shared.covered:
        raise InvariantError("I3: covered bi giam: %s" % (coveredTruoc - shared.covered,))

    # I4
    if not shared.covered <= oTrong:
        raise InvariantError("I4: covered chua o khong trong: %s" % (shared.covered - oTrong,))

    # I5
    if len(set(shared.L)) != len(shared.L):
        raise InvariantError("I5: L co cap trung")
    for (c_b, c_u) in shared.activePairs():
        if c_b not in shared.covered:
            raise InvariantError("I5: cap (%s, %s) co c_b chua phu" % (c_b, c_u))
        if c_u in shared.covered:
            raise InvariantError("I5: cap (%s, %s) co c_u da phu" % (c_b, c_u))
        if not isNeighbor4(c_b, c_u):
            raise InvariantError("I5: cap (%s, %s) khong ke canh" % (c_b, c_u))

    # I6
    for k, c_b in shared.reserved.items():
        if shared.goalOwner.get(c_b) != k:
            raise InvariantError("I6: reserved[%s] = %s nhung goalOwner = %s" % (k, c_b, shared.goalOwner.get(c_b)))
        if k in shared.failed:
            raise InvariantError("I6: robot hong %s van giu dich %s" % (k, c_b))
    for c_b, k in shared.goalOwner.items():
        if shared.reserved.get(k) != c_b:
            raise InvariantError("I6: goalOwner[%s] = %s nhung reserved = %s" % (c_b, k, shared.reserved.get(k)))
