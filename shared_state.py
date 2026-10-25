# dữ liệu chung của cả đội (đặc tả mục 4)
# CHỈ tầng điều phối được ghi, robot chỉ được ĐỌC + gọi addPairs()
# note:
# self.covered: C_cov(t) = hợp C^k_cov (M2), khởi tạo = các ô xuất phát
# self.L: list cặp quay lui (c_b, c_u) CHUNG cả đội, giữ thứ tự xuất hiện (L(t) trong kế hoạch)
#         mọi chỗ sửa self.L đều sửa TẠI CHỖ (self.L[:] = ...) để robot nào trỏ self.L = shared.L ko bị mất liên kết
# self.reserved / self.goalOwner: đặt chỗ đích quay lui, 2 chiều luôn khớp nhau

from .protocol import Mode, ContractError, isNeighbor4


class SharedState:
    # viTriXuatPhat: dict {id: ô} hoặc list ô (id = 0, 1, 2, ...)
    def __init__(self, viTriXuatPhat):
        if not isinstance(viTriXuatPhat, dict):
            viTriXuatPhat = {k: o for k, o in enumerate(viTriXuatPhat)}
        if len(set(viTriXuatPhat.values())) != len(viTriXuatPhat):
            raise ValueError("2 robot xuat phat cung 1 o")

        self.t = 0                                    # số bước đã chạy xong
        self.positions = dict(viTriXuatPhat)          # {id: ô}, gồm cả robot hỏng
        self.modes = {k: Mode.COVER for k in viTriXuatPhat}

        self.covered = set(viTriXuatPhat.values())    # (M2)
        self.coveredBy = {k: {o} for k, o in viTriXuatPhat.items()}   # chỉ để đo tải

        self.L = []                                   # L(t) chung
        self.lostPairs = set()                        # cặp ko tới được, CHỈ ghi lúc dừng (mục 8.2)

        self.reserved = {}                            # {id: c_b}
        self.goalOwner = {}                           # {c_b: id}

        self.failed = set()                           # robot hỏng
        self.failedCells = set()                      # ô robot hỏng đang đứng

        self.tabu = {k: {} for k in viTriXuatPhat}    # {id: {c_b: bước hết hạn}}
        self.soonCovered = set()                      # (Mức 2) ô sắp được robot đang quét phủ

    # ---------------- robot được gọi ----------------

    # thêm cặp quay lui, giữ (35): c_b đã phủ, c_u chưa phủ, kề cạnh
    # bỏ qua (ko lỗi) nếu cặp đã có, đã mất, hoặc ko còn thỏa điều kiện phủ
    # ném ContractError nếu c_u ko kề cạnh c_b (lỗi code tầng robot)
    # trả về list cặp thật sự được thêm
    # k = None: điều phối tự thêm (neo lại cặp khi robot hỏng, mục 8 - bổ sung)
    def addPairs(self, k, danhSachCap):
        if k is not None and k in self.failed:
            raise ContractError("robot %s da hong ma van them cap" % (k,))
        capDaThem = []
        for cap in danhSachCap:
            c_b, c_u = tuple(cap[0]), tuple(cap[1])
            if not isNeighbor4(c_b, c_u):
                raise ContractError("cap (%s, %s) khong ke canh" % (c_b, c_u))
            cap = (c_b, c_u)
            if cap in self.L or cap in self.lostPairs:
                continue
            if c_b not in self.covered or c_u in self.covered:
                continue
            self.L.append(cap)
            capDaThem.append(cap)
        return capDaThem

    # vùng robot được đi trên đó: covered trừ ô robot hỏng
    def traversable(self):
        return self.covered - self.failedCells

    # cặp còn hoạt động (chưa bị đánh dấu mất), giữ thứ tự
    def activePairs(self):
        return [cap for cap in self.L if cap not in self.lostPairs]

    # cặp robot k được chọn làm đích, giữ thứ tự xuất hiện (bản nâng của công thức (5) BoB)
    def planningPairs(self, k):
        ketQua = []
        for (c_b, c_u) in self.L:
            if (c_b, c_u) in self.lostPairs:
                continue
            if c_u in self.covered:
                continue
            if c_b in self.failedCells:
                continue
            chu = self.goalOwner.get(c_b)
            if chu is not None and chu != k:
                continue
            if self.isTabu(k, c_b):
                continue
            if c_u in self.soonCovered:
                continue
            ketQua.append((c_b, c_u))
        return ketQua

    # còn ít nhất 1 cặp hoạt động có c_b này và c_u chưa phủ
    def goalAlive(self, c_b):
        for (c_bCap, c_u) in self.L:
            if c_bCap == c_b and c_u not in self.covered and (c_bCap, c_u) not in self.lostPairs:
                return True
        return False

    # cặp XUẤT HIỆN ĐẦU TIÊN có c_b này (quy tắc chọn c_u của ACM mục 4.5.2), ko có -> None
    def firstPairAt(self, c_b):
        for (c_bCap, c_u) in self.L:
            if c_bCap == c_b and c_u not in self.covered and (c_bCap, c_u) not in self.lostPairs:
                return (c_bCap, c_u)
        return None

    def isTabu(self, k, c_b):
        hetHan = self.tabu.get(k, {}).get(c_b)
        return hetHan is not None and self.t < hetHan

    def liveRobots(self):
        return sorted(k for k in self.positions if k not in self.failed)

    # ---------------- chỉ điều phối gọi ----------------

    # xóa MỌI cặp có c_u = o: (33)(34) của ACM nâng lên toàn đội (M9)
    # KHÔNG xóa khi robot tới c_b (sửa câu chữ ĐX IV.3, đặc tả mục 14 dòng 2)
    def deletePairsWithCu(self, o):
        capBiXoa = [cap for cap in self.L if cap[1] == o]
        if len(capBiXoa) > 0:
            self.L[:] = [cap for cap in self.L if cap[1] != o]
        return capBiXoa

    # đặt chỗ đích c_b cho robot k, tự nhả đích cũ của k
    def reserve(self, k, c_b):
        chu = self.goalOwner.get(c_b)
        if chu is not None and chu != k:
            raise RuntimeError("dich %s da co robot %s giu" % (c_b, chu))
        self.release(k)
        self.reserved[k] = c_b
        self.goalOwner[c_b] = k

    # nhả đích của robot k (nếu có), trả về đích vừa nhả
    def release(self, k):
        c_b = self.reserved.pop(k, None)
        if c_b is not None and self.goalOwner.get(c_b) == k:
            del self.goalOwner[c_b]
        return c_b

    # cấm robot k chọn c_b trong soBuoc bước
    def addTabu(self, k, c_b, soBuoc):
        if soBuoc <= 0:
            return
        self.tabu.setdefault(k, {})[c_b] = self.t + soBuoc

    def clearExpiredTabu(self):
        for k in self.tabu:
            hetHan = [c_b for c_b, buoc in self.tabu[k].items() if buoc <= self.t]
            for c_b in hetHan:
                del self.tabu[k][c_b]
