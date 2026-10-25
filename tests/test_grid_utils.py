# kiểm thử hàm hỗ trợ tách đoạn Theta* thành dãy ô (quy tắc 6 của hợp đồng)
# so với canGoStraight() của ACM_LOS_FINAL.py: mọi ô trong dãy và mọi ô cạnh của bước chéo
# đều phải nằm trong tập ô mà canGoStraight đã kiểm tra -> bước TRAVERSE luôn đúng hợp đồng
# chạy: python -m unittest multi_acm.tests.test_grid_utils -v

import math
import random
import unittest

from multi_acm.grid_utils import (splitSegmentToCells, splitPathToCells, diagonalSideCells,
                                  bfsPathTo, bfsReach, pathFootprint)
from multi_acm.protocol import isNeighbor8


# các ô mà canGoStraight(A, B) của ACM_LOS_FINAL.py yêu cầu nằm trong vungChoPhep (chép đúng logic)
def cellsCheckedByCanGoStraight(A, B):
    dong0, cot0 = A
    dong1, cot1 = B
    dDong = dong1 - dong0
    dCot = cot1 - cot0
    danhSachT = {0.0, 1.0}
    for k in range(min(dong0, dong1), max(dong0, dong1) + 1):
        if dDong != 0:
            for bien in (k - 0.5, k + 0.5):
                t = (bien - dong0) / dDong
                if 0 < t < 1:
                    danhSachT.add(t)
    for k in range(min(cot0, cot1), max(cot0, cot1) + 1):
        if dCot != 0:
            for bien in (k - 0.5, k + 0.5):
                t = (bien - cot0) / dCot
                if 0 < t < 1:
                    danhSachT.add(t)
    danhSachT = sorted(danhSachT)
    diemKiemTra = list(danhSachT)
    for i in range(len(danhSachT) - 1):
        diemKiemTra.append((danhSachT[i] + danhSachT[i + 1]) / 2)
    saiSo = 1e-7
    ketQua = set()
    for t in diemKiemTra:
        dongThuc = dong0 + dDong * t
        cotThuc = cot0 + dCot * t
        for dong in range(math.ceil(dongThuc - 0.5 - saiSo), math.floor(dongThuc + 0.5 + saiSo) + 1):
            for cot in range(math.ceil(cotThuc - 0.5 - saiSo), math.floor(cotThuc + 0.5 + saiSo) + 1):
                ketQua.add((dong, cot))
    return ketQua


class TestSplit(unittest.TestCase):
    def test_doan_ngau_nhien(self):
        rng = random.Random(1)
        for _ in range(3000):
            A = (rng.randint(0, 12), rng.randint(0, 12))
            B = (rng.randint(0, 12), rng.randint(0, 12))
            if A == B:
                continue
            cacO = splitSegmentToCells(A, B)
            vung = cellsCheckedByCanGoStraight(A, B)
            self.assertEqual(cacO[0], A)
            self.assertEqual(cacO[-1], B)
            self.assertEqual(len(set(cacO)), len(cacO))
            for i in range(len(cacO) - 1):
                self.assertTrue(isNeighbor8(cacO[i], cacO[i + 1]), (A, B, cacO))
                for c in diagonalSideCells(cacO[i], cacO[i + 1]):
                    self.assertIn(c, vung, (A, B, cacO))
            self.assertTrue(set(cacO) <= vung)

    def test_vi_du(self):
        self.assertEqual(splitSegmentToCells((0, 0), (0, 3)), [(0, 0), (0, 1), (0, 2), (0, 3)])
        self.assertEqual(splitSegmentToCells((0, 0), (2, 2)), [(0, 0), (1, 1), (2, 2)])
        self.assertEqual(splitPathToCells([(0, 0), (2, 2), (2, 4)]), [(1, 1), (2, 2), (2, 3), (2, 4)])

    def test_bfs(self):
        vung = {(0, 0), (0, 1), (1, 1), (1, 0), (2, 2)}
        self.assertEqual(bfsPathTo((0, 0), {(1, 1)}, vung), [(1, 1)])
        self.assertIsNone(bfsPathTo((0, 0), {(2, 2)}, vung))         # (1,2), (2,1) ko trong vùng -> ko cắt góc
        self.assertEqual(bfsPathTo((0, 0), {(0, 0)}, vung), [])
        self.assertEqual(bfsReach([(0, 0)], vung), {(0, 0), (0, 1), (1, 1), (1, 0)})

    def test_footprint(self):
        self.assertEqual(pathFootprint((0, 0), [(1, 1)], False), {(1, 1)})
        self.assertEqual(pathFootprint((0, 0), [(1, 1)], True), {(1, 1), (1, 0), (0, 1)})


if __name__ == '__main__':
    unittest.main()
