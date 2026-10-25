# kiểm thử đơn vị (đặc tả mục 12.1): ưu tiên, X1, X3, đổi chỗ, cắt chéo, X4, đặt chỗ, xóa cặp, hợp đồng
# chạy: python -m unittest multi_acm.tests.test_conflict -v

import math
import unittest

from multi_acm.protocol import (Kind, Mode, Reason, Intent, ContractError, makeStayIntent,
                                RESP_GOAL_TAKEN, RESP_GOAL_DEAD)
from multi_acm.config import CoordConfig
from multi_acm.shared_state import SharedState
from multi_acm.conflict import moveKey, goalKey, resolveGoals, resolveMoves, validateIntent
from multi_acm.deadlock import DeadlockManager


def taoShared(viTri, daPhu = ()):
    shared = SharedState(viTri)
    shared.covered.update(daPhu)
    return shared

def intentUncovered(k, o, v = 1.0, theta = 0.0):
    return Intent(robotId = k, kind = Kind.UNCOVERED, target = o, mode = Mode.COVER, v = v, theta = theta)

def intentTraverse(k, o, path = None, v = 1.0, theta = 0.0, goal = None, mode = None):
    if path is None:
        path = (o,)
    if mode is None:
        mode = Mode.BACKTRACK if goal is not None else Mode.RETURN
    return Intent(robotId = k, kind = Kind.TRAVERSE, target = o, mode = mode, v = v, theta = theta,
                  goal = goal, path = tuple(path))

# lưới 5x5 toàn ô trống bên trong viền
GRID_5 = [[1, 1, 1, 1, 1, 1, 1],
          [1, 0, 0, 0, 0, 0, 1],
          [1, 0, 0, 0, 0, 0, 1],
          [1, 0, 0, 0, 0, 0, 1],
          [1, 0, 0, 0, 0, 0, 1],
          [1, 0, 0, 0, 0, 0, 1],
          [1, 1, 1, 1, 1, 1, 1]]
TAT_CA_O_5 = {(d, c) for d in range(1, 6) for c in range(1, 6)}


class TestPriority(unittest.TestCase):
    # ví dụ ĐX: r1 = (5, pi/2, 1), r2 = (7, pi, 2), r3 = (7, pi/2, 3) -> r3 > r2 > r1
    def test_vi_du_DX(self):
        r1 = Intent(1, Kind.TRAVERSE, (0, 0), Mode.BACKTRACK, v = 5, theta = math.pi / 2, goal = (0, 0), path = ((0, 0),))
        r2 = Intent(2, Kind.TRAVERSE, (0, 0), Mode.BACKTRACK, v = 7, theta = math.pi, goal = (0, 0), path = ((0, 0),))
        r3 = Intent(3, Kind.TRAVERSE, (0, 0), Mode.BACKTRACK, v = 7, theta = math.pi / 2, goal = (0, 0), path = ((0, 0),))
        thuTu = sorted([r1, r2, r3], key = moveKey)
        self.assertEqual([it.robotId for it in thuTu], [3, 2, 1])
        thuTu = sorted([r1, r2, r3], key = lambda it: goalKey(it, 'paper'))
        self.assertEqual([it.robotId for it in thuTu], [3, 2, 1])

    def test_forced_luon_dau(self):
        thuong = Intent(0, Kind.TRAVERSE, (1, 1), Mode.RETURN, v = 100, theta = 0, path = ((1, 1),))
        nhuong = Intent(9, Kind.TRAVERSE, (1, 2), Mode.WAIT, v = 1, theta = 0, path = ((1, 2),), forced = True)
        self.assertLess(moveKey(nhuong), moveKey(thuong))

    def test_sai_so_theta(self):
        a = Intent(1, Kind.UNCOVERED, (0, 0), Mode.COVER, v = 2, theta = math.pi / 2)
        b = Intent(2, Kind.UNCOVERED, (0, 0), Mode.COVER, v = 2, theta = math.pi / 2 + 1e-13)
        self.assertLess(moveKey(a), moveKey(b))      # theta coi như bằng nhau -> ID nhỏ thắng


class TestMoves(unittest.TestCase):
    def setUp(self):
        self.config = CoordConfig()

    # X1: 2 robot cùng vào 1 ô chưa phủ, robot v lớn hơn thắng
    def test_X1_cung_o_chua_phu(self):
        shared = taoShared({0: (2, 1), 1: (2, 3)})
        intents = {0: intentUncovered(0, (2, 2), v = 1), 1: intentUncovered(1, (2, 2), v = 3)}
        kq = resolveMoves(intents, shared, {}, self.config)
        self.assertEqual(kq[1], (True, None, None))
        self.assertEqual(kq[0], (False, Reason.X1_LOST, 1))

    # đổi chỗ trên ô đã phủ: cả 2 OCCUPIED (X2 bao bởi luật bảo thủ)
    def test_doi_cho(self):
        shared = taoShared({0: (2, 1), 1: (2, 2)}, TAT_CA_O_5)
        intents = {0: intentTraverse(0, (2, 2)), 1: intentTraverse(1, (2, 1))}
        kq = resolveMoves(intents, shared, {}, self.config)
        self.assertEqual(kq[0][:2], (False, Reason.OCCUPIED))
        self.assertEqual(kq[1][:2], (False, Reason.OCCUPIED))

    # đi nối đuôi: robot sau OCCUPIED, robot trước đi
    def test_noi_duoi(self):
        shared = taoShared({0: (2, 1), 1: (2, 2)}, TAT_CA_O_5)
        intents = {0: intentTraverse(0, (2, 2), v = 9), 1: intentTraverse(1, (2, 3), v = 1)}
        kq = resolveMoves(intents, shared, {}, self.config)
        self.assertEqual(kq[0], (False, Reason.OCCUPIED, 1))
        self.assertEqual(kq[1], (True, None, None))

    # 2 bước chéo cắt nhau trong cùng khối 2x2
    def test_cat_cheo(self):
        shared = taoShared({0: (1, 1), 1: (1, 2)}, TAT_CA_O_5)
        intents = {0: intentTraverse(0, (2, 2), v = 2), 1: intentTraverse(1, (2, 1), v = 1)}
        kq = resolveMoves(intents, shared, {}, self.config)
        self.assertEqual(kq[0], (True, None, None))
        self.assertEqual(kq[1], (False, Reason.CROSSING, 0))

    # 2 bước chéo song song ko cắt nhau
    def test_cheo_song_song(self):
        shared = taoShared({0: (1, 1), 1: (1, 2)}, TAT_CA_O_5)
        intents = {0: intentTraverse(0, (2, 2)), 1: intentTraverse(1, (2, 3))}
        kq = resolveMoves(intents, shared, {}, self.config)
        self.assertTrue(kq[0][0] and kq[1][0])

    # robot ko xung đột với ai luôn được đi, bất kể ưu tiên
    def test_ko_xung_dot(self):
        shared = taoShared({0: (1, 1), 1: (5, 5)})
        intents = {0: intentUncovered(0, (1, 2), v = 1), 1: intentUncovered(1, (4, 5), v = 3)}
        kq = resolveMoves(intents, shared, {}, self.config)
        self.assertTrue(kq[0][0] and kq[1][0])

    # khóa nhường đường: bước vào ô bị khóa -> LOCKED; bước forced ko bị khóa
    def test_LOCKED(self):
        shared = taoShared({0: (2, 1), 1: (4, 4)}, TAT_CA_O_5)
        intents = {0: intentTraverse(0, (2, 2))}
        kq = resolveMoves(intents, shared, {0: {(2, 2), (2, 3)}}, self.config)
        self.assertEqual(kq[0], (False, Reason.LOCKED, None))
        intents = {0: Intent(0, Kind.TRAVERSE, (2, 2), Mode.WAIT, v = 1, path = ((2, 2),), forced = True)}
        kq = resolveMoves(intents, shared, {0: {(2, 2)}}, self.config)
        self.assertEqual(kq[0], (True, None, None))

    # robot hỏng vẫn chiếm ô
    def test_robot_hong_chiem_o(self):
        shared = taoShared({0: (2, 1), 1: (2, 2)}, TAT_CA_O_5)
        shared.failed.add(1)
        shared.failedCells.add((2, 2))
        kq = resolveMoves({0: intentTraverse(0, (2, 2))}, shared, {}, self.config)
        self.assertEqual(kq[0], (False, Reason.OCCUPIED, 1))


class TestGoals(unittest.TestCase):
    def taoCap(self):
        # c_b = (2, 2) đã phủ, c_u = (2, 3) chưa phủ
        shared = taoShared({0: (2, 1), 1: (1, 2), 2: (5, 5)}, {(2, 1), (2, 2), (1, 2), (1, 1)})
        shared.addPairs(0, [((2, 2), (2, 3))])
        return shared

    # X4 cùng đích mới, 'paper': robot v lớn hơn giữ đích, robot kia GOAL_TAKEN + STAY
    def test_X4_paper(self):
        shared = self.taoCap()
        intents = {0: intentTraverse(0, (2, 2), v = 1.0, theta = 0.0, goal = (2, 2)),
                   1: intentTraverse(1, (2, 2), v = 1.0, theta = 1.0, goal = (2, 2))}
        intentsMoi, phanHoi = resolveGoals(intents, shared, CoordConfig(goalRule = 'paper'))
        self.assertEqual(shared.reserved, {0: (2, 2)})       # v hòa, theta 0 < 1 -> robot 0
        self.assertEqual(phanHoi, {1: RESP_GOAL_TAKEN})
        self.assertEqual(intentsMoi[1].kind, Kind.STAY)

    def test_X4_paper_xa_hon_thang(self):
        shared = self.taoCap()
        intents = {0: intentTraverse(0, (2, 2), v = 1.0, goal = (2, 2)),
                   1: intentTraverse(1, (2, 2), v = 5.0, goal = (2, 2))}
        resolveGoals(intents, shared, CoordConfig(goalRule = 'paper'))
        self.assertEqual(shared.reserved, {1: (2, 2)})

    def test_X4_nearest(self):
        shared = self.taoCap()
        intents = {0: intentTraverse(0, (2, 2), v = 1.0, goal = (2, 2)),
                   1: intentTraverse(1, (2, 2), v = 5.0, goal = (2, 2))}
        intentsMoi, phanHoi = resolveGoals(intents, shared, CoordConfig(goalRule = 'nearest'))
        self.assertEqual(shared.reserved, {0: (2, 2)})
        self.assertEqual(phanHoi, {1: RESP_GOAL_TAKEN})

    def test_dich_da_co_nguoi_giu(self):
        shared = self.taoCap()
        shared.reserve(2, (2, 2))
        intentsMoi, phanHoi = resolveGoals({0: intentTraverse(0, (2, 2), goal = (2, 2))}, shared, CoordConfig())
        self.assertEqual(phanHoi, {0: RESP_GOAL_TAKEN})
        self.assertEqual(shared.reserved, {2: (2, 2)})

    def test_dich_chet_va_nha(self):
        shared = self.taoCap()
        shared.reserve(0, (2, 2))
        shared.covered.add((2, 3))
        shared.deletePairsWithCu((2, 3))
        intentsMoi, phanHoi = resolveGoals({0: intentTraverse(0, (2, 2), goal = (2, 2))}, shared, CoordConfig())
        self.assertEqual(phanHoi, {0: RESP_GOAL_DEAD})
        self.assertEqual(shared.reserved, {})
        self.assertEqual(shared.goalOwner, {})

    def test_dich_moi_da_chet(self):
        shared = self.taoCap()
        intentsMoi, phanHoi = resolveGoals({0: intentTraverse(0, (2, 1), goal = (1, 1))}, shared, CoordConfig())
        self.assertEqual(phanHoi, {0: RESP_GOAL_DEAD})

    def test_robot_ko_con_goal_thi_nha(self):
        shared = self.taoCap()
        shared.reserve(0, (2, 2))
        resolveGoals({0: makeStayIntent(0, Mode.WAIT, idle = True)}, shared, CoordConfig())
        self.assertEqual(shared.reserved, {})

    def test_dich_tren_o_robot_hong(self):
        shared = self.taoCap()
        shared.reserve(0, (2, 2))
        shared.failedCells.add((2, 2))
        intentsMoi, phanHoi = resolveGoals({0: intentTraverse(0, (2, 2), goal = (2, 2))}, shared, CoordConfig())
        self.assertEqual(phanHoi, {0: RESP_GOAL_DEAD})
        self.assertEqual(shared.reserved, {})

    def test_dich_tabu(self):
        shared = self.taoCap()
        shared.addTabu(0, (2, 2), 5)
        intentsMoi, phanHoi = resolveGoals({0: intentTraverse(0, (2, 2), goal = (2, 2))}, shared, CoordConfig())
        self.assertEqual(phanHoi, {0: RESP_GOAL_TAKEN})
        self.assertEqual(shared.planningPairs(0), [])
        self.assertEqual(shared.planningPairs(1), [((2, 2), (2, 3))])
        shared.t = 5
        self.assertEqual(shared.planningPairs(0), [((2, 2), (2, 3))])

    def test_doi_dich_thi_nha_dich_cu(self):
        shared = self.taoCap()
        shared.covered.add((3, 1))
        shared.addPairs(0, [((3, 1), (4, 1))])
        shared.reserve(0, (2, 2))
        resolveGoals({0: intentTraverse(0, (3, 1), goal = (3, 1))}, shared, CoordConfig())
        self.assertEqual(shared.reserved, {0: (3, 1)})
        self.assertNotIn((2, 2), shared.goalOwner)


class TestPairs(unittest.TestCase):
    # 2 cặp chung c_b, 1 c_u bị phủ -> cặp anh em VẪN CÒN (sửa ĐX IV.3)
    def test_cap_anh_em_con_lai(self):
        shared = taoShared({0: (2, 2)})
        shared.addPairs(0, [((2, 2), (2, 1)), ((2, 2), (2, 3))])
        shared.reserve(0, (2, 2))
        shared.covered.add((2, 1))
        shared.deletePairsWithCu((2, 1))
        self.assertEqual(shared.L, [((2, 2), (2, 3))])
        self.assertTrue(shared.goalAlive((2, 2)))
        self.assertEqual(shared.firstPairAt((2, 2)), ((2, 2), (2, 3)))

    def test_addPairs_dieu_kien(self):
        shared = taoShared({0: (2, 2)})
        shared.covered.add((1, 2))
        them = shared.addPairs(0, [((2, 2), (2, 3)), ((2, 2), (2, 3)), ((3, 3), (3, 4)), ((2, 2), (1, 2))])
        self.assertEqual(them, [((2, 2), (2, 3))])                  # trùng, c_b chưa phủ, c_u đã phủ: bỏ qua
        with self.assertRaises(ContractError):
            shared.addPairs(0, [((2, 2), (3, 3))])                  # ko kề cạnh

    def test_L_sua_tai_cho(self):
        shared = taoShared({0: (2, 2)})
        thamChieu = shared.L
        shared.addPairs(0, [((2, 2), (2, 3)), ((2, 2), (1, 2))])
        shared.covered.add((2, 3))
        shared.deletePairsWithCu((2, 3))
        self.assertIs(shared.L, thamChieu)
        self.assertEqual(thamChieu, [((2, 2), (1, 2))])

    def test_firstPairAt_theo_thu_tu(self):
        shared = taoShared({0: (2, 2)})
        shared.addPairs(0, [((2, 2), (3, 2)), ((2, 2), (1, 2))])
        self.assertEqual(shared.firstPairAt((2, 2)), ((2, 2), (3, 2)))

    def test_reserve_hai_chieu(self):
        shared = taoShared({0: (2, 2), 1: (4, 4)})
        shared.reserve(0, (2, 2))
        shared.reserve(0, (4, 4))
        self.assertEqual(shared.reserved, {0: (4, 4)})
        self.assertEqual(shared.goalOwner, {(4, 4): 0})
        with self.assertRaises(RuntimeError):
            shared.reserve(1, (4, 4))


class TestContract(unittest.TestCase):
    def setUp(self):
        self.shared = taoShared({0: (2, 2)}, {(2, 2), (2, 3), (3, 3)})

    def kiemTra(self, it):
        validateIntent(it, 0, self.shared, GRID_5)

    def test_hop_le(self):
        self.kiemTra(intentUncovered(0, (1, 2)))
        self.kiemTra(intentTraverse(0, (2, 3)))
        self.kiemTra(makeStayIntent(0, Mode.WAIT, idle = True))

    def test_sai(self):
        danhSachSai = [
            intentUncovered(0, (3, 3)),                                    # UNCOVERED đi chéo
            intentUncovered(0, (2, 3)),                                    # UNCOVERED vào ô đã phủ
            intentTraverse(0, (1, 2)),                                     # TRAVERSE vào ô chưa phủ
            intentTraverse(0, (2, 4), path = ((2, 4),)),                   # ko kề
            Intent(0, Kind.STAY, (2, 3), Mode.WAIT),                       # STAY có target
            Intent(0, Kind.TRAVERSE, (2, 3), Mode.RETURN, path = ()),      # TRAVERSE thiếu path
            Intent(0, Kind.TRAVERSE, (2, 3), Mode.RETURN, path = ((3, 3),)),   # path[0] != target
            Intent(1, Kind.UNCOVERED, (1, 2), Mode.COVER),                 # sai robotId
            Intent(0, Kind.UNCOVERED, (1, 2), Mode.COVER, v = -1),         # v âm
            Intent(0, Kind.UNCOVERED, (1, 2), Mode.COVER, theta = 4.0),    # theta > pi
            Intent(0, Kind.UNCOVERED, (1, 2), Mode.COVER, idle = True),    # idle mà ko STAY
            Intent(0, Kind.UNCOVERED, (1, 2), Mode.COVER, goal = (2, 2)),  # goal khi ko quay lui
            Intent(0, Kind.TRAVERSE, (2, 3), Mode.BACKTRACK, path = ((2, 3),)),   # quay lui thiếu goal
            Intent(0, Kind.UNCOVERED, (1, 2), Mode.COVER, forced = True),  # robot tự đặt forced
            Intent(0, Kind.UNCOVERED, (1, 2), Mode.FAILED),                # robot tự báo FAILED
            'khong phai Intent',
        ]
        for it in danhSachSai:
            with self.assertRaises(ContractError, msg = repr(it)):
                self.kiemTra(it)

    def test_cheo_cat_goc(self):
        shared = taoShared({0: (2, 2)}, {(2, 2), (3, 3), (2, 3)})
        with self.assertRaises(ContractError):
            validateIntent(intentTraverse(0, (3, 3)), 0, shared, GRID_5)    # (3, 2) chưa phủ
        shared.covered.add((3, 2))
        validateIntent(intentTraverse(0, (3, 3)), 0, shared, GRID_5)

    def test_vao_vat_can(self):
        shared = taoShared({0: (1, 1)})
        with self.assertRaises(ContractError):
            validateIntent(intentUncovered(0, (0, 1)), 0, shared, GRID_5)


class TestStrictDiagonal(unittest.TestCase):
    # mặc định (đúng đặc tả): robot đứng ở ô cạnh ko chặn bước chéo
    def test_mac_dinh_cho_phep(self):
        shared = taoShared({0: (1, 1), 1: (1, 2)}, TAT_CA_O_5)
        kq = resolveMoves({0: intentTraverse(0, (2, 2))}, shared, {}, CoordConfig())
        self.assertTrue(kq[0][0])

    # strictDiagonal: robot ở ô cạnh -> OCCUPIED (để chống kẹt xử lý được)
    def test_robot_o_canh(self):
        shared = taoShared({0: (1, 1), 1: (1, 2)}, TAT_CA_O_5)
        kq = resolveMoves({0: intentTraverse(0, (2, 2))}, shared, {}, CoordConfig(strictDiagonal = True))
        self.assertEqual(kq[0], (False, Reason.OCCUPIED, 1))

    # strictDiagonal: robot khác bước vào ô cạnh của bước chéo đã nhận -> CROSSING
    def test_buoc_vao_o_canh(self):
        shared = taoShared({0: (1, 1), 1: (3, 2)}, TAT_CA_O_5)
        intents = {0: intentTraverse(0, (2, 2), v = 5), 1: intentTraverse(1, (2, 1), v = 1)}
        kq = resolveMoves(intents, shared, {}, CoordConfig(strictDiagonal = True))
        self.assertTrue(kq[0][0])
        self.assertEqual(kq[1], (False, Reason.CROSSING, 0))
        # ngược lại: robot vào ô cạnh trước (ưu tiên cao hơn), bước chéo bị chặn
        intents = {0: intentTraverse(0, (2, 2), v = 1), 1: intentTraverse(1, (2, 1), v = 5)}
        kq = resolveMoves(intents, shared, {}, CoordConfig(strictDiagonal = True))
        self.assertTrue(kq[1][0])
        self.assertEqual(kq[0], (False, Reason.CROSSING, 1))


class TestDeadlockUnit(unittest.TestCase):
    # chu trình 3 robot chặn vòng tròn được phát hiện
    def test_tim_chu_trinh(self):
        shared = taoShared({0: (1, 1), 1: (1, 2), 2: (2, 2)}, TAT_CA_O_5)
        dl = DeadlockManager(shared, CoordConfig())
        dl.blocker = {0: 1, 1: 2, 2: 0}
        self.assertEqual(dl._findCycle(0), [0, 1, 2])
        dl.blocker = {0: 1, 1: 2}
        self.assertIsNone(dl._findCycle(0))


if __name__ == '__main__':
    unittest.main()
