# kiểm thử kịch bản (đặc tả mục 12.2): đối đầu hành lang, robot rảnh chắn đường, ngõ cụt, robot hỏng
# + chu trình 4 robot, đích bị robot khác phủ mất (T13), n = 1 điều phối luôn trả OK
# chạy: python -m unittest multi_acm.tests.test_scenarios -v

import unittest

from multi_acm.config import CoordConfig
from multi_acm.shared_state import SharedState
from multi_acm.coordinator import Coordinator
from multi_acm.simulator import parse, computeCAcc, runSimple
from multi_acm.grid_utils import getFreeCells
from multi_acm.robots.goal_robot import GoalRobot
from multi_acm.robots.simple_robot import SimpleRobot

# 2 phòng nối bằng hành lang rộng 1 ô (hàng 2, cột 4..8)
HAI_PHONG = """
#############
#...#####...#
#...........#
#...#####...#
#############
"""

# bản đồ trong README Phase 1 (8 x 10, 38 ô trống)
BAN_DO_README = """
##########
#0.......#
#.##..#..#
#.#...#..#
#...#....#
#...#.##.#
#.#.....1#
##########
"""


# tạo coordinator với bản đồ đã phủ sẵn (GoalRobot chỉ đi trên vùng đã phủ)
# moTa: {id: (ô xuất phát, ô đích, laQuayLui)}
def taoGoal(banDo, moTa, config = None, phuTruoc = True, capThem = ()):
    grid, _ = parse(banDo)
    shared = SharedState({k: v[0] for k, v in moTa.items()})
    if phuTruoc:
        shared.covered.update(getFreeCells(grid))
    for cap in capThem:
        shared.covered.discard(cap[1])
    shared.addPairs(None, list(capThem))
    robots = {k: GoalRobot(k, shared, v[1], laQuayLui = v[2]) for k, v in moTa.items()}
    coord = Coordinator(grid, shared, list(robots.values()), config or CoordConfig(maxSteps = 500))
    return grid, shared, robots, coord


class TestScenarios(unittest.TestCase):
    # đối đầu trong hành lang 1 ô: cả 2 tới đích, có >= 1 lần nhường đường, ko livelock
    def test_doi_dau_hanh_lang(self):
        grid, shared, robots, coord = taoGoal(HAI_PHONG, {0: ((2, 1), (2, 11), False),
                                                          1: ((2, 11), (2, 1), False)})
        kq = coord.run()
        self.assertTrue(robots[0].daToi and robots[1].daToi)
        self.assertGreaterEqual(kq['deadlock_stats']['R1'], 1)
        self.assertLess(kq['steps'], 60)

    # robot rảnh đứng giữa hành lang: bị dời, robot kia qua được
    def test_robot_ranh_chan_hanh_lang(self):
        grid, shared, robots, coord = taoGoal(HAI_PHONG, {0: ((2, 1), (2, 11), False),
                                                          1: ((2, 6), (2, 6), False)})
        kq = coord.run()
        self.assertTrue(robots[0].daToi)
        self.assertGreaterEqual(robots[1].soLanBiDoi, 1)
        self.assertNotIn(shared.positions[1], {(2, c) for c in range(4, 9)})   # ko còn đứng trong hành lang

    # ngõ cụt, robot rảnh đứng trên đích, ko có chỗ né, robot kia ko giữ đích -> R2b, vẫn dừng
    def test_ngo_cut_R2b(self):
        grid, shared, robots, coord = taoGoal("#########\n#.......#\n#########",
                                              {0: ((1, 1), (1, 7), False), 1: ((1, 7), (1, 7), False)})
        kq = coord.run()
        self.assertTrue(robots[0].boCuoc)
        self.assertGreaterEqual(kq['deadlock_stats']['R2b'], 1)

    # robot rảnh lùi sâu hơn vào ngõ cụt để nhường
    def test_lui_sau_hon(self):
        grid, shared, robots, coord = taoGoal("##########\n#........#\n##########",
                                              {0: ((1, 1), (1, 4), False), 1: ((1, 3), (1, 3), False)})
        coord.run()
        self.assertTrue(robots[0].daToi)
        self.assertGreaterEqual(shared.positions[1][1], 5)

    # R2a: hàng robot rảnh chắn kín ngõ cụt dẫn tới đích -> chuyển đích cho robot rảnh
    # luật 'paper': robot ở xa (v lớn) thắng X4 -> robot gần bị thua rồi rảnh, đứng chắn đường
    def test_R2a_chuyen_dich(self):
        banDo = "########\n#......#\n######.#\n########"
        grid, _ = parse(banDo)
        shared = SharedState({0: (1, 1), 1: (1, 5)})
        shared.covered.update({(1, c) for c in range(1, 7)})
        shared.addPairs(None, [((1, 6), (2, 6))])
        k = GoalRobot(0, shared, (1, 6), laQuayLui = True)
        j = SimpleRobot(1, grid, shared)
        coord = Coordinator(grid, shared, [k, j], CoordConfig(goalRule = 'paper', maxSteps = 200))
        kq = coord.run()
        self.assertEqual(kq['deadlock_stats']['R2a'], 1)
        self.assertTrue(k.boCuoc)
        self.assertEqual(shared.covered, getFreeCells(grid))
        self.assertIn((2, 6), shared.coveredBy[1])

    # robot hỏng giữa hành lang: R0, robot kia ko treo
    def test_robot_hong_hanh_lang(self):
        grid, shared, robots, coord = taoGoal(HAI_PHONG, {0: ((2, 1), (2, 11), False),
                                                          1: ((2, 6), (2, 6), False)})
        kq = coord.run(failAt = {1: 0})
        self.assertGreaterEqual(kq['deadlock_stats']['R0'], 1)
        self.assertTrue(robots[0].boCuoc)
        self.assertLess(kq['steps'], 30)

    # 4 robot quay vòng trong khối 2x2 (chu trình chặn nhau độ dài 4) -> phải gỡ được
    def test_chu_trinh_4_robot(self):
        banDo = "#######\n#.....#\n#.....#\n#######"
        grid, shared, robots, coord = taoGoal(banDo, {0: ((1, 1), (1, 2), False), 1: ((1, 2), (2, 2), False),
                                                      2: ((2, 2), (2, 1), False), 3: ((2, 1), (1, 1), False)})
        kq = coord.run()
        self.assertTrue(all(r.daToi for r in robots.values()))
        self.assertGreaterEqual(kq['deadlock_stats']['R1'], 1)

    # T13: robot A đang quay lui tới c_b thì robot B phủ mất c_u -> A nhận GOAL_DEAD, ko bước vào ô đã phủ
    def test_dich_bi_phu_mat(self):
        banDo = "##########\n#........#\n########.#\n########.#\n##########"
        grid, _ = parse(banDo)
        shared = SharedState({0: (1, 1), 1: (3, 8)})
        shared.covered.update({(1, c) for c in range(1, 9)})
        shared.addPairs(None, [((1, 8), (2, 8))])
        a = GoalRobot(0, shared, (1, 8), laQuayLui = True)
        b = SimpleRobot(1, grid, shared)
        coord = Coordinator(grid, shared, [a, b], CoordConfig(maxSteps = 200))
        kq = coord.run()
        self.assertTrue(a.boCuoc)
        self.assertGreaterEqual(kq['stats']['GOAL_DEAD'], 1)
        self.assertIn((2, 8), shared.coveredBy[1])
        self.assertEqual(shared.covered, getFreeCells(grid))

    # 2 robot phủ bản đồ có vật cản 8 x 10: đúng C_acc
    def test_hai_robot_ban_do_README(self):
        grid, viTri = parse(BAN_DO_README)
        coord, robots, kq = runSimple(grid, viTri)
        self.assertEqual(kq['covered'], computeCAcc(grid, viTri.values()))
        self.assertEqual(len(kq['covered']), 38)

    # n = 1: điều phối luôn trả OK (mọi khác biệt với ACM đơn robot là lỗi tách bước)
    def test_mot_robot_luon_OK(self):
        grid, viTri = parse(BAN_DO_README)
        coord, robots, kq = runSimple(grid, {0: viTri[0]})
        self.assertEqual(kq['covered'], computeCAcc(grid, [viTri[0]]))
        for ten in ('X1', 'X2_cheo', 'X3', 'LOCKED', 'X4_GOAL_TAKEN', 'GOAL_DEAD'):
            self.assertEqual(kq['stats'][ten], 0, ten)
        self.assertEqual(sum(kq['deadlock_stats'].values()), 0)
        self.assertEqual(kq['waits'][0], 0)                    # ko bước nào đứng yên

    # robot o hỏng trong lúc robot y đang né cho o: kế hoạch né bị hủy, y lập lại từ vị trí MỚI (ko dùng Intent cũ)
    def test_hong_khi_dang_nhuong_duong(self):
        for buocThem in range(0, 3):
            grid, shared, robots, coord = taoGoal(HAI_PHONG, {0: ((2, 1), (2, 11), False),
                                                              1: ((2, 6), (2, 6), False)})
            coord.start()
            while not coord.deadlock.isYielding(1):
                self.assertFalse(coord.daDung)
                coord.step()
            for _ in range(buocThem):
                if coord.deadlock.isYielding(1):
                    coord.step()
            coord.failRobot(0)
            self.assertFalse(coord.deadlock.isYielding(1))
            self.assertEqual(coord.deadlock.pending, {})
            kq = coord.run()
            self.assertEqual(robots[1].pos, shared.positions[1])
            self.assertIn(0, kq['failed'])

    # robot hỏng chắn lối DUY NHẤT vào 1 vùng chưa phủ: thuật toán vẫn dừng, báo cặp mất
    def test_cap_mat_khi_hong_chan_loi(self):
        banDo = "########\n#......#\n####.###\n####.###\n########"
        grid, _ = parse(banDo)
        shared = SharedState({0: (1, 1), 1: (1, 4)})
        shared.covered.update({(1, c) for c in range(1, 7)})
        shared.addPairs(None, [((1, 4), (2, 4))])
        robots = [SimpleRobot(0, grid, shared), SimpleRobot(1, grid, shared)]
        coord = Coordinator(grid, shared, robots, CoordConfig(maxSteps = 200))
        coord.start()
        coord.failRobot(1)                     # robot 1 hỏng ngay trên c_b, (2, 4) chỉ vào được từ (1, 4)
        kq = coord.run()
        self.assertEqual(kq['lost_pairs'], {((1, 4), (2, 4))})
        self.assertNotIn((2, 4), kq['covered'])

    # robot hỏng đứng trên c_b nhưng c_u còn ô kề cạnh khác đã phủ -> neo lại cặp, ko mất ô
    def test_neo_lai_cap(self):
        banDo = "#######\n#.....#\n#.....#\n#######"
        grid, _ = parse(banDo)
        shared = SharedState({0: (1, 1), 1: (1, 3)})
        shared.covered.update({(1, c) for c in range(1, 6)} | {(2, 2)})
        shared.addPairs(None, [((1, 3), (2, 3))])
        robots = [SimpleRobot(0, grid, shared), SimpleRobot(1, grid, shared)]
        coord = Coordinator(grid, shared, robots, CoordConfig(maxSteps = 200))
        coord.start()
        coord.failRobot(1)
        self.assertIn(((2, 2), (2, 3)), shared.L)
        kq = coord.run()
        self.assertGreaterEqual(kq['stats']['cap_neo_lai'], 1)
        self.assertEqual(kq['covered'], getFreeCells(grid))

    # robot hỏng khi đang giữ đích: đích được nhả, robot khác phủ nốt
    def test_robot_hong_giu_dich(self):
        grid, viTri = parse(BAN_DO_README)
        for buocHong in range(0, 25):
            coord, robots, kq = runSimple(grid, viTri, failAt = {0: buocHong})
            shared = coord.shared
            cAccSong = computeCAcc(grid, [shared.positions[1]], shared.failedCells)
            self.assertTrue(cAccSong <= shared.covered, buocHong)
            self.assertNotIn(0, shared.reserved)


if __name__ == '__main__':
    unittest.main()
