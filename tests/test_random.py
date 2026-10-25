# kiểm thử ngẫu nhiên (đặc tả mục 12.3)
# - 1, 2, 3, 4, 6 robot x 2 luật X4 x bản đồ thanh, bật/tắt bộ lọc: covered == C_acc
# - 2-4 robot, 1 robot hỏng ở bước ngẫu nhiên: phủ hết C'_acc
# - mê cung hành lang 1 ô (dễ kẹt nhất) + strictDiagonal
# chạy: python -m unittest multi_acm.tests.test_random -v

import random
import unittest

from multi_acm.config import CoordConfig
from multi_acm.simulator import (randomBarMap, randomMazeMap, randomStarts, computeCAcc, runSimple)

SO_BAN_DO = 12          # tăng lên (vd 60) khi muốn chạy kỹ


class TestRandom(unittest.TestCase):
    def test_phu_du_C_acc(self):
        rng = random.Random(2026)
        for i in range(SO_BAN_DO):
            grid = randomBarMap(20, 20, None, rng)
            for soRobot in (1, 2, 3, 4, 6):
                viTri = randomStarts(grid, soRobot, rng)
                cAcc = computeCAcc(grid, viTri.values())
                for luat in ('paper', 'nearest'):
                    for boLoc in (False, True):
                        config = CoordConfig(goalRule = luat, filterNearCover = boLoc, maxSteps = 5000)
                        coord, robots, kq = runSimple(grid, viTri, config)
                        self.assertEqual(kq['covered'], cAcc, (i, soRobot, luat, boLoc))
                        self.assertEqual(kq['lost_pairs'], set())

    def test_robot_hong(self):
        rng = random.Random(7)
        for i in range(SO_BAN_DO * 3):
            grid = randomBarMap(16, 16, None, rng)
            soRobot = rng.randint(2, 4)
            viTri = randomStarts(grid, soRobot, rng)
            failAt = {rng.choice(sorted(viTri)): rng.randint(0, 40)}
            coord, robots, kq = runSimple(grid, viTri, CoordConfig(maxSteps = 5000), failAt)
            shared = coord.shared
            cAccSong = computeCAcc(grid, [shared.positions[k] for k in shared.liveRobots()], shared.failedCells)
            self.assertTrue(cAccSong <= shared.covered, (i, cAccSong - shared.covered))

    def test_me_cung(self):
        rng = random.Random(99)
        for i in range(SO_BAN_DO * 2):
            grid = randomMazeMap(rng.randint(3, 6), rng.randint(3, 6), rng, tiLeLoThong = rng.choice([0.0, 0.1, 0.3]))
            soRobot = rng.randint(2, 8)
            viTri = randomStarts(grid, soRobot, rng)
            for strict in (False, True):
                config = CoordConfig(goalRule = rng.choice(['paper', 'nearest']), strictDiagonal = strict, maxSteps = 5000)
                coord, robots, kq = runSimple(grid, viTri, config, rng = random.Random(i))
                self.assertEqual(kq['covered'], computeCAcc(grid, viTri.values()), (i, strict))


if __name__ == '__main__':
    unittest.main()
