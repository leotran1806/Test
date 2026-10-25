# demo tầng điều phối với robot giả lập (chưa có ACM)
# chạy: python -m multi_acm.demo   (đứng ở thư mục CHA của multi_acm)
# in chỉ số + lưu hình demo_2_robot.png, demo_4_robot_hong.png

import random

from .simulator import parse, computeCAcc, runSimple, randomBarMap, randomStarts
from .metrics import computeMetrics
from .config import CoordConfig

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


def inKetQua(tieuDe, coord, robots, cAcc):
    m = computeMetrics(coord, robots, cAcc)
    print("=== %s ===" % tieuDe)
    print("CR = %.3f | makespan = %d | L_tot = %.2f | NT_tot = %d | O_inter = %d | du thua = %.3f"
          % (m['CR'], m['makespan'], m['L_tot'], m['NT_tot'], m['O_inter'], m['redundancy']))
    for k, v in m['per_robot'].items():
        print("  robot %d: L(P) = %.2f, NT(P) = %d, tu phu %d o, di %d buoc, cho %d buoc"
              % (k, v['L(P)'], v['NT(P)'], v['so_o_tu_phu'], v['so_buoc_di'], v['so_buoc_cho']))
    print("  thong ke dieu phoi:", {k: v for k, v in m['conflicts'].items() if v > 0})
    print("  chong ket:", {k: v for k, v in m['deadlock'].items() if v > 0})
    print("  cap mat:", m['lost_pairs'])
    return m


def main():
    grid, viTri = parse(BAN_DO_README)
    coord, robots, kq = runSimple(grid, viTri)
    inKetQua("Ban do README, 2 robot", coord, robots, computeCAcc(grid, viTri.values()))

    rng = random.Random(5)
    grid2 = randomBarMap(30, 30, 10, rng)
    viTri2 = randomStarts(grid2, 4, rng)
    coord2, robots2, kq2 = runSimple(grid2, viTri2, CoordConfig(goalRule = 'nearest'), failAt = {2: 40})
    shared2 = coord2.shared
    cAccSong = computeCAcc(grid2, [shared2.positions[k] for k in shared2.liveRobots()], shared2.failedCells)
    inKetQua("Ban do thanh 30x30, 4 robot, robot 2 hong o buoc 40", coord2, robots2, cAccSong)

    try:
        from .viz import plotRun
        plotRun(grid, robots, coord.shared, 'demo_2_robot.png', 'README map - 2 robot')
        plotRun(grid2, robots2, shared2, 'demo_4_robot_hong.png', '30x30 - 4 robot, robot 2 hong')
        print("da luu demo_2_robot.png, demo_4_robot_hong.png")
    except ImportError:
        print("chua cai matplotlib -> bo qua phan ve")


if __name__ == '__main__':
    main()
