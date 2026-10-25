# stress test (chạy tay): mê cung hành lang 1 ô (có / ko có lỗ thông), bản đồ thanh kiểu BoB,
# 1-8 robot, ~40% lần chạy có 1 tới n-1 robot hỏng ở bước ngẫu nhiên
# kiểm tra mỗi lần chạy: dừng (ko vượt maxSteps), ko ContractError / InvariantError (bất biến bật),
#                        phủ hết C'_acc (ô trống tới được từ robot sống, coi ô robot hỏng là vật cản)
# chạy: python -m multi_acm.tests.stress [soLan] [seed]

import sys
import time
import random
import traceback

from multi_acm.simulator import (randomBarMap, randomMazeMap, randomStarts, computeCAcc, runSimple)
from multi_acm.config import CoordConfig


def taoKichBan(rng):
    while True:
        try:
            return taoKichBanMotLan(rng)
        except ValueError:
            continue                                   # bản đồ quá vụn, ko đủ chỗ cho robot -> sinh lại


def taoKichBanMotLan(rng):
    loai = rng.choice(['maze', 'maze_lo', 'bar', 'bar_nho', 'bar_lon'])
    if loai == 'maze':
        grid = randomMazeMap(rng.randint(3, 7), rng.randint(3, 7), rng)
    elif loai == 'maze_lo':
        grid = randomMazeMap(rng.randint(3, 7), rng.randint(3, 7), rng, tiLeLoThong = rng.choice([0.05, 0.15, 0.3]))
    elif loai == 'bar':
        grid = randomBarMap(rng.randint(12, 24), rng.randint(12, 24), None, rng)
    elif loai == 'bar_nho':
        grid = randomBarMap(rng.randint(8, 12), rng.randint(8, 12), rng.randint(2, 6), rng)
    else:
        grid = randomBarMap(30, 30, None, rng)
    soRobot = rng.randint(1, 8)
    viTri = randomStarts(grid, soRobot, rng)
    failAt = {}
    if soRobot >= 2 and rng.random() < 0.4:
        soHong = rng.randint(1, soRobot - 1)
        for k in rng.sample(sorted(viTri), soHong):
            failAt[k] = rng.randint(0, 60)
    config = CoordConfig(goalRule = rng.choice(['paper', 'nearest']),
                         filterNearCover = rng.random() < 0.3,
                         strictDiagonal = rng.random() < 0.5,
                         maxSteps = 20000)
    ngauNhien = rng.random() < 0.5                     # robot chọn hướng / đích ngẫu nhiên
    return loai, grid, viTri, failAt, config, ngauNhien


def chayMotLan(seed):
    rng = random.Random(seed)
    loai, grid, viTri, failAt, config, ngauNhien = taoKichBan(rng)
    coord, robots, kq = runSimple(grid, viTri, config, failAt, rng = rng if ngauNhien else None)
    shared = coord.shared
    viTriSong = [shared.positions[k] for k in shared.liveRobots()]
    cAccSong = computeCAcc(grid, viTriSong, shared.failedCells)
    thieu = cAccSong - shared.covered
    if len(failAt) == 0:
        cAcc = computeCAcc(grid, viTri.values())
        if shared.covered != cAcc:
            return False, "phu khac C_acc (thieu %d, thua %d)" % (len(cAcc - shared.covered), len(shared.covered - cAcc)), kq
    if len(thieu) > 0:
        return False, "sot %d o cua C'_acc: %s" % (len(thieu), sorted(thieu)[:5]), kq
    return True, loai, kq


def main():
    soLan = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    seedGoc = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    batDau = time.time()
    soLoi = 0
    tongNhanh = {}
    buocMax = 0
    for i in range(soLan):
        seed = seedGoc + i
        try:
            ok, thongTin, kq = chayMotLan(seed)
        except Exception:
            soLoi = soLoi + 1
            print("seed %d: LOI" % seed)
            traceback.print_exc()
            continue
        if not ok:
            soLoi = soLoi + 1
            print("seed %d: %s" % (seed, thongTin))
            continue
        buocMax = max(buocMax, kq['steps'])
        for ten, gt in list(kq['stats'].items()) + list(kq['deadlock_stats'].items()):
            if gt > 0:
                tongNhanh[ten] = tongNhanh.get(ten, 0) + 1
        if len(kq['lost_pairs']) > 0:
            tongNhanh['cap_mat'] = tongNhanh.get('cap_mat', 0) + 1
    print("so lan: %d, loi: %d, buoc max: %d, thoi gian: %.1fs" % (soLan, soLoi, buocMax, time.time() - batDau))
    print("so lan chay co kich hoat tung nhanh:", dict(sorted(tongNhanh.items())))
    return soLoi


if __name__ == '__main__':
    sys.exit(1 if main() > 0 else 0)
