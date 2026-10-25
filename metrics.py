# chỉ số đánh giá (kế hoạch Phase 2 mục 9.1, M15-M22)
# robot phải có self.path dạng ACM_LOS_FINAL.py: list cấu hình (dòng, cột, góc)
# CR (M15): tỉ lệ phủ C_acc
# L_tot, NT_tot (M16): tổng độ dài, tổng số lần đổi hướng
# makespan (M17): số bước tới khi dừng (tính cả chờ); max L(P_k)
# O_inter (M18): chồng lấn giữa các robot [UAV (15)(16)]
# redundancy (M19): lặp do quay lui
# load (M20): lệch tải
# điều phối (M22): số xung đột theo loại, bước chờ, chống kẹt

import math

from .grid_utils import normalizeAngle


# L(P) (11) và NT(P) (12), giống getResult() của ACM_LOS_FINAL.py
def computePathMetrics(path):
    tongQuangDuong = 0.0
    for i in range(len(path) - 1):
        dong1, cot1, _ = path[i]
        dong2, cot2, _ = path[i + 1]
        tongQuangDuong = tongQuangDuong + math.hypot(dong2 - dong1, cot2 - cot1)
    soLanQuayDau = 0
    for i in range(len(path) - 1):
        if abs(normalizeAngle(path[i + 1][2] - path[i][2])) > 1e-9:
            soLanQuayDau = soLanQuayDau + 1
    return tongQuangDuong, soLanQuayDau


# các ô robot đi qua theo thứ tự (bỏ cấu hình quay tại chỗ)
def visitedCells(path):
    danhSach = []
    for (dong, cot, _) in path:
        if len(danhSach) == 0 or danhSach[-1] != (dong, cot):
            danhSach.append((dong, cot))
    return danhSach


# cAcc: C_acc dùng làm mẫu số của CR (tính bằng simulator.computeCAcc)
def computeMetrics(coord, robots, cAcc):
    shared = coord.shared
    if isinstance(robots, dict):
        robots = [robots[k] for k in sorted(robots)]

    theoRobot = {}
    soLanGhe = {}                       # {ô: số robot khác nhau từng đi qua}
    tongSoO = 0
    for robot in robots:
        L, NT = computePathMetrics(robot.path)
        cacO = visitedCells(robot.path)
        tongSoO = tongSoO + len(cacO)
        for o in set(cacO):
            soLanGhe[o] = soLanGhe.get(o, 0) + 1
        theoRobot[robot.robotId] = {
            'L(P)': L,
            'NT(P)': NT,
            'so_o_tu_phu': len(shared.coveredBy.get(robot.robotId, ())),
            'so_buoc_di': coord.soBuocDuocDi.get(robot.robotId, 0),
            'so_buoc_cho': coord.soBuocCho.get(robot.robotId, 0),
        }

    danhSachL = [v['L(P)'] for v in theoRobot.values()]
    trungBinhL = sum(danhSachL) / len(danhSachL) if len(danhSachL) > 0 else 0.0
    doLechL = math.sqrt(sum((x - trungBinhL) ** 2 for x in danhSachL) / len(danhSachL)) if len(danhSachL) > 0 else 0.0
    soDaPhu = len(shared.covered)

    return {
        'CR': len(cAcc & shared.covered) / len(cAcc) if len(cAcc) > 0 else 1.0,
        'L_tot': sum(danhSachL),
        'NT_tot': sum(v['NT(P)'] for v in theoRobot.values()),
        'makespan': shared.t,
        'max_L': max(danhSachL) if len(danhSachL) > 0 else 0.0,
        'O_inter': sum(max(0, n - 1) for n in soLanGhe.values()),
        'redundancy': tongSoO / soDaPhu - 1 if soDaPhu > 0 else 0.0,
        'load_std_L': doLechL,
        'load_max_over_mean': max(danhSachL) / trungBinhL if trungBinhL > 0 else 1.0,
        'per_robot': theoRobot,
        'conflicts': dict(coord.thongKe),
        'deadlock': dict(coord.deadlock.thongKe),
        'lost_pairs': len(shared.lostPairs),
    }
