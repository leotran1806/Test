# mô phỏng: đọc / sinh bản đồ, tính C_acc, chạy nhanh với robot giả lập
# quy ước bản đồ dạng chữ (parse): '#' vật cản, '.' ô trống, chữ số '0'..'9' = ô xuất phát của robot có ID đó

import random
from collections import deque

from .shared_state import SharedState
from .config import CoordConfig
from .coordinator import Coordinator
from .grid_utils import getFreeCells, getNeighbors4, isFreeCell
from .robots.simple_robot import SimpleRobot


# banDoChu: chuỗi nhiều dòng hoặc list chuỗi -> (grid, {id: ô xuất phát})
def parse(banDoChu):
    if isinstance(banDoChu, str):
        danhSachDong = [d.strip() for d in banDoChu.strip().splitlines() if d.strip() != '']
    else:
        danhSachDong = list(banDoChu)
    soCot = len(danhSachDong[0])
    grid = []
    viTriXuatPhat = {}
    for dong, chuoi in enumerate(danhSachDong):
        if len(chuoi) != soCot:
            raise ValueError("dong %d dai %d, khac %d" % (dong, len(chuoi), soCot))
        hang = []
        for cot, kyTu in enumerate(chuoi):
            if kyTu == '#':
                hang.append(1)
            elif kyTu == '.':
                hang.append(0)
            elif kyTu.isdigit():
                hang.append(0)
                viTriXuatPhat[int(kyTu)] = (dong, cot)
            else:
                raise ValueError("ky tu la '%s' tai (%d, %d)" % (kyTu, dong, cot))
        grid.append(hang)
    return grid, dict(sorted(viTriXuatPhat.items()))


# C_acc (M1): ô trống tới được (4 hướng) từ các ô xuất phát, oCam (vd ô robot hỏng) coi là vật cản
# chỉ dùng để ĐÁNH GIÁ, robot ko biết C_acc
def computeCAcc(grid, danhSachNguon, oCam = ()):
    oCam = set(oCam)
    daToi = set()
    hangDoi = deque()
    for o in danhSachNguon:
        if isFreeCell(grid, o) and o not in oCam and o not in daToi:
            daToi.add(o)
            hangDoi.append(o)
    while len(hangDoi) > 0:
        o = hangDoi.popleft()
        for oKe in getNeighbors4(o):
            if oKe not in daToi and oKe not in oCam and isFreeCell(grid, oKe):
                daToi.add(oKe)
                hangDoi.append(oKe)
    return daToi


# bản đồ thanh kiểu BoB: viền là vật cản, soThanh thanh rộng 1 ô dài 5-10 ô, ngang hoặc dọc
def randomBarMap(soDong = 30, soCot = 30, soThanh = None, rng = None):
    if rng is None:
        rng = random.Random()
    if soThanh is None:
        soThanh = rng.randint(8, 12)
    grid = [[0] * soCot for _ in range(soDong)]
    for dong in range(soDong):
        grid[dong][0] = 1
        grid[dong][soCot - 1] = 1
    for cot in range(soCot):
        grid[0][cot] = 1
        grid[soDong - 1][cot] = 1
    for _ in range(soThanh):
        doDai = rng.randint(5, 10)
        if rng.random() < 0.5:
            dong = rng.randint(1, soDong - 2)
            cotDau = rng.randint(1, max(1, soCot - 1 - doDai))
            for cot in range(cotDau, min(soCot - 1, cotDau + doDai)):
                grid[dong][cot] = 1
        else:
            cot = rng.randint(1, soCot - 2)
            dongDau = rng.randint(1, max(1, soDong - 1 - doDai))
            for dong in range(dongDau, min(soDong - 1, dongDau + doDai)):
                grid[dong][cot] = 1
    return grid


# mê cung hành lang rộng 1 ô (DFS quay lui), kích thước (2*soDongO + 1) x (2*soCotO + 1)
# tiLeLoThong: tỉ lệ tường trong được đục thêm -> có vòng, nhiều đường
def randomMazeMap(soDongO, soCotO, rng = None, tiLeLoThong = 0.0):
    if rng is None:
        rng = random.Random()
    soDong = 2 * soDongO + 1
    soCot = 2 * soCotO + 1
    grid = [[1] * soCot for _ in range(soDong)]
    batDau = (0, 0)
    daTham = {batDau}
    nganXep = [batDau]
    grid[1][1] = 0
    while len(nganXep) > 0:
        dong, cot = nganXep[-1]
        danhSachKe = []
        for (dDong, dCot) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            oKe = (dong + dDong, cot + dCot)
            if 0 <= oKe[0] < soDongO and 0 <= oKe[1] < soCotO and oKe not in daTham:
                danhSachKe.append(oKe)
        if len(danhSachKe) == 0:
            nganXep.pop()
            continue
        oKe = rng.choice(danhSachKe)
        daTham.add(oKe)
        grid[2 * oKe[0] + 1][2 * oKe[1] + 1] = 0
        grid[dong + oKe[0] + 1][cot + oKe[1] + 1] = 0
        nganXep.append(oKe)
    if tiLeLoThong > 0:
        for dong in range(1, soDong - 1):
            for cot in range(1, soCot - 1):
                if grid[dong][cot] == 1 and (dong % 2 == 1) != (cot % 2 == 1):
                    if rng.random() < tiLeLoThong:
                        grid[dong][cot] = 0
    return grid


# chọn soRobot ô xuất phát khác nhau, cùng thành phần liên thông lớn nhất
def randomStarts(grid, soRobot, rng = None):
    if rng is None:
        rng = random.Random()
    oTrong = getFreeCells(grid)
    conLai = set(oTrong)
    thanhPhanLonNhat = set()
    while len(conLai) > 0:
        o = min(conLai)
        thanhPhan = computeCAcc(grid, [o])
        conLai = conLai - thanhPhan
        if len(thanhPhan) > len(thanhPhanLonNhat):
            thanhPhanLonNhat = thanhPhan
    if len(thanhPhanLonNhat) < soRobot:
        raise ValueError("khong du o trong cho %d robot" % (soRobot,))
    danhSach = rng.sample(sorted(thanhPhanLonNhat), soRobot)
    return {k: o for k, o in enumerate(danhSach)}


# chạy nhanh với robot giả lập SimpleRobot
# trả về (coordinator, {id: robot}, kết quả)
# rng: truyền vào để robot chọn ngẫu nhiên (stress test)
def runSimple(grid, viTriXuatPhat, config = None, failAt = None, huongBanDau = 'N', tamNhin = 3, rng = None):
    if config is None:
        config = CoordConfig()
    if not isinstance(viTriXuatPhat, dict):
        viTriXuatPhat = {k: o for k, o in enumerate(viTriXuatPhat)}
    shared = SharedState(viTriXuatPhat)
    danhSachRobot = [SimpleRobot(k, grid, shared, huongBanDau = huongBanDau, tamNhin = tamNhin,
                                 rng = None if rng is None else random.Random(rng.random()))
                     for k in sorted(viTriXuatPhat)]
    coord = Coordinator(grid, shared, danhSachRobot, config)
    ketQua = coord.run(failAt = failAt)
    return coord, {r.robotId: r for r in danhSachRobot}, ketQua
