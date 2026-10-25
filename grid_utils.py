# hàm hỗ trợ trên lưới ô, dùng chung cho điều phối và robot giả lập
# quy ước giống ACM_LOS_FINAL.py: grid[dòng][cột] == 1 là vật cản, == 0 là ô trống
# đi 8 hướng (TRAVERSE) thì bước chéo KHÔNG được cắt góc: 2 ô cạnh phải nằm trong vùng cho phép

import math
from collections import deque

# 4 hướng N, S, W, E (công thức (5) bản thảo)
N4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
# 8 hướng: 4 hướng trục trước, 4 hướng chéo sau -> BFS ưu tiên bước thẳng khi hòa
N8 = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


# giống y chang ACM_LOS_FINAL.py
def getNeighbors4(o):
    dong, cot = o
    return [(dong - 1, cot), (dong + 1, cot), (dong, cot - 1), (dong, cot + 1)]

# giống y chang ACM_LOS_FINAL.py
def normalizeAngle(goc):
    goc = (goc + math.pi) % (2 * math.pi) - math.pi
    if goc <= - math.pi:
        goc = goc + 2 * math.pi
    return goc

# góc phải quay từ heading sang beta, chuẩn hóa về [0, pi] (theta khi TRAVERSE, bảng 3.3)
def turnAngle(beta, heading):
    return abs(normalizeAngle(beta - heading))

# giống y chang ACM_LOS_FINAL.py: lấy C_free (2)
def getFreeCells(grid):
    ketQua = set()
    for dong in range(len(grid)):
        for cot in range(len(grid[0])):
            if grid[dong][cot] == 0:
                ketQua.add((dong, cot))
    return ketQua

def isInsideGrid(grid, o):
    return 0 <= o[0] < len(grid) and 0 <= o[1] < len(grid[0])

def isFreeCell(grid, o):
    return isInsideGrid(grid, o) and grid[o[0]][o[1]] == 0


# các ô kề 8 hướng của o nằm trong vungChoPhep, bước chéo ko cắt góc
# vungGoc: tập dùng để xét 2 ô cạnh của bước chéo (mặc định = vungChoPhep)
def getNeighbors8NoCornerCut(o, vungChoPhep, vungGoc = None):
    if vungGoc is None:
        vungGoc = vungChoPhep
    ketQua = []
    for (dDong, dCot) in N8:
        oKe = (o[0] + dDong, o[1] + dCot)
        if oKe not in vungChoPhep:
            continue
        if dDong != 0 and dCot != 0:
            if (o[0] + dDong, o[1]) not in vungGoc:
                continue
            if (o[0], o[1] + dCot) not in vungGoc:
                continue
        ketQua.append(oKe)
    return ketQua


# BFS nhiều nguồn, 8 hướng ko cắt góc, trên vungChoPhep
# trả về set các ô tới được (gồm cả các nguồn)
def bfsReach(danhSachNguon, vungChoPhep):
    daToi = set()
    hangDoi = deque()
    for nguon in danhSachNguon:
        if nguon not in daToi:
            daToi.add(nguon)
            hangDoi.append(nguon)
    while len(hangDoi) > 0:
        o = hangDoi.popleft()
        for oKe in getNeighbors8NoCornerCut(o, vungChoPhep):
            if oKe not in daToi:
                daToi.add(oKe)
                hangDoi.append(oKe)
    return daToi


# BFS 8 hướng ko cắt góc từ batDau tới ô gần nhất (theo số bước) thỏa laDich(o)
# laDich: set các ô đích hoặc hàm o -> bool
# vungGoc: tập xét 2 ô cạnh của bước chéo (mặc định = vungChoPhep)
# trả về list ô [o1, ..., oDich] (KHÔNG gồm batDau), [] nếu batDau đã là đích, None nếu ko tới được
# batDau ko cần nằm trong vungChoPhep
def bfsPathTo(batDau, laDich, vungChoPhep, vungGoc = None):
    if not callable(laDich):
        tapDich = laDich
        laDich = lambda o: o in tapDich
    if laDich(batDau):
        return []
    oCha = {batDau: None}
    hangDoi = deque([batDau])
    while len(hangDoi) > 0:
        o = hangDoi.popleft()
        for oKe in getNeighbors8NoCornerCut(o, vungChoPhep, vungGoc):
            if oKe in oCha:
                continue
            oCha[oKe] = o
            if laDich(oKe):
                duongDi = [oKe]
                while oCha[duongDi[-1]] != batDau:
                    duongDi.append(oCha[duongDi[-1]])
                duongDi.reverse()
                return duongDi
            hangDoi.append(oKe)
    return None


# HỖ TRỢ TẦNG ROBOT (quy tắc 6): tách 1 đoạn Theta* A -> B thành dãy ô kề nhau mà đoạn thẳng đi qua
# cùng ý tưởng getCellsOnLine() của ACM_LOS_FINAL.py nhưng tính t bằng phân số -> chính xác tuyệt đối
# đoạn đi đúng qua góc lưới -> bước chéo (2 ô cạnh chạm góc, canGoStraight của ACM đã kiểm tra chúng)
# trả về list ô [A, ..., B], 2 ô liên tiếp luôn kề 8 hướng
def splitSegmentToCells(A, B):
    from fractions import Fraction
    dong0, cot0 = A
    dong1, cot1 = B
    dDong = dong1 - dong0
    dCot = cot1 - cot0
    danhSachT = {Fraction(0), Fraction(1)}
    if dDong != 0:
        for k in range(min(dong0, dong1), max(dong0, dong1) + 1):
            for j in (Fraction(2 * k - 1, 2), Fraction(2 * k + 1, 2)):
                t = (j - dong0) / dDong
                if 0 < t < 1:
                    danhSachT.add(t)
    if dCot != 0:
        for k in range(min(cot0, cot1), max(cot0, cot1) + 1):
            for j in (Fraction(2 * k - 1, 2), Fraction(2 * k + 1, 2)):
                t = (j - cot0) / dCot
                if 0 < t < 1:
                    danhSachT.add(t)
    danhSachT = sorted(danhSachT)
    ketQua = [tuple(A)]
    for i in range(len(danhSachT) - 1):
        t = (danhSachT[i] + danhSachT[i + 1]) / 2
        o = (math.floor(dong0 + dDong * t + Fraction(1, 2)), math.floor(cot0 + dCot * t + Fraction(1, 2)))
        if o != ketQua[-1]:
            ketQua.append(o)
    if ketQua[-1] != tuple(B):
        ketQua.append(tuple(B))
    return ketQua

# tách cả đường Theta* [c_b0, ..., c_bm] (38) thành dãy ô cho các bước TRAVERSE (KHÔNG gồm ô đầu)
def splitPathToCells(duongDiTheta):
    ketQua = []
    for i in range(len(duongDiTheta) - 1):
        ketQua.extend(splitSegmentToCells(duongDiTheta[i], duongDiTheta[i + 1])[1:])
    return ketQua


# 2 ô cạnh của bước chéo A -> B (bước thẳng -> [])
def diagonalSideCells(A, B):
    if abs(A[0] - B[0]) == 1 and abs(A[1] - B[1]) == 1:
        return [(B[0], A[1]), (A[0], B[1])]
    return []

# "dấu chân" của đường đi: các ô trên đường + (nếu tinhGoc) 2 ô cạnh của mỗi bước chéo
def pathFootprint(viTri, duongDi, tinhGoc = False):
    ketQua = set(duongDi)
    if tinhGoc:
        truoc = viTri
        for o in duongDi:
            ketQua.update(diagonalSideCells(truoc, o))
            truoc = o
    return ketQua


# tổng độ dài Euclid của dãy ô bắt đầu từ viTri (dùng tính v khi TRAVERSE, bảng 3.3)
def pathLength(viTri, duongDi):
    tong = 0.0
    truoc = viTri
    for o in duongDi:
        tong = tong + math.hypot(o[0] - truoc[0], o[1] - truoc[1])
        truoc = o
    return tong
