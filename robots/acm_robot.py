import math
import heapq
from collections import deque

DANH_SACH_HUONG = ['N', 'E', 'S', 'W']
# dựa trên bản đồ thực tế
BUOC_DI = {
    'N': (-1, 0),
    'E': (0, 1),
    'S': (1, 0),
    'W': (0, -1),
}
# chọn hướng Đông làm 0 độ, tự quy ước
GOC_CUA_HUONG = {}
for _h in DANH_SACH_HUONG:
    GOC_CUA_HUONG[_h] = math.atan2(BUOC_DI[_h][0], BUOC_DI[_h][1])

# stage1
# thật sự xoay robot theo kieuQuay
def turnRobot(huong_hien_tai, kieuQuay):
    viTri = DANH_SACH_HUONG.index(huong_hien_tai)
    if kieuQuay == 'L':
        return DANH_SACH_HUONG[(viTri - 1) % 4]
    elif kieuQuay == 'R':
        return DANH_SACH_HUONG[(viTri + 1) % 4]
    elif kieuQuay == 'B':
        return DANH_SACH_HUONG[(viTri + 2) % 4]
    return huong_hien_tai   

# giống y chang turnRobot nhưng chỉ nhìn hướng quay mà ko quay thật
def getAbsoluteDirection(huong_hien_tai, huongTuongDoi):
    if huongTuongDoi == 'F': 
        return huong_hien_tai
    return turnRobot(huong_hien_tai, huongTuongDoi)

# o: vị trí hiện tại, buoc: hướng sắp bước theo, k = 1 mặc định bước 1 bước
def moveCell(o, buoc, k = 1):
    return (o[0] + buoc[0] * k, o[1] + buoc[1] * k)

def getNeighbors4(o):
    dong, cot  = o
    return [(dong - 1, cot), (dong + 1, cot), (dong, cot - 1), (dong, cot + 1)]

def normalizeAngle(goc):
    goc = (goc + math.pi) % (2 * math.pi) - math.pi
    if goc <= - math.pi:
        goc = goc + 2 * math.pi
    return goc

def getFreeCells(grid):
    ketQua = set()
    for dong in range(len(grid)):
        for cot in range(len(grid[0])):
            if grid[dong][cot] == 0:
                ketQua.add((dong, cot))
    return ketQua

class ACM_ROBOT:
    