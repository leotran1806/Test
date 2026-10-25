# Multi-ACM: tầng điều phối (Kiên)

hiện thực đúng file `dac_ta_tang_dieu_phoi.md` (đặc tả v1), đặt tên + phong cách theo `ACM_LOS_FINAL.py`

> lưu ý: code này mô tả nội dung bản thảo ACM chưa công bố -> chỉ dùng trong nhóm, repo **private**, ko push public lên GitHub, gửi thẳng cho thầy/cô

---

## 1. Chạy thử

đứng ở thư mục **cha** của `multi_acm/` (vd thư mục chứa cả `multi_acm/` và `ACM_LOS_FINAL.py`)

```
python -m unittest discover -s multi_acm/tests -t .      # 55 test, ~6 giây
python -m multi_acm.tests.stress 2000 0                  # stress: 2000 lần chạy ngẫu nhiên, seed bắt đầu 0
python -m multi_acm.demo                                 # in chỉ số + lưu 2 hình .png (cần matplotlib)
```

- Python 3.8+. Lõi chỉ dùng thư viện chuẩn (`math`, `heapq`, `collections`, `dataclasses`, `enum`, `fractions`)
- `viz.py` cần `pip install matplotlib`

ví dụ dùng nhanh:

```python
from multi_acm.simulator import parse, runSimple
from multi_acm.config import CoordConfig

grid, viTri = parse("""
##########
#0.......#
#.##..#..#
#.#...#..#
#...#....#
#...#.##.#
#.#.....1#
##########
""")                                            # '#' vật cản, '.' trống, chữ số = ô xuất phát robot có ID đó
coord, robots, ketQua = runSimple(grid, viTri, CoordConfig(goalRule = 'paper'), failAt = {1: 10})
print(ketQua['steps'], len(ketQua['covered']), ketQua['stats'], ketQua['deadlock_stats'])
```

---

## 2. Cấu trúc thư mục (đúng mục 2 đặc tả)

```
multi_acm/
├── protocol.py        # HỢP ĐỒNG 2 tầng: Mode, Kind, Reason, Intent, Response, ContractError, RobotProtocol
├── config.py          # CoordConfig
├── grid_utils.py      # N4, N8, BFS 8 hướng ko cắt góc, + splitPathToCells() cho tầng robot
├── shared_state.py    # SharedState: covered, L chung, đặt chỗ, robot hỏng, tabu
├── conflict.py        # validateIntent, moveKey, goalKey, resolveGoals (X4), resolveMoves (X1, X2, X2', X3, khóa)
├── deadlock.py        # DeadlockManager: R0, R1 + khóa, R2a, R2b
├── invariants.py      # I1-I6
├── coordinator.py     # Coordinator: start, step (7 pha), run(failAt), failRobot, điều kiện dừng
├── simulator.py       # parse, computeCAcc, randomBarMap, randomMazeMap, randomStarts, runSimple
├── metrics.py         # M15-M22
├── viz.py             # vẽ mỗi robot 1 màu
├── demo.py
├── robots/
│   ├── simple_robot.py  # robot giả lập = VÍ DỤ MẪU tuân thủ hợp đồng
│   ├── goal_robot.py    # robot đi A -> B (test kẹt / nhường đường)
│   └── acm_robot.py     # <- Phát viết (ACM thật dạng generator)
└── tests/
    ├── test_conflict.py   # đơn vị (mục 12.1) + strictDiagonal
    ├── test_scenarios.py  # kịch bản (mục 12.2) + chu trình 4 robot, T13, cặp mất, neo lại cặp
    ├── test_random.py     # ngẫu nhiên (mục 12.3)
    ├── test_grid_utils.py # tách đoạn Theta* thành dãy ô, so với canGoStraight của ACM
    └── stress.py          # chạy tay hàng nghìn lần
```

---

## 3. Bảng đổi tên: đặc tả (snake_case) -> code (camelCase như `ACM_LOS_FINAL.py`)

đặc tả viết snake_case, code đổi sang camelCase cho giống `ACM_LOS_FINAL.py` (`moveToCell`, `findBacktrackPairs`...). Ý nghĩa giữ nguyên 100%

| đặc tả | code |
|---|---|
| `Intent.robot_id` | `Intent.robotId` |
| `force_step(cell)` | `forceStep(o)` |
| `shared.pairs` | `shared.L` (giống `self.L` của ACM) |
| `covered_by`, `lost_pairs`, `goal_owner`, `failed_cells`, `soon_covered` | `coveredBy`, `lostPairs`, `goalOwner`, `failedCells`, `soonCovered` |
| `add_pairs`, `planning_pairs`, `goal_alive`, `first_pair_at` | `addPairs`, `planningPairs`, `goalAlive`, `firstPairAt` |
| `active_pairs`, `delete_pairs_with_cu`, `add_tabu` | `activePairs`, `deletePairsWithCu`, `addTabu` |
| `traversable`, `reserve`, `release` | giữ nguyên |
| `validate_intent`, `move_key`, `goal_key`, `turn_angle` | `validateIntent`, `moveKey`, `goalKey`, `turnAngle` |
| `resolve_goals`, `resolve_moves` | `resolveGoals`, `resolveMoves` |
| `update_waits`, `yield_plan`, `forced_intent`, `on_forced_result` | `updateWaits`, `yieldPlan`, `forcedIntent`, `onForcedResult` |
| `locked_cells`, `release_locks` | `lockedCells`, `releaseLocks` |
| `fail_robot`, `unreachable_pairs`, `_check_termination` | `failRobot`, `unreachablePairs`, `_checkTermination` |
| `goal_rule`, `theta_eps`, `deadlock_K`, `tabu_steps` | `goalRule`, `thetaEps`, `deadlockK`, `tabuSteps` |
| `lock_max_steps`, `max_steps`, `check_invariants`, `filter_near_cover` | `lockMaxSteps`, `maxSteps`, `checkInvariants`, `filterNearCover` |
| `free_cells`, `c_acc`, `random_bar_map`, `run_simple` | `getFreeCells` (giống ACM), `computeCAcc`, `randomBarMap`, `runSimple` |
| `neighbors8_no_corner_cut`, `bfs_reach`, `bfs_path_to` | `getNeighbors8NoCornerCut`, `bfsReach`, `bfsPathTo` |

tên kiểu (`Mode`, `Kind`, `Reason`, `Intent`, `Response`, `SharedState`, `Coordinator`, `DeadlockManager`, `CoordConfig`) và giá trị enum (`COVER`, `UNCOVERED`, `X1_LOST`...) giữ đúng đặc tả

---

## 4. Hướng dẫn cho Phát: viết `robots/acm_robot.py`

### 4.1. Khung generator (giữ gần nguyên `ACM_LOS_FINAL.py`)

ý tưởng: mọi chỗ ACM **thật sự di chuyển** (`moveToCell`, từng bước trong `followPath`, `enterBranch`) đổi thành `yield from self.stepTo(...)`. Các hàm cảm biến, chọn hướng, hole, `thetaStar`, `canGoStraight` dùng lại nguyên

```python
from ..protocol import Kind, Mode, Reason, Intent, makeStayIntent, getStepAngle
from ..grid_utils import splitPathToCells, turnAngle, pathLength

class ACMRobot(ACM_LOS):
    def __init__(self, robotId, grid, shared, rho = 3.0, huongBanDau = 'N'):
        super().__init__(grid, shared.positions[robotId], rho, huongBanDau)
        self.robotId = robotId
        self.shared = shared
        self.covered = shared.covered        # TRỎ CHUNG (M2)(M3) -> isBlocked, measureDirection... tự dùng bản đồ chung
                                             # KHÔNG bao giờ gán lại self.covered, KHÔNG tự .add()

    # 1 bước = 1 lần yield. Trả về Response cuối cùng
    def stepTo(self, o, kind, mode, v, theta, goal = None, path = ()):
        while True:
            resp = yield Intent(self.robotId, kind, o, mode, v, theta, goal, tuple(path))
            if resp.ok:
                self.pos = o                       # chỉ cập nhật pos + path riêng (quy tắc 4)
                self.path.append((o[0], o[1], self.path[-1][2]))
                return resp
            if resp.replan:
                return resp                        # GOAL_TAKEN / GOAL_DEAD / ABORT / YIELDED / OCCUPIED+replan
            if kind == Kind.UNCOVERED and resp.reason == Reason.X1_LOST:
                return resp                        # ô đã bị robot khác phủ -> đo lại l_t, chọn lại theo ACM
            # OCCUPIED / CROSSING / LOCKED (replan = False): thử lại CÙNG ô ở bước sau

    def forceStep(self, o):                         # quy tắc 9: ko yield, ko phát hiện cặp
        goc = getStepAngle(self.pos, o)
        self.turnToAngle(goc)
        self.pos = o
        self.path.append((o[0], o[1], goc))
        # heading giờ có thể chéo -> trước khi quét lại phải đưa self.heading về N/E/S/W (quy tắc 8)

    def run(self):                                  # generator
        ...  # viết lại run() / runStraight() / coverHole() / globalBacktrack() với yield from self.stepTo(...)
        # hết việc: while True: yield makeStayIntent(self.robotId, Mode.WAIT, idle = True) rồi KIỂM TRA LẠI
        #           planningPairs mỗi bước (robot khác có thể vừa thêm cặp / vừa chuyển đích cho mình - R2a)
```

### 4.2. Những dòng PHẢI sửa trong `ACM_LOS_FINAL.py`

| chỗ | sửa thành |
|---|---|
| `findBacktrackPairs`: `self.L.append(i)` | gom cặp mới vào list rồi `self.shared.addPairs(self.robotId, capMoi)` |
| `moveToCell`: `self.covered.add(o)` + `self.L = capConLai` | bỏ hết, điều phối đã làm (chỉ còn cập nhật `pos`, `path`) |
| `globalBacktrack`: `self.L = capConHieuLuc`, `if duongDi is None: self.L = []` | bỏ. Lấy đích từ `self.shared.planningPairs(self.robotId)`; ko tới được -> STAY idle |
| `globalBacktrack`: chọn `oCanGhe` từ `self.L` | `self.shared.firstPairAt(c_b)[1]` |
| `thetaStar(dichDen)` | `self.thetaStar(dichDen, self.shared.traversable())` |
| `followPath`: `self.pos = oKeTiep` nhảy cả đoạn | `for o in splitPathToCells(duongDi): yield from self.stepTo(o, Kind.TRAVERSE, ...)`; `path` của Intent = các ô còn lại; `goal` = c_b trên MỌI bước, kể cả bước `enterBranch` |
| `runStraight`: gọi `findBacktrackPairs()` trước khi bước | vẫn được, vì mỗi lần chạy lại vòng lặp là SAU khi bước đã được chấp nhận (quy tắc 7) |
| `coverHole/ghiCapCucBo`: `self.L[soCapCu:] = ...` | ko sửa thẳng `self.L`; lọc cặp trong list tạm rồi `addPairs` |
| `coverHole`: nhận ABORT khi quay lui cục bộ | `addPairs` các cặp cục bộ còn mở rồi rời hole |
| quay về `q_resume` (mode `RETURN`) nhận ABORT | coi `q_resume` đã cạn |

### 4.3. v, theta (bảng 3.3 đặc tả)

| hành động | `kind` | `mode` | `v` | `theta` |
|---|---|---|---|---|
| quét, đi tiếp F | UNCOVERED | COVER | `measureDirection('F')` | 0 |
| quét, rẽ L/R | UNCOVERED | COVER | `doDai[a*]` | pi/2 |
| quay đầu ở ô xuất phát | UNCOVERED | COVER | ℓ_t(B) | pi |
| vào nhánh c_b -> c_u | UNCOVERED | BACKTRACK | số ô trống chưa phủ liên tiếp từ c_b theo hướng c_b -> c_u (>= 1) | `abs(alpha)` (43) |
| bước trong hole | UNCOVERED | HOLE | như quét, chỉ đếm ô thuộc H | như quét |
| quay lui toàn cục | TRAVERSE | BACKTRACK | `pathLength` Euclid còn lại theo đỉnh Theta* | `turnAngle(beta, gocHienTai)` |
| quay lui cục bộ trong hole | TRAVERSE | HOLE | như trên | như trên |
| về q_resume | TRAVERSE | RETURN | như trên | như trên |
| chờ / rảnh | STAY | WAIT | 0 | 0 |

`goal` chỉ khác `None` khi `mode == BACKTRACK` (validateIntent sẽ báo lỗi nếu sai)

### 4.4. Checklist ghép (mục 13.2 đặc tả + bổ sung)

- [ ] **n = 1** cho đúng kết quả `ACM_LOS_FINAL.py`: 38/38 ô, L(P) = 41,414, NT(P) = 18, 1 quay lui toàn cục, 2 hole (bản đồ README). Với n = 1 điều phối luôn trả OK (đã có test `test_mot_robot_luon_OK`)
- [ ] ko còn dòng nào gán lại `self.L = ...` hoặc `self.covered = ...`
- [ ] `moveToCell` ko tự sửa `covered` / cặp
- [ ] `thetaStar` dùng `shared.traversable()`; đường tách bằng `splitPathToCells` (đã test: mọi bước kề 8 hướng, bước chéo ko cắt góc theo đúng `canGoStraight`)
- [ ] `goal` có trên mọi bước quay lui, kể cả bước vào nhánh
- [ ] xử lý đủ 8 dòng bảng 3.4 (nhất là ABORT trong hole)
- [ ] heading đưa về trục trước khi quét lại (sau bước chéo / sau `forceStep`)
- [ ] robot rảnh kiểm tra lại `planningPairs` **mỗi bước** (R2a có thể chuyển đích cho nó)
- [ ] chạy `stress.py` với `ACMRobot` thay `SimpleRobot` (sửa `runSimple`) ko còn `ContractError`, `InvariantError`

---

## 5. Chỗ code BỔ SUNG / KHÁC đặc tả (cần báo cô)

mục 14 đặc tả (7 chỗ lệch so với ĐX) giữ nguyên. Ngoài ra code có thêm:

| # | đặc tả | code | lý do |
|---|---|---|---|
| B1 | R1 chỉ xét "đối đầu" `blocker[j] = k` | xét **chu trình chặn nhau bất kỳ** (đối đầu = chu trình độ dài 2) | 4 robot quay vòng trong khối 2x2 (test `test_chu_trinh_4_robot`) kẹt vĩnh viễn nếu chỉ xét độ dài 2 |
| B2 | j "rảnh" = đề xuất STAY | thêm: j bị từ chối **LOCKED** cũng coi là đứng yên | ko thì j (đang bị khóa) và robot chặn j có thể chờ nhau tới hết `lockMaxSteps` |
| B3 | ko có | robot hỏng đứng trên c_b -> **neo lại** cặp vào ô đã phủ khác kề cạnh c_u; lúc dừng cũng thử neo lại cặp ko tới được trước khi kết luận mất | giữ bảo đảm phủ hết C'_acc khi lối vào c_u còn chỗ khác. Chỉ dùng `covered` + cặp, ko dùng bản đồ |
| B4 | `goal_key nearest = (v, ID)` | robot đứng ngay tại c_b (bước vào nhánh) tính khoảng cách 0 | lúc vào nhánh `v` là độ dài đoạn thẳng, ko phải khoảng cách |
| B5 | ko nói | robot đổi sang đích khác -> **nhả đích cũ ngay** ở pha 2; robot đề xuất đích đang bị tabu với nó -> GOAL_TAKEN | tránh giữ chỗ "treo"; phòng tầng robot ko lọc tabu |
| B6 | ko có | `strictDiagonal` (mặc định **tắt** = đúng đặc tả): robot ở ô cạnh bước chéo cũng là xung đột | thân robot đi chéo sẽ chạm robot ở ô cạnh, giống luật ko cắt góc với vật cản. Bật khi muốn mô hình thân robot chặt hơn |
| B7 | `ContractError` trong `conflict.py` | định nghĩa ở `protocol.py`, `conflict.py` import lại | `shared_state.addPairs` cũng cần ném lỗi mà ko được import `conflict` (tránh vòng import) |
| B8 | làm tròn `theta` | làm tròn cả `v` theo `thetaEps` | `v` là độ dài Euclid, 2 robot cách đều có thể lệch 1e-15 |
| B9 | `fail_robot` hủy kế hoạch né liên quan | robot đang né bị hủy kế hoạch -> **resume ngay** trong `failRobot` với YIELDED để lấy Intent mới | stress tìm ra: nếu để tới bước sau, pha 1 dùng Intent CŨ (tính trước khi bị dời) -> robot "nhảy" ô. Pha 1 và pha 4 có thêm kiểm tra chặn lỗi này (`RuntimeError`) |

---

## 6. Kết quả kiểm thử (máy nhóm chạy lại để xác nhận)

- 55 unittest: đạt hết (có test hồi quy cho B9: `test_hong_khi_dang_nhuong_duong`)
- stress trên code cuối: **24.000 lần chạy** (4 x 6.000, seed 100000-123999) + chạy lại dải seed 50000-59999 nơi từng tìm ra lỗi B9. Kịch bản: mê cung 1 ô có / ko lỗ thông, bản đồ thanh 8x8 tới 30x30, 1-8 robot, ~40% lần chạy có 1 tới n-1 robot hỏng, 2 luật X4, bật/tắt bộ lọc, bật/tắt `strictDiagonal`, một nửa số lần robot chọn hướng / đích ngẫu nhiên. Kết quả: **0 lỗi** (ko kẹt, ko sót ô C'_acc, ko vi phạm bất biến, ko ContractError), tối đa ~1.660 bước. Mọi nhánh X1, X2', X3, X4, LOCKED, GOAL_DEAD, R0, R1, R2a, R2b, hủy nhường đường, cặp mất đều được kích hoạt hàng trăm tới hàng nghìn lần
- chuỗi bước dài nhất mà **ko robot nào di chuyển**: 4 bước (1.500 lần chạy) -> ko phải dựa vào `lockMaxSteps` hết hạn để thoát kẹt
- bản đồ 50x50, 8 robot: ~0,5 giây

### hạn chế (mục 15 đặc tả + thêm)

1. chưa có chứng minh hình thức thang R0-R2b luôn thoát kẹt (bằng chứng là thực nghiệm)
2. luật bảo thủ -> robot đi nối đuôi cách 1 ô
3. robot hỏng chắn lối duy nhất -> vùng đó ko phủ được, thuật toán vẫn dừng và báo cặp mất (`test_cap_mat_khi_hong_chan_loi`)
4. điều phối trung tâm là điểm lỗi duy nhất; truyền thông hoàn hảo, đồng bộ
5. mỗi robot chỉ giữ 1 khóa nhường đường; robot bị chọn né lần 2 thì khóa cũ bị thay (chưa thấy gây kẹt trong stress)
6. stress hiện chạy với robot giả lập; phải chạy lại sau khi có `acm_robot.py`
