# # vẽ kết quả: mỗi robot 1 màu, ô do robot nào phủ tô nhạt màu robot đó, ô robot hỏng đánh dấu X
# # cần matplotlib: pip install matplotlib

# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt
# from matplotlib.colors import to_rgb

# MAU_ROBOT = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple',
#              'tab:brown', 'tab:pink', 'tab:olive', 'tab:cyan', 'tab:gray']


# def plotRun(grid, robots, shared, tenFile, tieuDe = ''):
#     if isinstance(robots, dict):
#         robots = [robots[k] for k in sorted(robots)]
#     soDong, soCot = len(grid), len(grid[0])

#     # nền: vật cản đen, ô trống chưa phủ xám nhạt, ô đã phủ tô nhạt màu robot phủ nó
#     anh = [[(1.0, 1.0, 1.0)] * soCot for _ in range(soDong)]
#     for dong in range(soDong):
#         for cot in range(soCot):
#             if grid[dong][cot] == 1:
#                 anh[dong][cot] = (0.15, 0.15, 0.15)
#             elif (dong, cot) not in shared.covered:
#                 anh[dong][cot] = (0.8, 0.8, 0.8)
#     for k, cacO in shared.coveredBy.items():
#         mau = to_rgb(MAU_ROBOT[k % len(MAU_ROBOT)])
#         mauNhat = tuple(0.7 + 0.3 * c for c in mau)
#         for (dong, cot) in cacO:
#             anh[dong][cot] = mauNhat

#     hinh, truc = plt.subplots(figsize = (max(5, soCot * 0.35), max(4, soDong * 0.35)))
#     truc.imshow(anh, origin = 'upper')
#     for robot in robots:
#         mau = MAU_ROBOT[robot.robotId % len(MAU_ROBOT)]
#         dong = [p[0] for p in robot.path]
#         cot = [p[1] for p in robot.path]
#         truc.plot(cot, dong, '-', color = mau, linewidth = 1.5, label = 'robot %d' % robot.robotId)
#         truc.plot(cot[0], dong[0], 'o', color = mau, markersize = 8)
#         truc.plot(cot[-1], dong[-1], 's', color = mau, markersize = 8)
#     for (dong, cot) in shared.failedCells:
#         truc.plot(cot, dong, 'kx', markersize = 12, markeredgewidth = 3)
#     truc.set_xticks([])
#     truc.set_yticks([])
#     truc.set_title(tieuDe)
#     truc.legend(loc = 'upper left', bbox_to_anchor = (1.01, 1.0), fontsize = 8)
#     hinh.tight_layout()
#     hinh.savefig(tenFile, dpi = 130)
#     plt.close(hinh)
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as patches
import numpy as np

def plotRun(grid, robots, shared, filename, title="Multi-ACM Run"):
    """
    Vẽ bản đồ di chuyển của các robot:
    - Trục tọa độ & lưới caro rõ ràng.
    - Điểm xuất phát (Start: hình tròn 'o') và điểm kết thúc (Finish: ngôi sao '*').
    - Mũi tên chỉ hướng di chuyển dọc theo lộ trình.
    - Đường đi quét mới (Uncovered): nét liền theo màu riêng của từng robot.
    - Đường quay lui (Backtracking): nét đứt màu đỏ riêng biệt.
    """
    rows = len(grid)
    cols = len(grid[0])
    
    # Thiết lập kích thước figure tỉ lệ với bản đồ
    fig, ax = plt.subplots(figsize=(max(6, cols * 0.5), max(6, rows * 0.5)))
    ax.set_title(title, fontsize=14, fontweight='bold', pad=15)

    # 1. Vẽ bản đồ gốc (Grid)
    grid_array = np.array(grid)
    # 0 = Ô trống (trắng), 1 = Vật cản (xám đậm)
    cmap = mcolors.ListedColormap(['#ffffff', '#555555'])
    ax.imshow(grid_array, cmap=cmap, origin='upper', vmin=0, vmax=1)

    # 2. Thiết lập trục tọa độ (Coordinate axes & Grid Lines)
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.set_xticks(np.arange(0, cols, 1))
    ax.set_yticks(np.arange(0, rows, 1))
    ax.grid(which='minor', color='#cccccc', linestyle='-', linewidth=1)
    ax.tick_params(which='minor', size=0)
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)  # y lật ngược vì hàng (row) tăng khi đi xuống

    # Bảng màu cho các robot
    robot_colors = ['#1f77b4', '#2ca02c', '#9467bd', '#ff7f0e', '#8c564b', '#e377c2', '#17becf', '#bcbd22']
    
    if isinstance(robots, dict):
        robot_list = [robots[k] for k in sorted(robots)]
    else:
        robot_list = robots

    # Đánh dấu trạng thái vẽ Legend cho Backtracking (chỉ hiển thị 1 lần trên chú thích)
    added_backtrack_legend = False

    # 3. Vẽ lộ trình di chuyển của từng robot
    for i, robot in enumerate(robot_list):
        color = robot_colors[i % len(robot_colors)]
        path = robot.path
        
        if not path:
            continue
            
        visited = set()
        visited.add((path[0][0], path[0][1]))
        
        # --- Vẽ ĐIỂM XUẤT PHÁT (START) ---
        start_row, start_col = path[0][0], path[0][1]
        ax.plot(start_col, start_row, marker='o', color=color, markersize=8, 
                markeredgecolor='black', zorder=5, label=f'Robot {robot.robotId} (Start)')
        
        # --- Vẽ CÁC BƯỚC DI CHUYỂN & MŨI TÊN ---
        for j in range(len(path) - 1):
            r1, c1, _ = path[j]
            r2, c2, _ = path[j+1]
            
            # Bỏ qua bước đứng yên tại chỗ
            if r1 == r2 and c1 == c2:
                continue
                
            # Kiểm tra xem có phải bước Backtracking (đi vào ô đã từng ghé qua) hay không
            is_backtracking = (r2, c2) in visited
            visited.add((r2, c2))
            
            if is_backtracking:
                line_color = '#d62728'  # Màu đỏ phản quang/nổi bật cho Backtracking
                line_style = '--'
                alpha = 0.85
                zorder = 3
                
                # Thêm chú thích cho đường Backtracking duy nhất 1 lần trên Legend
                if not added_backtrack_legend:
                    ax.plot([], [], color='#d62728', linestyle='--', linewidth=2, label='Backtracking')
                    added_backtrack_legend = True
            else:
                line_color = color
                line_style = '-'
                alpha = 0.9
                zorder = 4
                
            # Vẽ mũi tên chỉ hướng
            arrow = patches.FancyArrowPatch(
                (c1, r1), (c2, r2),
                arrowstyle='-|>',
                color=line_color,
                linestyle=line_style,
                mutation_scale=12,  # Kích thước đầu mũi tên
                linewidth=2,
                alpha=alpha,
                zorder=zorder
            )
            ax.add_patch(arrow)

        # --- Vẽ ĐIỂM KẾT THÚC (FINISH) ---
        end_row, end_col = path[-1][0], path[-1][1]
        ax.plot(end_col, end_row, marker='*', color=color, markersize=14, 
                markeredgecolor='black', zorder=6)

    # 4. Hiển thị ô hỏng (Failed cells) nếu có
    for r, c in shared.failedCells:
        ax.plot(c, r, marker='X', color='black', markersize=12, markeredgewidth=2, zorder=7, label='Robot Failed')

    # Đặt Legend nằm ngoài hình vẽ để tránh che bản đồ
    ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), frameon=True)
    plt.tight_layout()
    
    # Lưu file
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()