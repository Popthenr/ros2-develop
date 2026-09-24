import subprocess
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button


# ============================================================
# 基本设置
# ============================================================

PROJECT_DIR = Path(__file__).resolve().parent.parent
ASTAR_PROGRAM = PROJECT_DIR / "build" / "astar"
MAP_FILE = PROJECT_DIR / "map.txt"
PATH_FILE = PROJECT_DIR / "path.txt"
EXPLORED_FILE = PROJECT_DIR / "explored.txt"


ROWS = 15
COLS = 15


# 0 = 空地
# 1 = 障碍物
grid = np.zeros((ROWS, COLS), dtype=int)


# 起点、终点
start = None
goal = None


# 模式
mode = "start"


# 当前 A* 结果
explored = []
path = []


# ============================================================
# 创建窗口
# ============================================================

fig, ax = plt.subplots(figsize=(8, 8))

plt.subplots_adjust(
    bottom=0.20
)


# ============================================================
# 颜色
# ============================================================

# 0 空地
# 1 障碍物
# 2 搜索过的节点
# 3 最终路径
# 4 起点
# 5 终点

cmap = ListedColormap([
    "white",
    "black",
    "lightgray",
    "dodgerblue",
    "limegreen",
    "red"
])


# ============================================================
# 绘图函数
# ============================================================

def draw():
    """刷新整个地图"""

    display = np.zeros(
        (ROWS, COLS),
        dtype=int
    )


    # 障碍物
    display[grid == 1] = 1


    # A* 搜索过的节点
    for row, col in explored:

        if grid[row, col] == 0:
            display[row, col] = 2


    # 最终路径
    for row, col in path:
        display[row, col] = 3


    # 起点
    if start is not None:
        display[start[0], start[1]] = 4


    # 终点
    if goal is not None:
        display[goal[0], goal[1]] = 5


    ax.clear()


    ax.imshow(
        display,
        cmap=cmap,
        vmin=0,
        vmax=5
    )


    # 网格
    ax.set_xticks(
        np.arange(-0.5, COLS, 1),
        minor=True
    )

    ax.set_yticks(
        np.arange(-0.5, ROWS, 1),
        minor=True
    )

    ax.grid(
        which="minor",
        linewidth=1
    )


    # 坐标轴
    ax.set_xticks(range(COLS))
    ax.set_yticks(range(ROWS))

    ax.set_xlabel("Column")
    ax.set_ylabel("Row")


    ax.set_title(
        "A* Path Finding"
    )


    # --------------------------------------------------------
    # 画最终路径折线
    # --------------------------------------------------------

    if path:

        path_x = [
            col
            for row, col in path
        ]

        path_y = [
            row
            for row, col in path
        ]

        ax.plot(
            path_x,
            path_y,
            linewidth=3,
            marker="o",
            markersize=4
        )


    fig.canvas.draw_idle()


# ============================================================
# 保存地图
# ============================================================

def save_map():
    """把当前地图写入 map.txt"""

    with open(MAP_FILE, "w") as file:

        file.write(
            f"{ROWS} {COLS}\n"
        )

        for row in range(ROWS):

            line = " ".join(
                str(grid[row, col])
                for col in range(COLS)
            )

            file.write(
                line + "\n"
            )


# ============================================================
# 读取 A* 结果
# ============================================================

def read_points(filename):

    points = []

    if not filename.exists():
        return points


    with open(filename, "r") as file:

        for line in file:

            line = line.strip()

            if not line:
                continue

            row, col = map(
                int,
                line.split()
            )

            points.append(
                (row, col)
            )


    return points


# ============================================================
# 鼠标点击
# ============================================================

def on_click(event):

    global start
    global goal
    global path
    global explored


    # 点击坐标轴以外
    if event.inaxes != ax:
        return


    if event.xdata is None or event.ydata is None:
        return


    col = int(event.xdata)
    row = int(event.ydata)


    # 越界
    if row < 0 or row >= ROWS:
        return

    if col < 0 or col >= COLS:
        return


    # --------------------------------------------------------
    # 设置起点
    # --------------------------------------------------------

    if mode == "start":

        start = (row, col)

        grid[row, col] = 0

        path = []
        explored = []

        print(
            f"Start = ({row}, {col})"
        )


    # --------------------------------------------------------
    # 设置终点
    # --------------------------------------------------------

    elif mode == "goal":

        goal = (row, col)

        grid[row, col] = 0

        path = []
        explored = []

        print(
            f"Goal = ({row}, {col})"
        )


    # --------------------------------------------------------
    # 设置 / 删除障碍物
    # --------------------------------------------------------

    elif mode == "wall":

        # 起点终点不能变成障碍物
        if start == (row, col):
            return

        if goal == (row, col):
            return


        # 切换
        if grid[row, col] == 0:

            grid[row, col] = 1

        else:

            grid[row, col] = 0


        path = []
        explored = []


    draw()


# ============================================================
# 设置模式
# ============================================================

def set_start(event):

    global mode

    mode = "start"

    status_text.set_text(
        "Current mode: Set Start"
    )

    fig.canvas.draw_idle()


def set_goal(event):

    global mode

    mode = "goal"

    status_text.set_text(
        "Current mode: Set Goal"
    )

    fig.canvas.draw_idle()


def set_wall(event):

    global mode

    mode = "wall"

    status_text.set_text(
        "Current mode: Add / Remove Wall"
    )

    fig.canvas.draw_idle()


# ============================================================
# 运行 A*
# ============================================================

def run_astar(event):

    global path
    global explored


    if start is None:

        status_text.set_text(
            "Please set a start point first."
        )

        fig.canvas.draw_idle()

        return


    if goal is None:

        status_text.set_text(
            "Please set a goal point first."
        )

        fig.canvas.draw_idle()

        return


    # 保存地图
    save_map()


    # 删除旧结果
    if PATH_FILE.exists():
        PATH_FILE.unlink()

    if EXPLORED_FILE.exists():
        EXPLORED_FILE.unlink()


    # 调用 C++ A*
    result = subprocess.run(
        [
            str(ASTAR_PROGRAM),
            str(start[0]),
            str(start[1]),
            str(goal[0]),
            str(goal[1])
        ],
        cwd=PROJECT_DIR,
        capture_output=True,
        text=True
    )


    print(result.stdout)

    if result.stderr:
        print(result.stderr)


    # 读取结果
    explored = read_points(
        EXPLORED_FILE
    )

    path = read_points(
        PATH_FILE
    )


    if path:

        status_text.set_text(
            f"A* finished!  Path length: {len(path) - 1}"
        )

    else:

        status_text.set_text(
            "No path found."
        )


    draw()


# ============================================================
# 清空
# ============================================================

def clear_map(event):

    global start
    global goal
    global path
    global explored


    grid[:, :] = 0

    start = None
    goal = None

    path = []
    explored = []


    status_text.set_text(
        "Map cleared."
    )


    draw()


# ============================================================
# 创建按钮
# ============================================================

button_start_ax = plt.axes(
    [0.05, 0.07, 0.15, 0.06]
)

button_goal_ax = plt.axes(
    [0.22, 0.07, 0.15, 0.06]
)

button_wall_ax = plt.axes(
    [0.39, 0.07, 0.18, 0.06]
)

button_run_ax = plt.axes(
    [0.59, 0.07, 0.15, 0.06]
)

button_clear_ax = plt.axes(
    [0.76, 0.07, 0.15, 0.06]
)


button_start = Button(
    button_start_ax,
    "Set Start"
)

button_goal = Button(
    button_goal_ax,
    "Set Goal"
)

button_wall = Button(
    button_wall_ax,
    "Add / Remove Wall"
)

button_run = Button(
    button_run_ax,
    "Run A*"
)

button_clear = Button(
    button_clear_ax,
    "Clear"
)


button_start.on_clicked(
    set_start
)

button_goal.on_clicked(
    set_goal
)

button_wall.on_clicked(
    set_wall
)

button_run.on_clicked(
    run_astar
)

button_clear.on_clicked(
    clear_map
)


# ============================================================
# 状态文字
# ============================================================

status_text = fig.text(
    0.5,
    0.01,
    "Current mode: Set Start",
    ha="center"
)


# 鼠标事件
fig.canvas.mpl_connect(
    "button_press_event",
    on_click
)


# 初始绘图
draw()


# 显示窗口
plt.show()