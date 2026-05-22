/**
 * @file    User_Task.c
 * @brief   程控任务与路径规划主文件
 * @details 本文件为 "260506 无人机目标检测" 任务的核心逻辑，包含：
 *          1) 水平位置 PID 控制器（当前未使用，保留备用）
 *          2) 网格地图路径规划：BFS 最短路径 + 蛇形覆盖遍历(替代TSP)
 *          3) 一键程控任务状态机：由遥控器 CH6 高位触发，自动完成
 *             起飞 -> 路径规划 -> 逐格移动（含停留检测）-> 降落的完整流程。
 *
 * @author  Eric2195
 * @version 当前版本：Horizontal_Move 固定时间版（2026-05-21）
 *
 * @硬件平台  匿名科创凌霄飞控 ANO_LX_FC (STM32F407)
 * @遥控通道  CH6 高位(>1800): 启动任务  |  中位: 取消/复位  |  低位(<1200): 一键降落
 */

#include "User_Task.h"      /* 本文件头文件，包含 ROWS、COLS、Point 等宏和类型定义 */
#include "Drv_RcIn.h"       /* 遥控器输入驱动，提供 rc_in.fail_safe 和 rc_in.rc_ch 等 */
#include "LX_FC_Fun.h"      /* 飞控功能函数：OneKey_Takeoff、Horizontal_Move、LX_Change_Mode 等 */
#include "Ano_Math.h"       /* 匿名科创数学库，提供 LIMIT 等常用宏 */
#include "ANO_LX.h"         /* 飞控核心头文件，提供 rt_tar（程控目标值结构体）等 */
#include "LX_FC_State.h"    /* 飞控状态头文件 */

// /*============================ PID 参数与函数 ============================*/
// /**
//  * @note PID 部分说明
//  * 以下 y_move_pid / x_move_pid 为水平位置闭环控制器的雏形，
//  * 当前任务流程（UserTask_OneKeyCmd case 7）采用 Horizontal_Move 协议指令
//  * 做固定时间逐格移动，未调用本 PID。
//  * 保留原因：若后续需要基于雷达/光流坐标做实时闭环修正，可直接启用。
//  *
//  * 坐标映射约定（与雷达 SLAM 一致）：
//  *   - Y 方向：机头正前方为正，对应 vel_x（rt_tar.st_data.vel_x）
//  *   - X 方向：飞机左侧为正，对应 vel_y（rt_tar.st_data.vel_y）
//  */

// /* 水平位置 Y 方向（机头前后）PD 参数 */
// #define KP1 0.40f
// #define KD1 0.05f

// /* 水平位置 X 方向（飞机左右）PD 参数 */
// #define KP2 0.35f
// #define KD2 0.08f

// /**
//  * @brief  Y 方向 PD 控制器
//  * @param  cy  Y 方向误差（目标值 - 当前值），单位：cm
//  * @return 输出速度目标值，范围限制在 [-10, +10]
//  * @note   输出正值表示向前飞，负值表示向后飞
//  */
// float y_move_pid(s16 cy)
// {
//     static float err_old;
//     float err_d;
//     float pid_out_y;
//     err_d = cy - err_old;
//     err_old = cy;
//     pid_out_y = KP1 * cy + KD1 * err_d;
//     pid_out_y = -LIMIT(pid_out_y, -10, 10);
//     return pid_out_y;
// }

// /**
//  * @brief  X 方向 PD 控制器
//  * @param  cx  X 方向误差（目标值 - 当前值），单位：cm
//  * @return 输出速度目标值，范围限制在 [-10, +10]
//  * @note   输出正值表示向左飞，负值表示向右飞
//  */
// float x_move_pid(s16 cx)
// {
//     static float err_old;
//     float err_d;
//     float pid_out_x;
//     err_d = cx - err_old;
//     err_old = cx;
//     pid_out_x = KP2 * cx + KD2 * err_d;
//     pid_out_x = -LIMIT(pid_out_x, -10, 10);
//     return pid_out_x;
// }

/*============================ 路径规划部分 ============================*/
/**
 * @note 路径规划部分说明
 * 场地规格：7 格(纵向) x 9 格(横向)，单格 50cm x 50cm。
 * @attention 网格坐标系以右下角 A9/B1 为原点 (0,0)，与常规左上角原点不同：
 *   - row（纵向索引）：增大 = 机头正前方 / 向上（B1→B7）
 *   - col（横向索引）：增大 = 向左（A9→A1）
 * 起始点固定为 (0, 0)，即右下角 A9/B1。
 *
 * 障碍物（禁飞区）：在 barriers[] 数组中硬编码，默认 A7/A8/A9, B3。
 *
 * 算法流程：
 *   1. generate_barriers()    : 初始化网格，标记障碍物为不可通行(0)
 *   2. collect_accessible_cells(): 收集所有可通行格子的坐标
 *   3. snake_tsp()             : 用蛇形扫描策略生成覆盖顺序(替代TSP)
 *   4. build_full_path()      : 对每一对相邻访问点，用 BFS 找出最短路径并拼接
 *   5. real_routine()         : 将网格坐标转换为实际厘米坐标
 *
 * 重要：build_full_path 中循环上限为 count-1，即遍历完最后一个格子后直接结束，
 *       不返回起点。
 */

#ifndef INT_MAX
#define INT_MAX 32767
#endif

/**
 * @brief  队列结构体，用于 BFS（广度优先搜索）算法
 * @note   BFS 需要把待探索的节点按顺序存起来，先遇到的先处理（先进先出），
 *         队列是实现这种"先进先出"逻辑最简单的数据结构。
 *         data[] 是存储空间，front 指向队头（下一个要出队的位置），
 *         rear 指向队尾（下一个要入队的空位）。
 */
typedef struct {
    Point data[MAX_CELLS];  /* 队列内部数组，最多能存 MAX_CELLS(63) 个 Point */
    int front, rear;        /* front: 队头索引；rear: 队尾索引 */
} Queue;

/**
 * @brief  全局网格地图，记录每个格子是否可通行
 * @note   这是一个二维数组，大小为 grid[7][9]。
 *         每个元素的值含义：
 *           1 = 可以飞（空地）
 *           0 = 障碍物/禁飞区（不可通行）
 *         generate_barriers() 会把障碍物位置设为 0，其余默认为 1。
 */
int grid[ROWS][COLS];

/**
 * @brief  障碍物列表，直接在代码里写死
 * @note   数组里每个元素填的是 {A, B} 坐标，不是 grid 的 row/col 索引！
 *         例如 {6, 4} 表示 A6/B4（第 A6 列、第 B4 行交叉的那个格子）。
 *         generate_barriers() 会自动把 {A,B} 换算成 grid 数组能理解的 {row,col}。
 *         当前障碍物在 A6 这一列、B4~B6 这一排，形成一堵竖墙。
 */
Point barriers[BARRIER_COUNT] = {
    {7, 3},
    {8, 3},
    {9, 3}
};

/**
 * @brief  可达格子列表
 * @note   路径规划时，先把所有值为 1 的空地格子坐标收集到这里，
 *         方便后续 TSP 算法逐个决定访问顺序。
 *         accessible_count 记录当前实际存了多少个格子。
 */
static Point accessible_cells[MAX_CELLS];
int accessible_count = 0;

/**
 * @brief  最终拼接完成的逐格路径
 * @note   这是 TSP + BFS 计算完成后，飞机实际需要经过的每一格的坐标序列。
 *         例如：[(0,0), (1,0), (1,1), (0,1), (0,2), ...]
 *         数组索引 i 和 i+1 之间一定只相差一格（上下左右相邻）。
 *         final_path_length 记录路径总长度。
 */
static Point final_path[MAX_PATH_LENGTH];
int final_path_length = 0;

/**
 * @brief  路径规划全局缓冲区（static避免栈溢出）
 * @note   STM32F407任务栈通常只有1-2KB，以下数组若放局部变量会导致栈溢出复位。
 *         改用全局静态存储区，由 snake_tsp / snake_tsp_col / build_full_path 复用。
 */
static Point g_order_row[MAX_CELLS * 2];    /* 行扫描访问顺序 */
static Point g_order_col[MAX_CELLS * 2];    /* 列扫描访问顺序 */
static Point g_seg_buffer[MAX_PATH_LENGTH]; /* BFS路径拼接缓冲区（build_full_path + run_path_planner复用） */

/**
 * @brief  四邻域搜索方向表
 * @note   BFS 找最短路径时，只能往上下左右四个方向走（不能斜着飞）。
 *         每一对数字 {dr, dc} 表示行变化和列变化：
 *           {0,  1} : row 不变，col +1 = 向左（因为 col 增加是向左）
 *           {1,  0} : row +1，col 不变 = 向上/前进
 *           {0, -1} : row 不变，col -1 = 向右
 *           {-1, 0} : row -1，col 不变 = 向下/后退
 *         顺序无所谓，只要四个方向都覆盖到就行。
 */
static int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

/**
 * @brief  厘米级路径点数组
 * @note   real_routine() 会把 final_path 里的网格坐标乘以 50cm 转换成实际物理坐标，
 *         存到这里。当前版本没有使用这个数组做闭环控制，仅用于调试输出。
 *         path_len_routine 是数组有效长度。
 */
PathPoint path_points[MAX_PATH_POINTS];
uint8_t path_len_routine = 0;

/**
 * @brief  当前飞机水平位置（由树莓派 SLAM 通过 USART3 实时更新）
 * @note   单位：厘米。now_x 对应左右方向（飞机左侧为正），
 *         now_y 对应前后方向（机头前方为正）。
 *         当前版本采用 Horizontal_Move 固定时间移动，未用这两个变量做闭环，
 *         只在 Ano_Scheduler.c 里读取并通过串口打印调试用。
 */
s16 now_x = 0;
s16 now_y = 0;

/**
 * @brief  初始化网格地图，把所有格子设为可通行
 * @note   用双重循环遍历 grid[7][9] 的每一个元素，全部赋值为 1。
 *         这只是第一步，接下来 generate_barriers() 会把障碍物位置改回 0。
 *         i 控制行号（0~6，对应 B1~B7），j 控制列号（0~8，对应 A9~A1）。
 */
void init_grid() {
    int i, j;
    for (i = 0; i < ROWS; i++)           /* 外层循环：逐行扫描，从上到下（B1 到 B7） */
        for (j = 0; j < COLS; j++)       /* 内层循环：逐列扫描，从左到右（A9 到 A1） */
            grid[i][j] = 1;              /* 先假设所有格子都是空地（1） */
}

/**
 * @brief  初始化队列，清空队头和队尾指针
 * @param  q  指向 Queue 结构体的指针
 * @note   把 front 和 rear 都设为 0，表示队列是空的。
 *         这种用数组实现的队列叫做"顺序队列"，rear 永远指向下一个可写入的位置。
 */
void init_queue(Queue *q) {
    q->front = q->rear = 0;
}

/**
 * @brief  判断队列是否为空
 * @param  q  指向 Queue 结构体的指针
 * @return 1 表示空队列，0 表示队列里有数据
 * @note   当 front == rear 时，说明所有入队的数据都已经出队了，队列为空。
 *         这个实现没有处理"循环队列"和满队列判断，因为 BFS 里不会存超过 MAX_CELLS 个节点。
 */
int is_queue_empty(Queue *q) {
    return q->front == q->rear;
}

/**
 * @brief  入队操作：把一个新节点放到队尾
 * @param  q  指向 Queue 结构体的指针
 * @param  p  要入队的节点坐标（Point 类型）
 * @note   把 p 复制到 data[rear] 这个位置，然后 rear 自增 1，
 *         指向下一个空位。如果 rear 超过 MAX_CELLS 就会越界，
 *         但本程序地图最大只有 63 格，不会超过。
 */
void enqueue(Queue *q, Point p) {
    q->data[q->rear++] = p;
}

/**
 * @brief  出队操作：从队头取出一个节点
 * @param  q  指向 Queue 结构体的指针
 * @return 队头节点的坐标（Point 类型）
 * @note   返回 data[front] 这个位置的值，然后 front 自增 1。
 *         调用前应该先判断队列是否为空，否则可能取到脏数据。
 */
Point dequeue(Queue *q) {
    return q->data[q->front++];
}

/**
 * @brief  检查给定的行列号是否在地图范围内
 * @param  row  行号（纵向索引）
 * @param  col  列号（横向索引）
 * @return 1 表示在地图内，0 表示越界（如负数或超过 ROWS/COLS）
 * @note   这是所有路径搜索的边界保护。如果不管边界直接访问 grid[-1][5]，
 *         会导致内存越界，程序跑飞。四个条件必须同时满足才返回 1。
 */
int is_valid(int row, int col) {
    return row >= 0 && row < ROWS && col >= 0 && col < COLS;
}



/**
 * @brief  根据硬编码障碍物初始化网格地图
 * @note   调用顺序：先 init_grid() 全置 1，再把障碍物位置置 0。
 *         barriers 数组中填的是 {A, B} 坐标，这里自动换算为 grid 索引：
 *           grid_row = B - 1        （B1 对应 row 0，B7 对应 row 6）
 *           grid_col = COLS - A     （A9 对应 col 0，A1 对应 col 8）
 *         这样用户改障碍物时只需要看图上的 A/B 标注即可。
 */
void generate_barriers() {
    int i;
    init_grid();                     /* 第一步：先清空地图，全部设为可通行（1） */
    for (i = 0; i < BARRIER_COUNT; i++) {
        int a = barriers[i].row;     /* 取出 A 坐标（横向 1~9）。注意：这里借用了 Point.row 字段存 A */
        int b = barriers[i].col;     /* 取出 B 坐标（纵向 1~7）。这里借用了 Point.col 字段存 B */
        int grid_row = b - 1;        /* B 坐标转 row 索引：B1 -> 0, B2 -> 1, ... */
        int grid_col = COLS - a;     /* A 坐标转 col 索引：A9 -> 9-9=0, A8 -> 9-8=1, ... */
        if (is_valid(grid_row, grid_col))  /* 安全检查：防止填错坐标导致数组越界 */
            grid[grid_row][grid_col] = 0;  /* 把该格子设为障碍物（0），不可通行 */
    }
}

/**
 * @brief  收集所有可通行格子的坐标
 * @note   扫描整个 grid[7][9]，把值为 1 的空地坐标依次存入 accessible_cells[]。
 *         accessible_count 在函数开头清零，最后等于空地总数（正常情况下是 63-3=60）。
 *         这个数组是 TSP 算法的输入，告诉算法"有哪些格子需要被访问"。
 */
void collect_accessible_cells() {
    int i, j;
    accessible_count = 0;            /* 清零计数器，重新开始收集 */
    for (i = 0; i < ROWS; i++)       /* 逐行扫描 */
        for (j = 0; j < COLS; j++)   /* 逐列扫描 */
            if (grid[i][j] == 1)     /* 如果该格子是空地（不是障碍物） */
                accessible_cells[accessible_count++] = (Point){i, j};  /* 存入列表，计数器加 1 */
}

/**
 * @brief  通用蛇形 TSP（替代最近邻 TSP）
 * @param  order  输出数组，存放计算出的访问顺序
 * @return order 数组的有效长度
 * @note   分析每行可达段，按蛇形方向交替输出格子坐标。
 *         偶数行: A9->A1 (col递增), 奇数行: A1->A9 (col递减)。
 *         然后由 build_full_path() 用 BFS 自动拼接行间路径（含绕行）。
 *         适应任意连续三格禁飞区配置。
 */
int snake_tsp(Point order[]) {
    int idx = 0;
    int r, c, s;

    for (r = 0; r < ROWS; r++) {
        /*--- 收集本行的可达段起止列 ---*/
        /* 方案：遍历本行所有列，收集连续可达区域 */
        int seg_starts[3];   /* 最多3个段(2个障碍物可分割出3段) */
        int seg_ends[3];
        int seg_count = 0;
        int cur_start = -1;

        for (c = 0; c < COLS; c++) {
            if (grid[r][c] == 1) {
                if (cur_start == -1) {
                    cur_start = c;  /* 新段开始 */
                }
            } else {
                if (cur_start != -1) {
                    seg_starts[seg_count] = cur_start;
                    seg_ends[seg_count] = c - 1;
                    seg_count++;
                    cur_start = -1;  /* 段结束 */
                }
            }
        }
        /* 处理行尾未闭合的段 */
        if (cur_start != -1) {
            seg_starts[seg_count] = cur_start;
            seg_ends[seg_count] = COLS - 1;
            seg_count++;
        }

        if (seg_count == 0) {
            continue;  /* 本行全被阻挡，跳过 */
        }

        /*--- 按蛇形方向输出本段格子 ---*/
        if (r % 2 == 0) {
            /* 偶数行: A9->A1 方向 (col递增) */
            for (s = 0; s < seg_count; s++) {
                for (c = seg_starts[s]; c <= seg_ends[s]; c++) {
                    order[idx++] = (Point){r, c};
                }
            }
        } else {
            /* 奇数行: A1->A9 方向 (col递减) */
            for (s = seg_count - 1; s >= 0; s--) {
                for (c = seg_ends[s]; c >= seg_starts[s]; c--) {
                    order[idx++] = (Point){r, c};
                }
            }
        }
    }

    return idx;
}

/**
 * @brief  列扫描蛇形 TSP（snake_tsp 的列版本）
 * @param  order  输出数组，存放列扫描访问顺序
 * @return order 数组有效长度
 * @note   按列扫描，偶数列从上到下(row递增)，奇数列从下到上(row递减)。
 *         禁飞区为垂直排列时，列扫描路径通常更短。
 */
int snake_tsp_col(Point order[]) {
    int idx = 0;
    int r, c, s;

    for (c = 0; c < COLS; c++) {
        /*--- 收集本列可达段起止行 ---*/
        int seg_starts[3];
        int seg_ends[3];
        int seg_count = 0;
        int cur_start = -1;

        for (r = 0; r < ROWS; r++) {
            if (grid[r][c] == 1) {
                if (cur_start == -1) {
                    cur_start = r;
                }
            } else {
                if (cur_start != -1) {
                    seg_starts[seg_count] = cur_start;
                    seg_ends[seg_count] = r - 1;
                    seg_count++;
                    cur_start = -1;
                }
            }
        }
        if (cur_start != -1) {
            seg_starts[seg_count] = cur_start;
            seg_ends[seg_count] = ROWS - 1;
            seg_count++;
        }

        if (seg_count == 0) {
            continue;
        }

        /*--- 偶数列: 从上到下(row递增), 奇数列: 从下到上(row递减) ---*/
        if (c % 2 == 0) {
            for (s = 0; s < seg_count; s++) {
                for (r = seg_starts[s]; r <= seg_ends[s]; r++) {
                    order[idx++] = (Point){r, c};
                }
            }
        } else {
            for (s = seg_count - 1; s >= 0; s--) {
                for (r = seg_ends[s]; r >= seg_starts[s]; r--) {
                    order[idx++] = (Point){r, c};
                }
            }
        }
    }

    return idx;
}

/**
 * @brief  BFS 寻找两格子间的最短路径
 * @param  start   起点坐标
 * @param  end     终点坐标
 * @param  path    输出数组，用于存放找到的最短路径（按顺序）
 * @param  max_len path 数组的最大容量，防止越界
 * @return 路径长度（包含起点和终点），如果不可达返回 0
 * @note   BFS（广度优先搜索）特性：第一次到达终点时，走的一定是步数最少的路径。
 *         因为 BFS 是一层一层向外扩展的，像水波一样，先被访问到的路径一定更短。
 *         parent[][][] 记录每个格子的"爸爸"是谁，方便最后倒推路径。
 */
int find_shortest_path(Point start, Point end, Point path[], int max_len) {
    static int visited[ROWS][COLS];  /* static避免栈溢出，全局存储 */
    static int parent[ROWS][COLS][2];
    static Queue q;
    int r, c;
    for (r = 0; r < ROWS; r++)      /* 手动清零 */
        for (c = 0; c < COLS; c++)
            visited[r][c] = 0;
    init_queue(&q);                  /* 初始化 */
    enqueue(&q, start);              /* 起点入队 */
    visited[start.row][start.col] = 1;  /* 标记起点已访问 */
    parent[start.row][start.col][0] = -1;  /* 起点没有前驱，用 -1 表示"到头了" */
    parent[start.row][start.col][1] = -1;

    /* BFS 主循环 */
    while (!is_queue_empty(&q)) {
        Point cur = dequeue(&q);     /* 取出当前要处理的格子 */
        /* 如果当前格子就是终点，说明找到了最短路径，开始倒推 */
        if (cur.row == end.row && cur.col == end.col) {
            int len = 0;             /* len 记录路径长度 */
            Point t = end;           /* 从终点开始倒推 */
            /* 不断查 parent，直到回到起点（parent 为 -1） */
            while (t.row != -1 && len < max_len) {
                path[len++] = t;     /* 把当前格子存入 path */
                int pr = parent[t.row][t.col][0];  /* 查"爸爸"的行号 */
                int pc = parent[t.row][t.col][1];  /* 查"爸爸"的列号 */
                t.row = pr; t.col = pc;            /* 跳到爸爸的位置 */
            }
            if (len >= max_len) return 0;  /* 路径超长，防御性返回失败 */
            {
                int i;
                /* 倒推得到的路径是"终点->起点"，需要翻转成"起点->终点" */
                for (i = 0; i < len / 2; i++) {
                    Point tmp = path[i];                    /* 暂存头部元素 */
                    path[i] = path[len - 1 - i];            /* 尾部元素搬到头部 */
                    path[len - 1 - i] = tmp;                /* 头部元素搬到尾部 */
                }
            }
            return len;              /* 返回路径长度 */
        }
        {
            int i;
            /* 如果不是终点，继续往四个方向探索 */
            for (i = 0; i < 4; i++) {
                int nr = cur.row + directions[i][0];  /* 新行号 */
                int nc = cur.col + directions[i][1];  /* 新列号 */
                /* 只有满足三个条件才入队：不越界、未访问、不是障碍物 */
                if (is_valid(nr, nc) && !visited[nr][nc] && grid[nr][nc]) {
                    visited[nr][nc] = 1;              /* 标记已访问 */
                    parent[nr][nc][0] = cur.row;      /* 记录"爸爸"是谁，方便后续倒推 */
                    parent[nr][nc][1] = cur.col;
                    enqueue(&q, (Point){nr, nc});     /* 入队，等待后续处理 */
                }
            }
        }
    }

    return 0;  /* 如果队列都空了还没找到终点，说明两点之间被障碍物彻底隔断，返回 0 */
}

/**
 * @brief  将 TSP 访问顺序拼接为完整逐格路径
 * @param  order  TSP 计算出的访问顺序数组
 * @param  count  order 数组的长度（等于可达格子总数）
 * @note   TSP 只给出了"先去 A，再去 B，再去 C..."的顺序，
 *         但 A 到 B 之间可能隔了好几格（比如中间要绕开障碍物）。
 *         所以需要对每一对相邻访问点调用 find_shortest_path(BFS)，
 *         把中间经过的每一格都补齐，拼成一条完整的"一步一格"路径。
 *
 *         去重处理：
 *           第 1 段路径（起点->第2个点）完整保留，包含起点；
 *           从第 2 段开始，跳过每段的起点（因为和前一段的终点重复），只保留后面的点。
 *
 *         循环上限为 count-1：只拼接到倒数第二个点->最后一个点，
 *         遍历完最后一个格子直接结束，不返回起点。
 */
void build_full_path(Point order[], int count) {
    int i;
    final_path_length = 0;           /* 清空最终路径 */
    for (i = 0; i < count - 1; i++) {  /* 两两一对，共 count-1 段 */
        Point start = order[i];      /* 当前段的起点 */
        Point end = order[i + 1];    /* 当前段的终点 */
        int seg_len = find_shortest_path(start, end, g_seg_buffer, MAX_PATH_LENGTH);  /* 调用 BFS */
        int j;
        if (seg_len == 0) {
            /* BFS找不到路径：两点被障碍物彻底隔断 */
            final_path_length = 0;   /* 标记路径无效 */
            return;                  /* 立即终止，不再继续拼接 */
        }
        /* 把当前段拼接到 final_path 后面。
           i==0 时保留全部（包含起点）；i>0 时跳过 seg[0]（重复节点） */
        for (j = (i == 0 ? 0 : 1); j < seg_len; j++) {
            final_path[final_path_length++] = g_seg_buffer[j];
        }
    }
}

/**
 * @brief  将网格坐标转换为实际厘米坐标
 * @note   每个格子 50cm，所以：
 *           x（左右物理坐标）= col * 50
 *           y（前后物理坐标）= row * 50
 *         注意：这里 x 和 y 只是记录用，当前版本未用于闭环控制。
 *         path_len_routine 记录转换后的路径点数量。
 */
void real_routine()
{
    int i;
    path_len_routine = 0;            /* 清零计数器 */
    for (i = 0; i < final_path_length; i++) {
        path_points[path_len_routine].x = GRID_SIZE_CM * final_path[i].row;  /* row -> 前后厘米坐标 */
        path_points[path_len_routine].y = GRID_SIZE_CM * final_path[i].col;  /* col -> 左右厘米坐标 */
        path_len_routine++;          /* 计数器加 1 */
    }
}

/**
 * @brief  路径规划入口函数，一键调用全部规划流程
 * @note   调用顺序（不能乱）：
 *   1. generate_barriers()       : 根据 A/B 障碍物坐标初始化网格
 *   2. collect_accessible_cells(): 收集所有空地
 *   3. snake_tsp()     : 用蛇形扫描策略生成覆盖顺序
 *   4. build_full_path()         : 用 BFS 把相邻点之间的最短路径拼起来
 *   5. real_routine()            : 转成厘米坐标（调试用）
 *
 *   执行完后，final_path[] 里就是飞机要逐格经过的完整路径。
 */
void run_path_planner(void) {
    int count_row, count_col;
    int len_row, len_col;

    generate_barriers();             /* 步骤 1：建图（标记障碍物） */
    collect_accessible_cells();      /* 步骤 2：收集空地 */

    /*--- 步骤 3：分别生成行扫描和列扫描的访问顺序（使用全局缓冲区） ---*/
    count_row = snake_tsp(g_order_row);      /* 行扫描 */
    count_col = snake_tsp_col(g_order_col);  /* 列扫描 */

    /*--- 步骤 4：分别拼接，比较长度，选更优的 ---*/
    build_full_path(g_order_row, count_row);
    len_row = final_path_length;

    build_full_path(g_order_col, count_col);
    len_col = final_path_length;     /* 注意：此时 final_path 已被列扫描结果覆盖 */

    if (len_col < len_row) {
        /* 列扫描更优，final_path 已经是列结果，无需额外操作 */
    } else {
        /* 行扫描更优或相等，重新计算一次行扫描结果覆盖 final_path */
        build_full_path(g_order_row, count_row);
    }

    real_routine();                  /* 步骤 5：厘米坐标转换 */
}

/*============================ 一键程控任务状态机 ============================*/
/**
 * @brief  一键程控任务主状态机
 * @note   【调用周期】：20ms（由 Ano_Scheduler.c 的 Loop_50Hz 调用）
 *
 *         【触发方式】（看遥控器 CH6 通道值）：
 *           - CH6 低位 (800~1200) : 一键降落（独立逻辑，随时可用）
 *           - CH6 中位 (1200~1800): 取消任务，清零速度，复位所有状态
 *           - CH6 高位 (1800~2200): 启动任务流程
 *
 *         【任务主流程】（mission_step 状态机）：
 *           case 0 : 空闲/复位状态，什么都不做
 *           case 1 : 切换为程控模式 (LX_Change_Mode(3))
 *           case 2 : 解锁电机 (FC_Unlock())
 *           case 3 : 延时 2s，等待解锁稳定
 *           case 4 : 一键起飞到 50cm (OneKey_Takeoff(50))
 *           case 5 : 悬停稳定 3s
 *           case 6 : 执行路径规划 (run_path_planner())
 *           case 7 : 航点跟踪：逐格移动 + 停留检测（内含子状态机 move_sub_step）
 *           case 8 : 降落 (OneKey_Land())
 *
 *         【case 7 子状态机】（move_sub_step）：
 *           sub_step 0 : 计算下一格方向，发送 Horizontal_Move 指令
 *           sub_step 1 : 等待移动完成（固定 6s）
 *           sub_step 2 : 停留检测 1s（目标检测窗口），完成后 wp_idx++
 *
 *         【安全保护】：
 *           - CH6 中位：立即清零 vel_x/vel_y/vel_z，重置所有状态
 *           - 失控保护(fail_safe)：遥控器信号丢失，同样清零并复位
 */
void UserTask_OneKeyCmd(void)
{
    /*--- 静态状态变量，跨调用周期保持状态 ---*/
    /*
     * 为什么用 static？
     * 这个函数每 20ms 被调用一次。如果用普通局部变量，每次调用都会被重新初始化，
     * 状态机就无法"记住"自己走到哪一步了。static 变量只会初始化一次，之后保持上次赋值。
     */
    static u8 one_key_land_f = 1;      /* 一键降落触发标志。
                                           0 = 还没开始降落，允许触发；
                                           1 = 正在降落或已降落，防止重复调用。
                                           初始设为 1，表示上电后默认不自动降落。 */
    static u8 one_key_mission_f = 0;   /* 任务启动标志。
                                           0 = 未启动任务；
                                           1 = 任务已启动，正在执行 mission_step 状态机。 */
    static u8 mission_step = 0;        /* 主状态机步骤号，对应上面列出的 case 0~8。
                                           每个 case 执行完后通过自增（++ 或 += 函数返回值）进入下一步。 */
    static u16 delay_cnt_ms = 0;       /* 通用延时计数器，单位毫秒。
                                           在 case 3（等待 2s）和 case 5（悬停 3s）中使用。
                                           每次调用增加 20（因为周期是 20ms）。 */
    static u16 hover_delay_ms = 0;     /* 悬停专用延时计数器（当前版本未使用，保留备用）。 */
    static u8 wp_idx = 0;              /* way-point index，当前走到 final_path 的第几个航点。
                                           从 0 开始，每走完一格加 1。 */
    static u8 move_sub_step = 0;       /* case 7 内部的子状态机步骤（0/1/2）。
                                           因为一格移动需要分"发指令->等移动->停留检测"三步，
                                           所以再用一个子状态机来管理。 */
    static u16 move_wait_ms = 0;       /* 子状态机延时计数器，单位毫秒。
                                           sub_step 1 里累计 6s，sub_step 2 里累计 1s。 */

    /*--- 第一层判断：遥控器是否失控 ---*/
    if (rc_in.fail_safe == 0)        /* fail_safe == 0 表示遥控器信号正常，可以执行程控逻辑 */
    {
        /*====== CH6 低位：一键降落（随时可用，优先级最高）======*/
        /* rc_in.rc_ch.st_data.ch_[ch_6_aux2] 是 CH6 通道的原始值（1000~2000）。
           低位范围 800~1200 对应遥控器拨杆的最下方档位。 */
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            /* one_key_land_f == 0 表示"还没执行过降落"，防止拨杆一直停在低位时重复调用 */
            if (one_key_land_f == 0)
            {
                /* OneKey_Land() 返回 1 表示降落指令已发出，把标志置 1 */
                one_key_land_f = OneKey_Land();
            }
        }
        else
        {
            /* CH6 离开低位，重置标志，下次拨回低位可以再次触发降落 */
            one_key_land_f = 0;
        }

        /*====== CH6 高位：启动任务流程 ======*/
        /* 高位范围 1800~2200 对应遥控器拨杆的最上方档位 */
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            /* one_key_mission_f == 0 表示"任务还没启动"，防止重复初始化 */
            if (one_key_mission_f == 0)
            {
                one_key_mission_f = 1;   /* 标记任务已启动 */
                mission_step = 1;        /* 从 case 1 开始（跳过 case 0 的空闲态） */
                delay_cnt_ms = 0;        /* 清空延时计数器 */
                hover_delay_ms = 0;      /* 清空悬停计数器 */
            }
        }
        else
        {
            /* CH6 不在高位（即中位或低位），标记任务未启动 */
            one_key_mission_f = 0;
        }

        /*====== 任务主状态机（只有 one_key_mission_f==1 时才执行）======*/
        if (one_key_mission_f == 1)
        {
            switch(mission_step)         /* 根据 mission_step 的值跳进对应的 case */
            {
                /*--- case 0: 空闲/复位状态 ---*/
                /* 正常流程不会进入这里，只有手动把 mission_step 设为 0 才会进来 */
                case 0:
                {
                    delay_cnt_ms = 0;    /* 清空通用延时 */
                    hover_delay_ms = 0;  /* 清空悬停延时 */
                }
                break;                   /* break 跳出 switch，等待下一次 20ms 周期 */

                /*--- case 1: 切换程控模式 ---*/
                /* LX_Change_Mode(3) 向飞控发送指令，切换到"程控模式"（模式 3）。
                   返回值：1 表示切换成功，可以进入下一步；0 表示还没切好，继续等待。 */
                case 1:
                {
                    mission_step += LX_Change_Mode(3);
                }
                break;

                /*--- case 2: 解锁电机 ---*/
                /* FC_Unlock() 向飞控发送解锁指令，让电机可以转动。
                   返回值：1 表示解锁成功；0 表示还在等待解锁。 */
                case 2:
                {
                    mission_step += FC_Unlock();
                }
                break;

                /*--- case 3: 延时 2s，等待解锁稳定 ---*/
                /* 电机解锁后不能立刻起飞，需要等几秒让飞控自检和姿态稳定。
                   delay_cnt_ms 每次加 20，因为本函数 20ms 调用一次。
                   当累计到 2000（即 2000ms = 2s）时，清零并进入下一步。 */
                case 3:
                {
                    delay_cnt_ms += 20;
                    if (delay_cnt_ms >= 2000)
                    {
                        delay_cnt_ms = 0;    /* 清零，为下次延时做准备 */
                        mission_step++;      /* mission_step 从 3 变成 4 */
                    }
                }
                break;

                /*--- case 4: 起飞到 50cm ---*/
                /* OneKey_Takeoff(50) 发送一键起飞指令，目标高度 50cm。
                   返回值：1 表示起飞指令已发出/已完成；0 表示还在执行中。 */
                case 4:
                {
                    mission_step += OneKey_Takeoff(50);
                }
                break;

                /*--- case 5: 悬停稳定 3s ---*/
                /* 起飞到达 50cm 后，悬停 3 秒等姿态和高度稳定，再开始水平移动。
                   计时原理和 case 3 一样，delay_cnt_ms 每次加 20，满 3000ms 进下一步。 */
                case 5:
                {
                    delay_cnt_ms += 20;
                    if (delay_cnt_ms >= 3000)
                    {
                        delay_cnt_ms = 0;
                        mission_step++;
                    }
                }
                break;

                /*--- case 6: 执行路径规划 ---*/
                /* 调用 run_path_planner() 运行 BFS+TSP，生成覆盖所有可达格子的路径。
                   如果 final_path_length > 0，说明规划成功，进入 case 7 开始逐格飞行。
                   如果等于 0，说明没有可达路径（比如障碍物把路全堵了），直接跳到 case 8 降落。 */
                case 6:
                {
                    run_path_planner();          /* 执行路径规划 */
                    if (final_path_length > 0)   /* 检查是否规划出有效路径 */
                    {
                        mission_step++;          /* 有路径，进入 case 7 航点跟踪 */
                    }
                    else
                    {
                        /* 无可达路径，直接跳到降落 */
                        mission_step = 8;
                    }
                }
                break;

                /*--- case 7: 航点跟踪（逐格移动 + 停留检测）---*/
                /* 这是整个任务最复杂的部分，负责让飞机沿着 final_path[] 一格一格地飞。
                   由于 Horizontal_Move 是"发一次指令，飞控自己执行"的模式，
                   我们需要等待飞行完成，再停留做目标检测，然后才发下一格的指令。
                   因此内部又用了一个子状态机 move_sub_step 来管理这三个阶段。 */
                case 7:
                {
                    /* wp_idx 是当前所在的航点索引。
                       final_path_length - 1 是因为每次比较的是"当前点"和"下一点"，
                       所以最后一个点不需要再比较。 */
                    if (wp_idx < final_path_length - 1)
                    {
                        /*-- sub_step 0: 计算方向并发送 Horizontal_Move 指令 --*/
                        if (move_sub_step == 0)
                        {
                            /* cur: 当前所在的格子坐标；next: 下一个要去的格子坐标 */
                            Point cur = final_path[wp_idx];
                            Point next = final_path[wp_idx + 1];
                            /* dr: row 方向的变化量（-1, 0, 1）
                               dc: col 方向的变化量（-1, 0, 1） */
                            int dr = next.row - cur.row;
                            int dc = next.col - cur.col;

                            /* 根据 dr 和 dc 判断要往哪个方向飞，换算成飞控能理解的角度。
                               Horizontal_Move 的 angle 参数：0=前方，90=右侧，180=后方，270=左侧。

                               坐标系提醒（右下角 A9/B1 为原点）：
                                 dr=+1 : row 增加 = 向上（B1->B7）= 机头前方 -> angle=0
                                 dr=-1 : row 减少 = 向下（B7->B1）= 机头后方 -> angle=180
                                 dc=+1 : col 增加 = 向左（A9->A1）= 飞机左侧 -> angle=270
                                 dc=-1 : col 减少 = 向右（A1->A9）= 飞机右侧 -> angle=90
                            */
                            u16 angle = 0;
                            if (dr == 1 && dc == 0)       angle = 0;
                            else if (dr == -1 && dc == 0) angle = 180;
                            else if (dr == 0 && dc == 1)  angle = 270;
                            else if (dr == 0 && dc == -1) angle = 90;

                            /* Horizontal_Move(距离, 速度, 角度)：
                               发送一条水平移动指令，让飞控向指定方向移动指定距离。
                               返回值：1 表示指令发送成功；0 表示还没准备好发送。 */
                            if (Horizontal_Move(GRID_SIZE_CM, 15, angle))
                            {
                                move_sub_step = 1;   /* 指令已发出，进入等待阶段 */
                                move_wait_ms = 0;    /* 清空等待计时器 */
                            }
                        }
                        /*-- sub_step 1: 等待移动完成（固定 6s） --*/
                        /* 为什么不等待飞控回传完成标志？因为当前版本没有位置闭环反馈，
                           只能根据经验时间估算。50cm / 15cm/s ≈ 3.3s，再加 2.7s 余量，共 6s。
                           move_wait_ms 每次加 20（ms），满 6000ms 认为移动完成。 */
                        else if (move_sub_step == 1)
                        {
                            move_wait_ms += 20;
                            if (move_wait_ms >= 6000)
                            {
                                move_wait_ms = 0;    /* 清零计时器 */
                                move_sub_step = 2;   /* 进入停留检测阶段 */
                            }
                        }
                        /*-- sub_step 2: 停留检测 1s（目标检测窗口） --*/
                        /* 飞机停稳后，留 1 秒时间做目标检测（比如树莓派拍照/识别）。
                           这 1s 内不发送任何移动指令，只是原地悬停。
                           时间到后 wp_idx++，准备前往下一格。 */
                        else if (move_sub_step == 2)
                        {
                            move_wait_ms += 20;
                            if (move_wait_ms >= 1000)
                            {
                                move_wait_ms = 0;    /* 清零计时器 */
                                move_sub_step = 0;   /* 回到 sub_step 0，准备发下一格指令 */
                                wp_idx++;            /* 航点索引加 1，下一个格子 */
                            }
                        }
                    }
                    else
                    {
                        /* wp_idx >= final_path_length - 1，说明所有航点都走完了。
                           清零索引和子状态机，进入 case 8 降落。 */
                        wp_idx = 0;
                        move_sub_step = 0;
                        move_wait_ms = 0;
                        mission_step++;
                    }
                }
                break;

                /*--- case 8: 降落 ---*/
                /* OneKey_Land() 发送一键降落指令，飞控自动缓降落地。
                   返回值：1 表示降落指令已发出/已完成；0 表示还在降落过程中。 */
                case 8:
                {
                    mission_step += OneKey_Land();
                }
                break;

                /*--- default: 异常保护 ---*/
                /* 如果 mission_step 被意外设成了 9、10 等非法值，不做任何事，
                   防止程序跑飞。实际正常流程不会进这里。 */
                default:
                    break;
            }
        }
        /*====== CH6 中位：取消任务，紧急复位 ======*/
        /* 当 CH6 不在高位时（one_key_mission_f == 0），进入这个 else 分支。
           这是紧急停止逻辑：立即把三轴速度目标清零，防止之前残留的 vel_x/vel_y
           指令继续生效导致飞机乱飞。同时重置所有状态和索引。 */
        else
        {
            /* 立即清零速度输出，防止残留指令干扰降落或悬停 */
            rt_tar.st_data.vel_x = 0;    /* 前后方向速度置零 */
            rt_tar.st_data.vel_y = 0;    /* 左右方向速度置零 */
            rt_tar.st_data.vel_z = 0;    /* 垂直方向速度置零 */

            /* 重置所有状态变量，下次拨到高位时从 case 1 重新开始 */
            mission_step = 0;
            delay_cnt_ms = 0;
            hover_delay_ms = 0;
            wp_idx = 0;
            move_sub_step = 0;
            move_wait_ms = 0;
        }
    }
    /*--- 第二层判断：失控保护触发 ---*/
    /* rc_in.fail_safe != 0 表示遥控器信号丢失（比如关机、超出遥控范围、干扰严重）。
       此时飞控已经触发了自身的失控保护（通常是自动降落），
       我们这里也要清零所有程控输出，并彻底重置任务状态，
       防止失控恢复后飞机突然执行之前残留的指令。 */
    else
    {
        rt_tar.st_data.vel_x = 0;    /* 清零前后速度 */
        rt_tar.st_data.vel_y = 0;    /* 清零左右速度 */
        rt_tar.st_data.vel_z = 0;    /* 清零垂直速度 */

        /* 重置所有状态和标志 */
        mission_step = 0;
        one_key_mission_f = 0;
        delay_cnt_ms = 0;
        hover_delay_ms = 0;
        wp_idx = 0;
        move_sub_step = 0;
        move_wait_ms = 0;
    }
}