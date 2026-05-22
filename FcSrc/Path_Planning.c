/**
 * @file    Path_Planning.c
 * @brief   网格地图路径规划：BFS 最短路径 + 蛇形覆盖遍历
 * @details 场地规格：7 格(纵向) x 9 格(横向)，单格 50cm x 50cm。
 *          包含：建图、可达格收集、蛇形 TSP、BFS 最短路径、路径拼接、厘米坐标转换。
 */

#include "User_Task.h"
#include "Path_Planning.h"

#ifndef INT_MAX
#define INT_MAX 32767
#endif

/**
 * @brief  BFS 队列结构体（顺序队列实现）
 * @note   front 指向队头，rear 指向队尾下一个空位。
 *         地图最大 63 格，不会出现满队列越界。
 */
typedef struct {
    Point data[MAX_CELLS];
    int front, rear;
} Queue;

/* 全局网格地图：1 表示空地，0 表示障碍物 */
int grid[ROWS][COLS];

/* 障碍物列表，以 {A, B} 坐标硬编码 */
Point barriers[BARRIER_COUNT] = {
    {7, 3},
    {8, 3},
    {9, 3}
};

/* 可达格子列表（仅本文件使用） */
static Point accessible_cells[MAX_CELLS];
int accessible_count = 0;

/* 最终拼接完成的逐格路径（供状态机读取） */
Point final_path[MAX_PATH_LENGTH];
int final_path_length = 0;

/* 路径规划全局缓冲区（static 避免栈溢出） */
static Point g_order_row[MAX_CELLS * 2];
static Point g_order_col[MAX_CELLS * 2];
static Point g_seg_buffer[MAX_PATH_LENGTH];

/* 四邻域搜索方向：右、下、左、上 */
static int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

/* 厘米级路径点数组（调试用） */
PathPoint path_points[MAX_PATH_LENGTH];
uint8_t path_len_routine = 0;

/* ---------- 队列操作 ---------- */

/**
 * @brief  初始化队列
 * @param  q  队列指针
 */
static void init_queue(Queue *q) {
    q->front = q->rear = 0;
}

/**
 * @brief  判断队列是否为空
 * @param  q  队列指针
 * @return 1 表示空，0 表示非空
 */
static int is_queue_empty(Queue *q) {
    return q->front == q->rear;
}

/**
 * @brief  入队操作
 * @param  q  队列指针
 * @param  p  待入队坐标
 */
static void enqueue(Queue *q, Point p) {
    q->data[q->rear++] = p;
}

/**
 * @brief  出队操作
 * @param  q  队列指针
 * @return 队头坐标
 */
static Point dequeue(Queue *q) {
    return q->data[q->front++];
}

/* ---------- 辅助函数 ---------- */

/**
 * @brief  检查坐标是否在地图范围内
 * @param  row  行号
 * @param  col  列号
 * @return 1 表示在范围内，0 表示越界
 */
static int is_valid(int row, int col) {
    return row >= 0 && row < ROWS && col >= 0 && col < COLS;
}

/**
 * @brief  初始化网格地图，全部设为可通行
 * @note   遍历 grid[ROWS][COLS]，每个元素赋值为 1。
 *         调用后需再执行 generate_barriers() 标记障碍物。
 */
void init_grid() {
    int i, j;
    for (i = 0; i < ROWS; i++)
        for (j = 0; j < COLS; j++)
            grid[i][j] = 1;
}

/**
 * @brief  根据硬编码障碍物初始化网格地图
 * @note   先把地图全置 1，再把障碍物位置置 0。
 *         barriers[] 中存的是 {A, B} 坐标，自动换算为 grid 索引：
 *         grid_row = B - 1，grid_col = COLS - A。
 */
void generate_barriers() {
    int i;
    init_grid();
    for (i = 0; i < BARRIER_COUNT; i++) {
        int a = barriers[i].row;
        int b = barriers[i].col;
        int grid_row = b - 1;
        int grid_col = COLS - a;
        if (is_valid(grid_row, grid_col))
            grid[grid_row][grid_col] = 0;
    }
}

/**
 * @brief  收集所有可通行格子的坐标
 * @note   扫描整个网格，把值为 1 的空地坐标存入 accessible_cells[]。
 *         accessible_count 记录实际收集到的格子数量。
 */
void collect_accessible_cells() {
    int i, j;
    accessible_count = 0;
    for (i = 0; i < ROWS; i++)
        for (j = 0; j < COLS; j++)
            if (grid[i][j] == 1)
                accessible_cells[accessible_count++] = (Point){i, j};
}

/**
 * @brief  行扫描蛇形 TSP
 * @param  order  输出数组，存放访问顺序
 * @return order 数组的有效长度
 * @note   偶数行从左到右（col 递增），奇数行从右到左（col 递减）。
 *         自动跳过障碍物所在段，适应任意连续障碍物配置。
 */
int snake_tsp(Point order[]) {
    int idx = 0;
    int r, c, s;

    for (r = 0; r < ROWS; r++) {
        int seg_starts[3];
        int seg_ends[3];
        int seg_count = 0;
        int cur_start = -1;

        for (c = 0; c < COLS; c++) {
            if (grid[r][c] == 1) {
                if (cur_start == -1) {
                    cur_start = c;
                }
            } else {
                if (cur_start != -1) {
                    seg_starts[seg_count] = cur_start;
                    seg_ends[seg_count] = c - 1;
                    seg_count++;
                    cur_start = -1;
                }
            }
        }
        if (cur_start != -1) {
            seg_starts[seg_count] = cur_start;
            seg_ends[seg_count] = COLS - 1;
            seg_count++;
        }

        if (seg_count == 0) {
            continue;
        }

        if (r % 2 == 0) {
            for (s = 0; s < seg_count; s++) {
                for (c = seg_starts[s]; c <= seg_ends[s]; c++) {
                    order[idx++] = (Point){r, c};
                }
            }
        } else {
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
 * @brief  列扫描蛇形 TSP
 * @param  order  输出数组，存放列扫描访问顺序
 * @return order 数组有效长度
 * @note   偶数列从上到下（row 递增），奇数列从下到上（row 递减）。
 *         禁飞区为垂直排列时，列扫描路径通常更短。
 */
int snake_tsp_col(Point order[]) {
    int idx = 0;
    int r, c, s;

    for (c = 0; c < COLS; c++) {
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
 * @param  path    输出数组，存放找到的最短路径
 * @param  max_len path 数组的最大容量
 * @return 路径长度（含起点和终点），不可达返回 0
 * @note   使用 static 数组避免栈溢出。第一次到达终点时即为最短路径。
 */
int find_shortest_path(Point start, Point end, Point path[], int max_len) {
    static int visited[ROWS][COLS];
    static int parent[ROWS][COLS][2];
    static Queue q;
    int r, c;
    for (r = 0; r < ROWS; r++)
        for (c = 0; c < COLS; c++)
            visited[r][c] = 0;
    init_queue(&q);
    enqueue(&q, start);
    visited[start.row][start.col] = 1;
    parent[start.row][start.col][0] = -1;
    parent[start.row][start.col][1] = -1;

    while (!is_queue_empty(&q)) {
        Point cur = dequeue(&q);
        if (cur.row == end.row && cur.col == end.col) {
            int len = 0;
            Point t = end;
            while (t.row != -1 && len < max_len) {
                path[len++] = t;
                int pr = parent[t.row][t.col][0];
                int pc = parent[t.row][t.col][1];
                t.row = pr; t.col = pc;
            }
            if (len >= max_len) return 0;
            {
                int i;
                for (i = 0; i < len / 2; i++) {
                    Point tmp = path[i];
                    path[i] = path[len - 1 - i];
                    path[len - 1 - i] = tmp;
                }
            }
            return len;
        }
        {
            int i;
            for (i = 0; i < 4; i++) {
                int nr = cur.row + directions[i][0];
                int nc = cur.col + directions[i][1];
                if (is_valid(nr, nc) && !visited[nr][nc] && grid[nr][nc]) {
                    visited[nr][nc] = 1;
                    parent[nr][nc][0] = cur.row;
                    parent[nr][nc][1] = cur.col;
                    enqueue(&q, (Point){nr, nc});
                }
            }
        }
    }

    return 0;
}

/**
 * @brief  将 TSP 访问顺序拼接为完整逐格路径
 * @param  order  TSP 计算出的访问顺序数组
 * @param  count  order 数组长度
 * @note   对每一对相邻访问点调用 BFS 找出最短路径并拼接。
 *         第 2 段起跳过每段起点（避免重复节点）。
 *         若某段 BFS 失败（不可达），final_path_length 置 0 并返回。
 */
void build_full_path(Point order[], int count) {
    int i;
    final_path_length = 0;
    for (i = 0; i < count - 1; i++) {
        Point start = order[i];
        Point end = order[i + 1];
        int seg_len = find_shortest_path(start, end, g_seg_buffer, MAX_PATH_LENGTH);
        int j;
        if (seg_len == 0) {
            final_path_length = 0;
            return;
        }
        for (j = (i == 0 ? 0 : 1); j < seg_len; j++) {
            final_path[final_path_length++] = g_seg_buffer[j];
        }
    }
}

/**
 * @brief  将网格坐标转换为实际厘米坐标
 * @note   x = row * 50cm（前后方向），y = col * 50cm（左右方向）。
 *         结果存入 path_points[]，仅用于调试输出。
 */
void real_routine()
{
    int i;
    path_len_routine = 0;
    for (i = 0; i < final_path_length; i++) {
        path_points[path_len_routine].x = GRID_SIZE_CM * final_path[i].row;
        path_points[path_len_routine].y = GRID_SIZE_CM * final_path[i].col;
        path_len_routine++;
    }
}

/**
 * @brief  路径规划入口函数
 * @note   执行完整规划流程：建图 -> 收集空地 -> 行/列蛇形 TSP -> BFS 拼接 -> 选优 -> 厘米转换。
 *         执行完后 final_path[] 和 final_path_length 即为结果。
 */
void run_path_planner(void) {
    int count_row, count_col;
    int len_row, len_col;

    generate_barriers();
    collect_accessible_cells();

    count_row = snake_tsp(g_order_row);
    count_col = snake_tsp_col(g_order_col);

    build_full_path(g_order_row, count_row);
    len_row = final_path_length;

    build_full_path(g_order_col, count_col);
    len_col = final_path_length;

    if (len_col < len_row) {
    } else {
        build_full_path(g_order_row, count_row);
    }

    real_routine();
}
