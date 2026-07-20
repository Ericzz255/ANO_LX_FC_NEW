#include "Path_Planning.h"
#include "Drv_Uart.h"

/*
 * Ground-station barrier coordinates use A=column(1..9), B=row(1..7).
 * Internally row increases forward and column increases to the aircraft's
 * right.  SLAM uses X forward-positive and Y left-positive.
 */

static const char step_chars[] = "abcde012345678";
#define STEP_CHAR_COUNT ((int)(sizeof(step_chars) - 1U))

typedef struct
{
    Point data[MAX_CELLS];
    int front;
    int rear;
} Queue;

static int grid[ROWS][COLS];

Point barriers[BARRIER_COUNT];
static u8 barriers_configured = 0;

Point final_path[MAX_PATH_LENGTH];
int final_path_length = 0;

static Point order_row[MAX_CELLS];
static Point order_col[MAX_CELLS];
static Point segment_path[MAX_CELLS];

static const int directions[4][2] = {
    {0, 1},
    {1, 0},
    {0, -1},
    {-1, 0}
};

static int is_valid(int row, int col)
{
    return row >= 0 && row < ROWS && col >= 0 && col < COLS;
}

static int point_equal(Point a, Point b)
{
    return a.row == b.row && a.col == b.col;
}

u8 PathPlanner_SetBarriers(const u8 barrier_data[BARRIER_COUNT * 2])
{
    Point pending[BARRIER_COUNT];
    int i;

    if (barrier_data == 0)
    {
        return 0;
    }

    for (i = 0; i < BARRIER_COUNT; i++)
    {
        int column_a = barrier_data[i * 2];
        int row_b = barrier_data[i * 2 + 1];

        if (column_a < 1 || column_a > COLS ||
            row_b < 1 || row_b > ROWS)
        {
            return 0;
        }

        pending[i].row = row_b;
        pending[i].col = column_a;

    }

    for (i = 0; i < BARRIER_COUNT; i++)
    {
        barriers[i] = pending[i];
    }
    barriers_configured = 1;
    final_path_length = 0;
    return 1;
}

u8 PathPlanner_HasBarrierConfiguration(void)
{
    return barriers_configured;
}

void PathPlanner_ClearBarrierConfiguration(void)
{
    int i;

    barriers_configured = 0;
    final_path_length = 0;
    for (i = 0; i < BARRIER_COUNT; i++)
    {
        barriers[i].row = 0;
        barriers[i].col = 0;
    }
}

static void queue_init(Queue *queue)
{
    queue->front = 0;
    queue->rear = 0;
}

static int queue_empty(const Queue *queue)
{
    return queue->front == queue->rear;
}

static int queue_push(Queue *queue, Point point)
{
    if (queue->rear >= MAX_CELLS)
    {
        return 0;
    }
    queue->data[queue->rear++] = point;
    return 1;
}

static Point queue_pop(Queue *queue)
{
    return queue->data[queue->front++];
}

static void init_grid(void)
{
    int row;
    int col;

    for (row = 0; row < ROWS; row++)
    {
        for (col = 0; col < COLS; col++)
        {
            grid[row][col] = 1;
        }
    }
}

void generate_barriers(void)
{
    int i;

    init_grid();
    for (i = 0; i < BARRIER_COUNT; i++)
    {
        int grid_row = barriers[i].row - 1;
        int grid_col = COLS - barriers[i].col;

        if (is_valid(grid_row, grid_col))
        {
            grid[grid_row][grid_col] = 0;
        }
    }
}

int snake_tsp(Point order[])
{
    int index = 0;
    int row;
    int col;

    for (row = 0; row < ROWS; row++)
    {
        if ((row & 1) == 0)
        {
            for (col = 0; col < COLS; col++)
            {
                if (grid[row][col])
                {
                    order[index].row = row;
                    order[index].col = col;
                    index++;
                }
            }
        }
        else
        {
            for (col = COLS - 1; col >= 0; col--)
            {
                if (grid[row][col])
                {
                    order[index].row = row;
                    order[index].col = col;
                    index++;
                }
            }
        }
    }

    return index;
}

int snake_tsp_col(Point order[])
{
    int index = 0;
    int row;
    int col;

    for (col = 0; col < COLS; col++)
    {
        if ((col & 1) == 0)
        {
            for (row = 0; row < ROWS; row++)
            {
                if (grid[row][col])
                {
                    order[index].row = row;
                    order[index].col = col;
                    index++;
                }
            }
        }
        else
        {
            for (row = ROWS - 1; row >= 0; row--)
            {
                if (grid[row][col])
                {
                    order[index].row = row;
                    order[index].col = col;
                    index++;
                }
            }
        }
    }

    return index;
}

int find_shortest_path(Point start, Point end, Point path[], int max_len)
{
    static int visited[ROWS][COLS];
    static Point parent[ROWS][COLS];
    static Queue queue;
    int row;
    int col;

    if (path == 0 || max_len <= 0 ||
        !is_valid(start.row, start.col) ||
        !is_valid(end.row, end.col) ||
        !grid[start.row][start.col] ||
        !grid[end.row][end.col])
    {
        return 0;
    }

    for (row = 0; row < ROWS; row++)
    {
        for (col = 0; col < COLS; col++)
        {
            visited[row][col] = 0;
            parent[row][col].row = -1;
            parent[row][col].col = -1;
        }
    }

    queue_init(&queue);
    if (!queue_push(&queue, start))
    {
        return 0;
    }
    visited[start.row][start.col] = 1;

    while (!queue_empty(&queue))
    {
        Point current = queue_pop(&queue);
        int direction;

        if (point_equal(current, end))
        {
            int length = 0;
            Point trace = end;
            int i;

            while (trace.row >= 0 && trace.col >= 0)
            {
                if (length >= max_len)
                {
                    return 0;
                }
                path[length++] = trace;
                trace = parent[trace.row][trace.col];
            }

            for (i = 0; i < length / 2; i++)
            {
                Point temp = path[i];
                path[i] = path[length - 1 - i];
                path[length - 1 - i] = temp;
            }
            return length;
        }

        for (direction = 0; direction < 4; direction++)
        {
            int next_row = current.row + directions[direction][0];
            int next_col = current.col + directions[direction][1];

            if (is_valid(next_row, next_col) &&
                !visited[next_row][next_col] &&
                grid[next_row][next_col])
            {
                Point next;
                next.row = next_row;
                next.col = next_col;
                visited[next_row][next_col] = 1;
                parent[next_row][next_col] = current;
                if (!queue_push(&queue, next))
                {
                    return 0;
                }
            }
        }
    }

    return 0;
}

static int build_full_path(Point order[], int count)
{
    int order_index;

    final_path_length = 0;
    if (order == 0 || count <= 0)
    {
        return 0;
    }

    final_path[final_path_length++] = order[0];

    for (order_index = 0; order_index < count - 1; order_index++)
    {
        int segment_length;
        int segment_index;

        segment_length = find_shortest_path(order[order_index],
                                            order[order_index + 1],
                                            segment_path,
                                            MAX_CELLS);
        if (segment_length <= 0)
        {
            final_path_length = 0;
            return 0;
        }

        for (segment_index = 1;
             segment_index < segment_length;
             segment_index++)
        {
            if (final_path_length >= MAX_PATH_LENGTH)
            {
                final_path_length = 0;
                return 0;
            }
            final_path[final_path_length++] = segment_path[segment_index];
        }
    }

    return final_path_length;
}

/*
 * Keep only the start, end and direction-change points.  Every removed point
 * lies on the same horizontal or vertical line between two retained points,
 * so the aircraft still passes through the original cells and all obstacle
 * detours remain unchanged.
 */
static void compress_collinear_path(void)
{
    int read_index;
    int write_index;
    int original_length = final_path_length;

    if (original_length <= 2)
    {
        return;
    }

    write_index = 1;
    for (read_index = 1;
         read_index < original_length - 1;
         read_index++)
    {
        int incoming_row =
            final_path[read_index].row -
            final_path[read_index - 1].row;
        int incoming_col =
            final_path[read_index].col -
            final_path[read_index - 1].col;
        int outgoing_row =
            final_path[read_index + 1].row -
            final_path[read_index].row;
        int outgoing_col =
            final_path[read_index + 1].col -
            final_path[read_index].col;

        if (incoming_row != outgoing_row ||
            incoming_col != outgoing_col)
        {
            final_path[write_index++] = final_path[read_index];
        }
    }

    final_path[write_index++] = final_path[original_length - 1];
    final_path_length = write_index;
}

u8 run_path_planner(void)
{
    int row_count;
    int col_count;
    int row_length;
    int col_length;

    final_path_length = 0;
    if (!barriers_configured)
    {
        return 0;
    }
    generate_barriers();

    /* The mission origin is grid(0,0); never take off if it is forbidden. */
    if (!grid[0][0])
    {
        return 0;
    }

    row_count = snake_tsp(order_row);
    col_count = snake_tsp_col(order_col);

    row_length = build_full_path(order_row, row_count);
    col_length = build_full_path(order_col, col_count);

    if (row_length <= 0 && col_length <= 0)
    {
        final_path_length = 0;
        return 0;
    }

    if (row_length > 0 && (col_length <= 0 || row_length <= col_length))
    {
        if (build_full_path(order_row, row_count) <= 0)
        {
            return 0;
        }
    }
    /* Otherwise the column route is already in final_path. */

    if (!point_equal(final_path[0], (Point){0, 0}))
    {
        final_path_length = 0;
        return 0;
    }

    compress_collinear_path();
    return 1;
}

u8 PathPlanner_PointToSlam(Point point, s16 *x_cm, s16 *y_cm)
{
    if (x_cm == 0 || y_cm == 0 ||
        !is_valid(point.row, point.col))
    {
        return 0;
    }

    *x_cm = (s16)(point.row * GRID_SIZE_CM);
    *y_cm = (s16)(point.col * GRID_SIZE_CM);
    return 1;
}

void send_step_feedback(int wp_index)
{
    u8 buffer[6];
    Point point;

    if (wp_index < 0 || wp_index >= final_path_length)
    {
        return;
    }

    point = final_path[wp_index];
    buffer[0] = 0x5A;
    buffer[1] = (u8)step_chars[wp_index % STEP_CHAR_COUNT];
    buffer[2] = (u8)(COLS - point.col);
    buffer[3] = (u8)(point.row + 1);
    buffer[4] = 0x5F;
    buffer[5] = (u8)(buffer[1] + buffer[2] + buffer[3]);
    DrvUart2SendBuf(buffer, 6);
}
