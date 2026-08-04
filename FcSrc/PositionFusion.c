#include "PositionFusion.h"
#include "Drv_AnoOf.h"
#include "Drv_Sys.h"

/* Set to 0 for an immediate return to pure SLAM positioning. */
#define POSITION_FUSION_USE_OPTICAL_FLOW       1U

/*
 * Coordinate mapping for the optical-flow velocity frame. The current
 * installation uses X-forward/Y-left, matching the SLAM and 0x41 command
 * frames. Change either sign after a hand-motion bench test if required.
 */
#define POSITION_FUSION_OF_X_SIGN              1.0f
#define POSITION_FUSION_OF_Y_SIGN              1.0f

/* SLAM is the primary source; optical flow deliberately has low weight. */
#define POSITION_FUSION_SLAM_R_CM2             9.0f
#define POSITION_FUSION_OF_R_CMPSS2          400.0f
#define POSITION_FUSION_ACC_STD_CMPSS         30.0f
#define POSITION_FUSION_INITIAL_POS_VAR        4.0f
#define POSITION_FUSION_INITIAL_VEL_VAR       25.0f

/* Reject flow when reflection/noise makes the module report poor data. */
#define POSITION_FUSION_OF_MIN_QUALITY        30U
#define POSITION_FUSION_OF_MAX_SPEED_CMPS     80.0f
#define POSITION_FUSION_OF_ACTIVE_MS         300U

typedef struct
{
    float pos;
    float vel;
    float p00;
    float p01;
    float p10;
    float p11;
} PositionFusionAxis;

static PositionFusionAxis fusion_axis[2];
static u8 fusion_initialized;
static u8 flow_update_cnt_last;
static u32 flow_last_accepted_ms;
static u8 flow_ever_accepted;

static float PositionFusion_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static s16 PositionFusion_FloatToS16(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }
    if (value < -32768.0f)
    {
        return (s16)-32768;
    }
    return (value >= 0.0f) ? (s16)(value + 0.5f) :
                             (s16)(value - 0.5f);
}

static void PositionFusion_LimitCovariance(PositionFusionAxis *axis)
{
    float cross = 0.5f * (axis->p01 + axis->p10);

    axis->p01 = cross;
    axis->p10 = cross;
    if (axis->p00 < 0.0001f)
    {
        axis->p00 = 0.0001f;
    }
    if (axis->p11 < 0.0001f)
    {
        axis->p11 = 0.0001f;
    }
}

static void PositionFusion_PredictAxis(PositionFusionAxis *axis,
                                       float dt_s)
{
    float q = POSITION_FUSION_ACC_STD_CMPSS *
              POSITION_FUSION_ACC_STD_CMPSS;
    float dt2 = dt_s * dt_s;
    float dt3 = dt2 * dt_s;
    float dt4 = dt2 * dt2;
    float old_p00 = axis->p00;
    float old_p01 = axis->p01;
    float old_p10 = axis->p10;
    float old_p11 = axis->p11;

    axis->pos += axis->vel * dt_s;
    axis->p00 = old_p00 + dt_s * (old_p01 + old_p10) +
                dt2 * old_p11 + 0.25f * q * dt4;
    axis->p01 = old_p01 + dt_s * old_p11 + 0.5f * q * dt3;
    axis->p10 = old_p10 + dt_s * old_p11 + 0.5f * q * dt3;
    axis->p11 = old_p11 + q * dt2;
    PositionFusion_LimitCovariance(axis);
}

static void PositionFusion_UpdatePosition(PositionFusionAxis *axis,
                                          float measurement)
{
    float old_p00 = axis->p00;
    float old_p01 = axis->p01;
    float old_p10 = axis->p10;
    float old_p11 = axis->p11;
    float innovation = measurement - axis->pos;
    float innovation_var = old_p00 + POSITION_FUSION_SLAM_R_CM2;
    float k_pos = old_p00 / innovation_var;
    float k_vel = old_p10 / innovation_var;

    axis->pos += k_pos * innovation;
    axis->vel += k_vel * innovation;
    axis->p00 = old_p00 - k_pos * old_p00;
    axis->p01 = old_p01 - k_pos * old_p01;
    axis->p10 = old_p10 - k_vel * old_p00;
    axis->p11 = old_p11 - k_vel * old_p01;
    PositionFusion_LimitCovariance(axis);
}

static void PositionFusion_UpdateVelocity(PositionFusionAxis *axis,
                                          float measurement)
{
    float old_p00 = axis->p00;
    float old_p01 = axis->p01;
    float old_p10 = axis->p10;
    float old_p11 = axis->p11;
    float innovation = measurement - axis->vel;
    float innovation_var = old_p11 + POSITION_FUSION_OF_R_CMPSS2;
    float k_pos = old_p01 / innovation_var;
    float k_vel = old_p11 / innovation_var;

    axis->pos += k_pos * innovation;
    axis->vel += k_vel * innovation;
    axis->p00 = old_p00 - k_pos * old_p10;
    axis->p01 = old_p01 - k_pos * old_p11;
    axis->p10 = old_p10 - k_vel * old_p10;
    axis->p11 = old_p11 - k_vel * old_p11;
    PositionFusion_LimitCovariance(axis);
}

void PositionFusion_Reset(void)
{
    u8 axis_index;

    for (axis_index = 0U; axis_index < 2U; axis_index++)
    {
        fusion_axis[axis_index].pos = 0.0f;
        fusion_axis[axis_index].vel = 0.0f;
        fusion_axis[axis_index].p00 = POSITION_FUSION_INITIAL_POS_VAR;
        fusion_axis[axis_index].p01 = 0.0f;
        fusion_axis[axis_index].p10 = 0.0f;
        fusion_axis[axis_index].p11 = POSITION_FUSION_INITIAL_VEL_VAR;
    }
    fusion_initialized = 0U;
    flow_update_cnt_last = ano_of.of_update_cnt;
    flow_last_accepted_ms = 0U;
    flow_ever_accepted = 0U;
}

void PositionFusion_PushSlam(s16 x_cm, s16 y_cm)
{
    if (fusion_initialized == 0U)
    {
        fusion_axis[0].pos = (float)x_cm;
        fusion_axis[1].pos = (float)y_cm;
        fusion_axis[0].vel = 0.0f;
        fusion_axis[1].vel = 0.0f;
        fusion_initialized = 1U;
        return;
    }

    PositionFusion_UpdatePosition(&fusion_axis[0], (float)x_cm);
    PositionFusion_UpdatePosition(&fusion_axis[1], (float)y_cm);
}

void PositionFusion_Update(float dt_s)
{
    u8 flow_new;

    if (fusion_initialized == 0U || dt_s <= 0.0f || dt_s > 0.1f)
    {
        return;
    }

    PositionFusion_PredictAxis(&fusion_axis[0], dt_s);
    PositionFusion_PredictAxis(&fusion_axis[1], dt_s);

    flow_new = (ano_of.of_update_cnt != flow_update_cnt_last) ? 1U : 0U;
    if (flow_new != 0U)
    {
        float vx_cmps;
        float vy_cmps;

        flow_update_cnt_last = ano_of.of_update_cnt;
#if POSITION_FUSION_USE_OPTICAL_FLOW
        vx_cmps = (float)ano_of.of1_dx * POSITION_FUSION_OF_X_SIGN;
        vy_cmps = (float)ano_of.of1_dy * POSITION_FUSION_OF_Y_SIGN;

        if (ano_of.link_sta != 0U &&
            ano_of.of1_sta != 0U &&
            ano_of.of_quality >= POSITION_FUSION_OF_MIN_QUALITY &&
            PositionFusion_Abs(vx_cmps) <= POSITION_FUSION_OF_MAX_SPEED_CMPS &&
            PositionFusion_Abs(vy_cmps) <= POSITION_FUSION_OF_MAX_SPEED_CMPS)
        {
            PositionFusion_UpdateVelocity(&fusion_axis[0], vx_cmps);
            PositionFusion_UpdateVelocity(&fusion_axis[1], vy_cmps);
            flow_last_accepted_ms = GetSysRunTimeMs();
            flow_ever_accepted = 1U;
        }
#else
        (void)vx_cmps;
        (void)vy_cmps;
#endif
    }
}

u8 PositionFusion_GetPosition(s16 *x_cm, s16 *y_cm)
{
    if (fusion_initialized == 0U || x_cm == 0 || y_cm == 0)
    {
        return 0U;
    }

    *x_cm = PositionFusion_FloatToS16(fusion_axis[0].pos);
    *y_cm = PositionFusion_FloatToS16(fusion_axis[1].pos);
    return 1U;
}

u8 PositionFusion_GetVelocity(s16 *vx_cmps, s16 *vy_cmps)
{
    if (fusion_initialized == 0U || vx_cmps == 0 || vy_cmps == 0)
    {
        return 0U;
    }

    *vx_cmps = PositionFusion_FloatToS16(fusion_axis[0].vel);
    *vy_cmps = PositionFusion_FloatToS16(fusion_axis[1].vel);
    return 1U;
}

u8 PositionFusion_IsOpticalFlowActive(void)
{
    if (flow_ever_accepted == 0U)
    {
        return 0U;
    }

    return ((u32)(GetSysRunTimeMs() - flow_last_accepted_ms) <=
            POSITION_FUSION_OF_ACTIVE_MS) ? 1U : 0U;
}
