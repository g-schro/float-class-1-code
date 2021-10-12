/*
 * @brief Implementation of draw module.
 *
 * This module drives a drawing machine based on a 2-joint planer robot. It
 * makes use of the step module for driving the stepper motors.
 *
 * The following console commands are provided:
 * > draw status
 * > draw tset
 * See code for details.
 *
 * MIT License
 * 
 * Copyright (c) 2021 Eugene R Schroeder
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "cmd.h"
#include "console.h"
#include "log.h"
#include "module.h"

#include "draw.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

#define PI_F ((float)M_PI)
#define NUM_MOTORS 2
#define NUM_CART_DIM 2
#define DFLT_MS_PER_STEP 4
#define DFLT_LINEAR_MM_PER_SEG 2.0f

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

enum move_state {
    MOVE_STATE_IDLE,
    MOVE_STATE_READY,
    MOVE_STATE_ACTIVE,
    MOVE_STATE_NOT_CALIB,
};

struct state
{
    struct draw_cfg cfg;
    uint32_t ms_per_step;
    float linear_mm_per_seg;

    // Following are used for all move types.
    enum move_state move_state;
    enum draw_move_type move_type;
    int32_t move_dest_steps[NUM_MOTORS];
    float move_dest_mm[NUM_CART_DIM];

    // Following are used for all linear moves only.
    float move_distance_mm[NUM_CART_DIM];
    int32_t last_seg_dest_steps[NUM_MOTORS];
    float seg_distance_mm[NUM_CART_DIM];
    uint32_t num_segments;
    uint32_t crnt_seg_num;

    // This is the tool point position. It is updated when a move completes
    // from the viewpoint of this module; the actual physical move might not be
    // complete.
    int32_t base_steps[NUM_MOTORS];
    float base_mm[NUM_CART_DIM];

    float steps_per_radian[NUM_MOTORS];
};
    
////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static int32_t start_ready_move(void);
static int32_t process_active_move(void);
static int32_t move_next_seg(void);
static int32_t solve_fk(float theta_1_rad, float theta_2_rad,
                        float* x_mm, float* y_mm);
static int32_t solve_ik(float x_mm, float y_mm,
                        float* theta_1_rad, float* theta_2_rad);
static int32_t joint_calib(float theta_1_rad, float theta_2_rad);
static int32_t jog_motor(uint32_t motor_idx, float jog_degrees);

static int32_t cmd_draw_status(int32_t argc, const char** argv);
static int32_t cmd_draw_test(int32_t argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

static struct state state;

static int32_t log_level = LOG_DEFAULT;

// Data structure with console command info.
static struct cmd_cmd_info cmds[] = {
    {
        .name = "status",
        .func = cmd_draw_status,
        .help = "Get module status, usage: draw status",
    },
    {
        .name = "test",
        .func = cmd_draw_test,
        .help = "Run test, usage: draw test [<op> [<arg>]] (enter no op/arg for help)",
    }
};

// Data structure passed to cmd module for console interaction.
static struct cmd_client_info cmd_info = {
    .name = "draw",
    .num_cmds = ARRAY_SIZE(cmds),
    .cmds = cmds,
    .log_level_ptr = &log_level,
};

static const float rad_to_deg_factor = 180.0F / PI_F;
static const float deg_to_rad_factor = PI_F / 180.0F;

#define rad_to_deg(r) ((r) * rad_to_deg_factor)
#define deg_to_rad(d) ((d) * deg_to_rad_factor)

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Get default draw configuration.
 *
 * @param[out] cfg The draw configuration with defaults filled in.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t draw_get_def_cfg(struct draw_cfg* cfg)
{
    cfg->step_motor_instance[0] = CONFIG_DRAW_DFLT_STEP_INSTANCE_1;
    cfg->step_motor_instance[1] = CONFIG_DRAW_DFLT_STEP_INSTANCE_2;
    cfg->link_len_mm[0] = CONFIG_DRAW_DFLT_LINK_1_LEN_MM;
    cfg->link_len_mm[1] = CONFIG_DRAW_DFLT_LINK_2_LEN_MM;

    return 0;
}

/*
 * @brief Initialize draw instance.
 *
 * @param[in] cfg The draw configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes the draw singleton module. Generally, it should
 * not access other modules as they might not have been initialized yet. An
 * exception is the log module.
 */
int32_t draw_init(struct draw_cfg* cfg)
{  
    memset(&state, 0, sizeof(state));

    state.cfg = *cfg;
    state.move_state = MOVE_STATE_NOT_CALIB;
    state.ms_per_step = DFLT_MS_PER_STEP;
    state.linear_mm_per_seg = DFLT_LINEAR_MM_PER_SEG;
    return 0;
}

/*
 * @brief Start draw instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function starts the draw singleton module, to enter normal operation.
 */
int32_t draw_start(void)
{
    int32_t rc;
    uint32_t idx;
    struct step_motor_info smi;

    rc = cmd_register(&cmd_info);
    if (rc < 0) {
        log_error("draw_start: cmd error %d\n", rc);
        return MOD_ERR_RESOURCE;
    }

    for (idx = 0; idx < NUM_MOTORS; idx++)
    {
        rc = step_get_info(state.cfg.step_motor_instance[idx], &smi);
        if (rc != 0)
            return rc;
        state.steps_per_radian[idx] = (float)smi.steps_per_rev/(PI_F * 2.0f);
    }
    return 0;
}

/*
 * @brief Run draw instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note This function should not block.
 *
 * This function runs the draw singleton module, during normal operation.
 */
int32_t draw_run(void)
{
    int rc = 0;

    if (state.move_state == MOVE_STATE_IDLE)
        return 0;

    // If we don't have open command slots for both motors, nothing we can do.
    if (step_get_free_cmd_slots(state.cfg.step_motor_instance[0]) <= 0 ||
        step_get_free_cmd_slots(state.cfg.step_motor_instance[1]) <= 0)
    {
        return 0;
    }

    if (state.move_state == MOVE_STATE_READY)
        rc = start_ready_move();
    else if (state.move_state == MOVE_STATE_ACTIVE)
        rc = process_active_move();
    else
        rc = MOD_ERR_INTERNAL;

    return rc;
}

/*
 * @brief Move to a position.
 *
 * @param[in] x_mm Location on x axis in mm.
 * @param[in] y_mm Location on y axis in mm.
 *
 * @param[in] argv Argument values, including "draw"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t draw_move_to(float x_mm, float y_mm, enum draw_move_type move_type)
{
    float move_dest_radians[NUM_MOTORS];
    int32_t rc;
    uint32_t idx;

    if (state.move_state != MOVE_STATE_IDLE)
        return MOD_ERR_STATE;

    rc = solve_ik(x_mm, y_mm, &move_dest_radians[0], &move_dest_radians[1]);
    if (rc != 0) {
        return rc;
    }

    state.move_dest_mm[0] = x_mm;
    state.move_dest_mm[1] = y_mm;

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        state.move_dest_steps[idx] = roundf(move_dest_radians[idx] *
                                            state.steps_per_radian[idx]);
    }
    state.move_type = move_type;
    state.move_state = MOVE_STATE_READY;

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Start a move that is in the "ready" state.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t start_ready_move(void)
{
    int32_t rc;
    uint32_t max_steps = 0;
    uint32_t idx;
    CRIT_STATE_VAR;
    float move_abs_distance_mm;

    if (state.move_type == DRAW_MOVE_TYPE_JOINT) {
        int32_t abs_steps;
        for (idx = 0; idx < NUM_MOTORS; idx++) {
            abs_steps = (state.move_dest_steps[idx] -
                         state.base_steps[idx]);
            if (abs_steps < 0)
                abs_steps = -abs_steps;
            if (abs_steps > max_steps)
                max_steps = abs_steps;
        }

        CRIT_BEGIN_NEST();
        for (idx = 0; idx < NUM_MOTORS; idx++) {
            rc = step_add_cmd(state.cfg.step_motor_instance[idx],
                              STEP_CMD_TO_P_IN_M,
                              state.move_dest_steps[idx],
                              max_steps * state.ms_per_step);
            if (rc < 0)
                break;
        }
        CRIT_END_NEST();

        if (rc < 0) {
            log_error("start_ready_move: step_add_cmd fails rc=%ld\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
            return rc;
        }
        
        // Move is done from this module's viewpoint.
        for (idx = 0; idx < NUM_MOTORS; idx++)
            state.base_steps[idx] = state.move_dest_steps[idx];
        for (idx = 0; idx < NUM_CART_DIM; idx++)
            state.base_mm[idx] = state.move_dest_mm[idx];
        state.move_state = MOVE_STATE_IDLE;

    } else if (state.move_type == DRAW_MOVE_TYPE_LINEAR) {
        state.crnt_seg_num = 0;
        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            state.move_distance_mm[idx] =
                (state.move_dest_mm[idx] - state.base_mm[idx]);
            state.last_seg_dest_steps[idx] = state.base_steps[idx];
        }
        move_abs_distance_mm = sqrtf((state.move_distance_mm[0] *
                                      state.move_distance_mm[0]) +
                                     (state.move_distance_mm[1] *
                                      state.move_distance_mm[1]));

        state.num_segments = ceilf(move_abs_distance_mm /
                                   state.linear_mm_per_seg);
        log_debug("start_ready_move num_segments=%lu\n", state.num_segments);
        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            state.seg_distance_mm[idx] =
                state.move_distance_mm[idx] / (float)state.num_segments;
        }
        rc = move_next_seg();
        state.move_state = rc == 0 ? MOVE_STATE_ACTIVE : MOVE_STATE_NOT_CALIB;
    } else {
        rc = MOD_ERR_INTERNAL;
        state.move_state = MOVE_STATE_NOT_CALIB;
    }
    return rc;
}

/*
 * @brief Process a move in the active state.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t process_active_move(void)
{
    return move_next_seg();
}

/*
 * @brief Start the next segment in a multi-segment move.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t move_next_seg(void)
{
    int32_t rc;
    uint32_t idx;
    float seg_dest_mm[NUM_CART_DIM];
    float seg_dest_radians[NUM_MOTORS];
    int32_t seg_dest_steps[NUM_MOTORS];
    int32_t num_steps;
    uint32_t max_steps = 0;
    CRIT_STATE_VAR;

    log_debug("move_next_seg crnt_seg_num=%lu\n", state.crnt_seg_num);

    if (state.move_type != DRAW_MOVE_TYPE_LINEAR ||
        state.crnt_seg_num >= state.num_segments)
        return MOD_ERR_INTERNAL;

    state.crnt_seg_num++;

    if (state.crnt_seg_num >= state.num_segments) {
        // For the last segment, use the move destination.
        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            seg_dest_mm[idx] = state.move_dest_mm[idx];
        }
    } else {
        // Calculate the segment destination.
        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            seg_dest_mm[idx] = state.base_mm[idx] +
                state.seg_distance_mm[idx] * (float)state.crnt_seg_num;
        }
    }

    rc = solve_ik(seg_dest_mm[0], seg_dest_mm[1],
                  &seg_dest_radians[0], &seg_dest_radians[1]);

    if (rc != 0) {
        // Presumably we encountered a point outside the workspace.
        log_error("move_next_seg: solve_ik fails rc=%ld\n", rc);
        state.move_state = MOVE_STATE_NOT_CALIB;
        return rc;
    }

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        seg_dest_steps[idx] = (int32_t)(seg_dest_radians[idx] *
                                        state.steps_per_radian[idx]);
        num_steps = seg_dest_steps[idx] - state.last_seg_dest_steps[idx];
        if (num_steps < 0)
            num_steps = -num_steps;
        if  (num_steps > max_steps)
            max_steps = num_steps;
    }

    if (max_steps > 0) {
        CRIT_BEGIN_NEST();
        for (idx = 0; idx < NUM_MOTORS; idx++) {
            rc = step_add_cmd(state.cfg.step_motor_instance[idx],
                              STEP_CMD_TO_P_IN_M,
                              seg_dest_steps[idx],
                              max_steps * state.ms_per_step);
            if (rc < 0)
                break;
        }
        CRIT_END_NEST();

        if (rc < 0) {
            log_error("move_next_seg: step_add_cmd fails rc=%ld\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
            return rc;
        }

        for (idx = 0; idx < NUM_MOTORS; idx++)
            state.last_seg_dest_steps[idx] = seg_dest_steps[idx];
    }

    // Check if this was the last segment.
    if (state.crnt_seg_num >= state.num_segments) {
        for (idx = 0; idx < NUM_MOTORS; idx++)
            state.base_steps[idx] = state.move_dest_steps[idx];
        for (idx = 0; idx < NUM_CART_DIM; idx++)
            state.base_mm[idx] = state.move_dest_mm[idx];
        state.move_state = MOVE_STATE_IDLE;
    }
    return rc;
}

static int32_t solve_fk(float theta_1_rad, float theta_2_rad,
                        float* x_mm, float* y_mm)
{
    float theta_sum_rad = theta_1_rad + theta_2_rad;

    *x_mm = (state.cfg.link_len_mm[0] * cosf(theta_1_rad) +
          state.cfg.link_len_mm[1] * cosf(theta_sum_rad));
    *y_mm = (state.cfg.link_len_mm[0] * sinf(theta_1_rad) +
             state.cfg.link_len_mm[1] * sinf(theta_sum_rad));

    return 0;
}

static int32_t solve_ik(float x_mm, float y_mm,
                        float* theta_1_rad, float* theta_2_rad)
{
    const float len_1_mm = state.cfg.link_len_mm[0];
    const float len_2_mm = state.cfg.link_len_mm[1];
    static int init = 0;
    static float two_len_1;
    static float two_len_1_len_2;
    static float len_1_sqr_p_len_2_sqr;
    static float len_1_sqr_m_len_2_sqr;

    float x_sqr = x_mm * x_mm;
    float y_sqr = y_mm * y_mm;
    float x_sqr_p_y_sqr = x_sqr + y_sqr;

    float beta;
    float alpha;
    float gamma;

    if (!init) {
        init = 1;
        two_len_1 = 2.0F * len_1_mm;
        two_len_1_len_2 = 2.0F * len_1_mm * len_2_mm;
        len_1_sqr_p_len_2_sqr = len_1_mm * len_1_mm + len_2_mm * len_2_mm;
        len_1_sqr_m_len_2_sqr = len_1_mm * len_1_mm - len_2_mm * len_2_mm;
    }

    alpha = acosf((x_sqr_p_y_sqr + len_1_sqr_m_len_2_sqr)/
                  (two_len_1 * sqrtf(x_sqr_p_y_sqr)));
    if (isnanf(alpha))
        return MOD_ERR_INFEASIBLE;
    beta = acosf((len_1_sqr_p_len_2_sqr - x_sqr_p_y_sqr)/
                 two_len_1_len_2);
    if (isnanf(beta))
        return MOD_ERR_INFEASIBLE;
    
    gamma = atan2f(y_mm, x_mm);
    *theta_1_rad = gamma - alpha;
    *theta_2_rad = PI_F - beta;
    return 0;
}

/*
 * @brief Joint calibration.
 *
 * @param[in] theta_1_rad  Motor angle 1 in radians.
 * @param[in] theta_2_rad  Motor angle 2 in radians.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t joint_calib(float theta_1_rad, float theta_2_rad)
{
    int rc = 0;
    float pos_mm[NUM_CART_DIM];
    int32_t pos_steps[NUM_MOTORS] = {
        theta_1_rad * state.steps_per_radian[0],
        theta_2_rad * state.steps_per_radian[1]
    };
    uint32_t idx;

    if (state.move_state != MOVE_STATE_IDLE &&
        state.move_state != MOVE_STATE_NOT_CALIB) {
        return MOD_ERR_STATE;
    }

    rc = solve_fk(theta_1_rad, theta_2_rad, &pos_mm[0], &pos_mm[1]);
    if (rc != 0) {
        log_error("joint_calib: solve_fk fails rc=%d\n", rc);
        return rc;
    }

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        rc = step_set_position(state.cfg.step_motor_instance[idx],
                               pos_steps[idx]);
        if (rc != 0) {
            log_error("joint_calib: step_set_position 1 fails rc=%d\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
            return rc;
        }
    }

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        state.base_steps[idx] = pos_steps[idx];
    }
    for (idx = 0; idx < NUM_CART_DIM; idx++) {
        state.base_mm[idx] = pos_mm[idx];
    }
    state.move_state = MOVE_STATE_IDLE;
    return rc;
}    

/*
 * @brief Jog motor.
 *
 * @param[in] motor_idx Motor to jog (0-based).
 * @param[in] jog_degrees Number of degrees to jog.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t jog_motor(uint32_t motor_idx, float jog_degrees)
{
    int32_t jog_steps = roundf(state.steps_per_radian[motor_idx] *
                               deg_to_rad(jog_degrees));
    int32_t rc;

    if (motor_idx >= NUM_MOTORS)
        return MOD_ERR_ARG;

    if (step_get_free_cmd_slots(
            state.cfg.step_motor_instance[motor_idx]) <= 0) {
        return MOD_ERR_BUSY;
    }

    if (state.move_state != MOVE_STATE_IDLE &&
        state.move_state != MOVE_STATE_NOT_CALIB) {
        return MOD_ERR_BUSY;
    }

    state.base_steps[motor_idx] += jog_steps;

    rc = step_add_cmd(state.cfg.step_motor_instance[motor_idx],
                      STEP_CMD_TO_P_IN_M,
                      state.base_steps[motor_idx],
                      (jog_steps > 0 ? jog_steps : -jog_steps) *
                      state.ms_per_step);
    if (rc < 0) {
        log_error("joint_calib: step_add_cmd fails rc=%ld\n", rc);
        state.move_state = MOVE_STATE_NOT_CALIB;
    } else {
        rc = solve_fk(state.base_steps[0]/state.steps_per_radian[0],
                      state.base_steps[1]/state.steps_per_radian[1],
                      &state.base_mm[0], &state.base_mm[1]);
        if (rc < 0) {
            log_error("joint_calib: solve_fk fails rc=%ld\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
        }
    }
    return rc;
}

/*
 * @brief Console command function for "draw status".
 *
 * @param[in] argc Number of arguments, including "draw"
 * @param[in] argv Argument values, including "draw"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: draw status
 */
static int32_t cmd_draw_status(int32_t argc, const char** argv)
{
    printc("Move: state=%d type=%d move_step_steps=[%ld, %ld]\n",
           state.move_state, state.move_type, state.move_dest_steps[0],
           state.move_dest_steps[1]);
    printc("      last_seg_dest_steps=[%ld, %ld] base_steps=[%ld, %ld]\n",
           state.last_seg_dest_steps[0], state.last_seg_dest_steps[1], 
           state.base_steps[0], state.base_steps[1]);

    printc_float("      base_mm=[", state.base_mm[0], 3, ", ");
    printc_float(NULL, state.base_mm[1], 3, "]\n");

    printc("      num_segments=%lu crnt_seg_num=%lu\n", state.num_segments,
           state.crnt_seg_num);
    printc("ms_per_step=%lu linear_mm_per_seg=", state.ms_per_step);
    printc_float(NULL, state.linear_mm_per_seg, 1, "\n");
    return 0;
}

/*
 * @brief Console command function for "draw test".
 *
 * @param[in] argc Number of arguments, including "draw"
 * @param[in] argv Argument values, including "draw"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: draw test [<op> [<arg>]]
 */
static int32_t cmd_draw_test(int32_t argc, const char** argv)
{
    int32_t rc = 0;
    int32_t num_args;
    struct cmd_arg_val arg_vals[3];
    float theta_1;
    float theta_2;
    float x;
    float y;

    // Handle help case.
    if (argc == 2) {
        printc("Test operations and param(s) are as follows:\n"
               "  Compute IK, usage: draw test ik <x> <y>\n"
               "  Compute FK, usage: draw test fk <t1> <t2>\n");
        printc("  Set zero angle position, usage: draw test zero\n"
               "  Set jog joint, usage: draw test jog {1|2} <deg>\n"
               "  Move, usage: draw move x y {j|l}\n");
        printc("  Set speed, usage: draw test speed <ms-per-step>\n"
               "  Set mm per seg, usage: draw test mm <mm-per-seg>\n");

        return 0;
    }

    if (strcasecmp(argv[2], "ik") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "ff", arg_vals);
        if (num_args != 2) {
            return MOD_ERR_BAD_CMD;
        }
        rc = solve_ik(arg_vals[0].val.f,
                      arg_vals[1].val.f,
                      &theta_1,
                      &theta_2);
        printc_float("theta_1=", rad_to_deg(theta_1), 3, NULL);
        printc_float(" theat2=", rad_to_deg(theta_2), 3, "\n");
    } else if (strcasecmp(argv[2], "fk") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "ff", arg_vals);
        if (num_args != 2) {
            return MOD_ERR_BAD_CMD;
        }
        rc = solve_fk(deg_to_rad(arg_vals[0].val.f),
                      deg_to_rad(arg_vals[1].val.f),
                      &x,
                      &y);
        printc_float("x=", x, 4, NULL);
        printc_float(" y=", y, 4, "\n");
    } else if (strcasecmp(argv[2], "zero") == 0) {
        rc = joint_calib(0.0f, 0.0f);

    } else if (strcasecmp(argv[2], "jog") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "if", arg_vals);
        if (num_args != 2) {
            return MOD_ERR_BAD_CMD;
        }
        if (arg_vals[0].val.i != 1 && arg_vals[0].val.i != 2) {
            printc("Invalid joint\n");
            return MOD_ERR_ARG;
        }
        rc = jog_motor(arg_vals[0].val.i - 1, arg_vals[1].val.f);
    } else if (strcasecmp(argv[2], "move") == 0) {
        enum draw_move_type move_type = DRAW_MOVE_TYPE_JOINT;
        num_args = cmd_parse_args(argc-3, argv+3, "ffs", arg_vals);
        if (num_args != 3)
            return MOD_ERR_BAD_CMD;
        if (strcasecmp(arg_vals[2].val.s, "j") == 0) {
            move_type = DRAW_MOVE_TYPE_JOINT;
        } else if (strcasecmp(arg_vals[2].val.s, "l") == 0) {
            move_type = DRAW_MOVE_TYPE_LINEAR;
        } else {
            printc("Invalid move type '%s'\n", arg_vals[2].val.s);
            return MOD_ERR_BAD_CMD;
        }
        rc = draw_move_to(arg_vals[0].val.f, arg_vals[1].val.f, move_type);
    } else if (strcasecmp(argv[2], "speed") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "u", arg_vals);
        if (num_args != 1)
            return MOD_ERR_BAD_CMD;
        state.ms_per_step = arg_vals[0].val.u;
    } else if (strcasecmp(argv[2], "mm") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "f", arg_vals);
        if (num_args != 1)
            return MOD_ERR_BAD_CMD;
        state.linear_mm_per_seg = arg_vals[0].val.f;
    } else {
        printc("Invalid test '%s'\n", argv[2]);
        return MOD_ERR_BAD_CMD;
    }
    printc("Return code %ld\n", rc);
    return 0;
}
