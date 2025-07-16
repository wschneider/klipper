// Polar kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

static double
polar_stepper_radius_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return sqrt(c.x*c.x + c.y*c.y);
}

static double
polar_stepper_angle_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    double angle;
    
    // Handle the origin case where x and y are very close to zero
    // Use epsilon tolerance to handle floating-point precision errors
    double epsilon = 1e-9;  // 1 nanometer tolerance
    if (fabs(c.x) < epsilon && fabs(c.y) < epsilon) {
        // At origin (within tolerance), maintain current angle to avoid undefined atan2(0,0)
        angle = sk->commanded_pos;
    } else {
        angle = atan2(c.y, c.x);
        // Normalize angle to minimize rotation with better precision handling
        double angle_diff = angle - sk->commanded_pos;
        if (angle_diff > M_PI)
            angle_diff -= 2. * M_PI;
        else if (angle_diff < -M_PI)
            angle_diff += 2. * M_PI;
        
        // If the angle difference is very small, just use the current position
        if (fabs(angle_diff) < 1e-6)  // About 0.0001 degrees
            angle = sk->commanded_pos;
        else
            angle = sk->commanded_pos + angle_diff;
    }
    return angle;
}

static void
polar_stepper_angle_post_fixup(struct stepper_kinematics *sk)
{
    // Normalize the stepper_bed angle
    if (sk->commanded_pos < -M_PI)
        sk->commanded_pos += 2 * M_PI;
    else if (sk->commanded_pos > M_PI)
        sk->commanded_pos -= 2 * M_PI;
}

struct stepper_kinematics * __visible
polar_stepper_alloc(char type)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (type == 'r') {
        sk->calc_position_cb = polar_stepper_radius_calc_position;
    } else if (type == 'a') {
        sk->calc_position_cb = polar_stepper_angle_calc_position;
        sk->post_cb = polar_stepper_angle_post_fixup;
    }
    sk->active_flags = AF_X | AF_Y;
    return sk;
}
