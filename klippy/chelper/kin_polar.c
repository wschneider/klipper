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
    
    // Handle the origin case where x==y==0
    if (fabs(c.x) < 1e-6 && fabs(c.y) < 1e-6) {
        // At origin, smoothly transition to target angle if this is the start of a move
        if (move_time <= 0.001) {
            // Beginning of move - check if we need to rotate to a new angle
            struct coord end_coord = move_get_coord(m, m->move_t);
            if (fabs(end_coord.x) > 1e-6 || fabs(end_coord.y) > 1e-6) {
                // Move goes somewhere - calculate target angle
                angle = atan2(end_coord.y, end_coord.x);
                if (angle - sk->commanded_pos > M_PI)
                    angle -= 2. * M_PI;
                else if (angle - sk->commanded_pos < -M_PI)
                    angle += 2. * M_PI;
                return angle;
            }
        }
        // Stay at current angle when at origin
        angle = sk->commanded_pos;
    } else {
        angle = atan2(c.y, c.x);
        // Normalize angle to minimize rotation
        if (angle - sk->commanded_pos > M_PI)
            angle -= 2. * M_PI;
        else if (angle - sk->commanded_pos < -M_PI)
            angle += 2. * M_PI;
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
