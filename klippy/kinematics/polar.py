# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper


# Fixing polar kinematics:
# 1. All moves that cross the origin x=0, y=0 should be split into two moves
# 2. Moves with a start position at the origin should then have a rotation step
#       before move

class PolarKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis steppers
        stepper_bed = stepper.PrinterStepper(config.getsection('stepper_bed'),
                                             units_in_radians=True)
        rail_arm = stepper.LookupRail(config.getsection('stepper_arm'))
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        stepper_bed.setup_itersolve('polar_stepper_alloc', b'a')
        rail_arm.setup_itersolve('polar_stepper_alloc', b'r')
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails = [rail_arm, rail_z]
        self.steppers = [stepper_bed] + [ s for r in self.rails
                                          for s in r.get_steppers() ]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.)

        # Max radial velocity is linear velocity at the edge of the bed
        self.max_r_velocity = toolhead.max_velocity / max_xy
        self.max_r_accel = toolhead.max_accel / max_xy

    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.steppers[0].get_name()]
        arm_pos = stepper_positions[self.rails[0].get_name()]
        z_pos = stepper_positions[self.rails[1].get_name()]
        return [math.cos(bed_angle) * arm_pos, math.sin(bed_angle) * arm_pos,
                z_pos]
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if "z" in homing_axes:
            self.limit_z = self.rails[1].get_range()
        if "x" in homing_axes and "y" in homing_axes:
            self.limit_xy2 = self.rails[0].get_range()[1]**2
    def clear_homing_state(self, clear_axes):
        if "x" in clear_axes or "y" in clear_axes:
            # X and Y cannot be cleared separately
            self.limit_xy2 = -1.
        if "z" in clear_axes:
            self.limit_z = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        if axis == 0:
            homepos[1] = 0.
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Always home XY together
        homing_axes = homing_state.get_axes()
        home_xy = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_xy:
            updated_axes = [0, 1]
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        # Do actual homing
        if home_xy:
            self._home_axis(homing_state, 0, self.rails[0])
        if home_z:
            self._home_axis(homing_state, 2, self.rails[1])
    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0]**2 + end_pos[1]**2
        if xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)

        # Apply angular velocity limit for moves near the origin
        # Calculate the minimum radius during the move to determine the most restrictive constraint
        start_r = math.sqrt(move.start_pos[0]**2 + move.start_pos[1]**2)
        end_r = math.sqrt(end_pos[0]**2 + end_pos[1]**2)
        min_r = min(start_r, end_r)
        
        # Only apply angular velocity limit if there's significant XY movement
        xy_move_d = math.sqrt(move.axes_d[0]**2 + move.axes_d[1]**2)
        if xy_move_d > 1e-6:
            # If this move starts or ends at the origin, there is no angle change
            # Moves that cross the origin would have been split earlier in the
            # code path:
            if start_r < 1e-6 or end_r < 1e-6:
                # If the move starts or ends at the origin, we cannot apply angular velocity limit
                return

            # Calculate the angular change for this move
            angle_start = math.atan2(move.start_pos[1], move.start_pos[0])
            angle_end = math.atan2(end_pos[1], end_pos[0])
            angle_diff = angle_end - angle_start
            
            # Normalize angle difference to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # If there's significant angular change, apply velocity limit
            if abs(angle_diff) > 1e-6 and min_r > 1e-6:
                # Maximum linear velocity based on angular velocity limit
                # v_linear = r * omega_max, so v_max = min_r * max_r_velocity
                max_linear_velocity = min_r * self.max_r_velocity
                max_linear_accel = min_r * self.max_r_accel
                
                # Apply the limit if it's more restrictive than current limits
                logging.info("Moving from (%f, %f) to (%f, %f) with angle change %f radians",
                             move.start_pos[0], move.start_pos[1],
                             end_pos[0], end_pos[1], angle_diff)
                logging.info("Adjusting velocity and acceleration from: %f, %f to: %f, %f",
                             move.max_velocity, move.max_accel,
                             max_linear_velocity, max_linear_accel)

                move.limit_speed(max_linear_velocity, max_linear_accel)


    def rotate_bed(self, angle):
        # Use force_move to actually command the bed stepper to the new angle
        stepper_bed = self.steppers[0]
        
        # Calculate the angle difference (shortest path)
        current_angle = stepper_bed.get_commanded_position()
        angle_diff = angle - current_angle
        
        # Normalize to shortest rotation
        if angle_diff > 3.14159:
            angle_diff -= 2 * 3.14159
        elif angle_diff < -3.14159:
            angle_diff += 2 * 3.14159
        
        # Use force_move to actually move the stepper
        if abs(angle_diff) > 1e-6:  # Only move if there's a significant difference
            force_move = self.printer.lookup_object('force_move')
            force_move.manual_move(stepper_bed, angle_diff, self.max_r_velocity, self.max_r_accel)  # 1 rad/s, 1 rad/s^2

            # Manually update the stepper's commanded position to the target angle
            # This is needed because when set_position is called with (0,0), 
            # the polar angle calc function returns the current angle to avoid atan2(0,0)
            # creating a circular dependency where the angle never gets updated
            import chelper
            ffi_main, ffi_lib = chelper.get_ffi()
            sk = stepper_bed.get_stepper_kinematics()
            # Use a temporary move structure to set the commanded position
            # We'll create a fake coordinate that would result in the target angle
            target_x = math.cos(angle) * 1.0  # Use radius of 1.0 
            target_y = math.sin(angle) * 1.0
            ffi_lib.itersolve_set_position(sk, target_x, target_y, 0.0)


    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return PolarKinematics(toolhead, config)
