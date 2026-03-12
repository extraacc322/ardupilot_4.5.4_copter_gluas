/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// first commit
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsCoax.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsCoax::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // make sure 6 output channels are mapped
    for (uint8_t i = 0; i < 6; i++) {
        add_motor_num(CH_1 + i);
    }

    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    // setup actuator scaling
    for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) {
        if (i == 0 || i == 3) continue;
        SRV_Channels::set_angle(SRV_Channels::get_motor_function(i), AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    }

    _mav_type = MAV_TYPE_COAXIAL;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_COAX);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsCoax::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok(frame_class == MOTOR_FRAME_COAX);
}

// set update rate to motors - a value in hertz
void AP_MotorsCoax::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    uint32_t mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_4 ;
    rc_set_freq(mask, _speed_hz);
}

bool AP_MotorsCoax::check_coaxrotorstartup_timer_condition(){
    return (((int)t_first != -1) && (((uint32_t)AP_HAL::millis() - t_first) >= (uint32_t)_time_betw_rotor_startups));
}

void AP_MotorsCoax::output_to_motors()
{
    // Check arming status and set shutdown_spoolstate_tracker and t_first appropriately
    if (armed()){
        if (check_coaxrotorstartup_timer_condition()){
            shutdown_spoolstate_tracker = shutdown_spoolstate_tracker + 1;
            // cap shutdown_spoolstate_tracker at 100 and reset to 2
            if (shutdown_spoolstate_tracker >= 100){
                shutdown_spoolstate_tracker = 2;
            }
        }
    } else {
        // set shutdown_tracker to zero
        // gcs().send_text(MAV_SEVERITY_INFO,"armed, t_first, shutdown_spoolstate_tracker: %d, %d, %d: ",armed(),(int)t_first,(int)shutdown_spoolstate_tracker);
        shutdown_spoolstate_tracker = 0;
        t_first = -1;
    }

    if (shutdown_spoolstate_tracker == 0) { // either disarmed or armed, but rotors have not been spun up at least once (i.e., spool state has not gone beyond ground idle at least once)
        // gcs().send_text(MAV_SEVERITY_INFO,"SST: 0");
        switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            t_first = -1;
            for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) {
                if (i == 0 || i == 3) continue; 
                rc_write_angle(AP_MOTORS_MOT_1 + i, _keep_servo_trim * _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE); 
            }
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));
            break;
        case SpoolState::GROUND_IDLE: 
        // sends output to motors when armed but not flying
        // Vehicle enters into GROUND_IDLE state immediately after arming and before receiving a non-zero throttle command
            for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) {
                if (i == 0 || i == 3) continue;
                rc_write_angle(AP_MOTORS_MOT_1 + i, _keep_servo_trim * _spin_up_ratio * _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
            }
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
            if (check_coaxrotorstartup_timer_condition()){
                set_actuator_with_slew(_actuator[AP_MOTORS_MOT_4], actuator_spin_up_to_ground_idle());
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[AP_MOTORS_MOT_4]));
            } else {
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(0)); // send zero throttle to lower (cw) rotor
            }
            break;
        case SpoolState::SPOOLING_UP:
            t_first = AP_HAL::millis(); // record the first time the rotors are commanded a non-zero throttle after SHUT_DOWN spool_state
            // gcs().send_text(MAV_SEVERITY_INFO,"spooling up");
            // set motor output based on thrust requests
            for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) { 
                if (i == 0 || i == 3) continue;
                rc_write_angle(AP_MOTORS_MOT_1 + i, _keep_servo_trim * _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE); 
            }
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], thr_lin.thrust_to_actuator(_thrust_yt_ccw));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
            if (check_coaxrotorstartup_timer_condition()){
                set_actuator_with_slew(_actuator[AP_MOTORS_MOT_4], thr_lin.thrust_to_actuator(_thrust_yt_cw));
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[AP_MOTORS_MOT_4])); //
            } else {
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(0)); // send zero throttle to lower (cw) rotor
            }
            break;
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) { 
                if (i == 0 || i == 3) continue;
                rc_write_angle(AP_MOTORS_MOT_1 + i, _keep_servo_trim * _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE); 
            }
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], thr_lin.thrust_to_actuator(_thrust_yt_ccw));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
            if (check_coaxrotorstartup_timer_condition()){
                set_actuator_with_slew(_actuator[AP_MOTORS_MOT_4], thr_lin.thrust_to_actuator(_thrust_yt_cw));
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[AP_MOTORS_MOT_4])); // 
            } else {
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(0)); // send zero throttle to lower (cw) rotor
            }
            break;
        }

    } else if (shutdown_spoolstate_tracker >= 1) { // In flight
        if (shutdown_spoolstate_tracker == 1) { // first time a non-zero throttle is commanded after arming
            gcs().send_text(MAV_SEVERITY_INFO,"SST: 1");
        }
        switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) { 
                if (i == 0 || i == 3) continue;
                rc_write_angle(AP_MOTORS_MOT_1 + i, _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE); 
            }
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) {
                if (i == 0 || i == 3) continue;
                rc_write_angle(AP_MOTORS_MOT_1 + i, _spin_up_ratio * _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
            }
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_4], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[AP_MOTORS_MOT_4]));
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) {
                if (i == 0 || i == 3) continue;
                rc_write_angle(AP_MOTORS_MOT_1 + i, _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
            }
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], thr_lin.thrust_to_actuator(_thrust_yt_ccw));
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_4], thr_lin.thrust_to_actuator(_thrust_yt_cw));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[AP_MOTORS_MOT_4]));
            break;
        }
    }

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsCoax::get_motor_mask()
{
    uint32_t motor_mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_4;
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// sends commands to the motors
void AP_MotorsCoax::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0
    float   thrust_out;                 //

    // apply voltage and air pressure compensation
    // const float compensation_gain = thr_lin.get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff);
    pitch_thrust = (_pitch_in + _pitch_in_ff);
    yaw_thrust = (_yaw_in + _yaw_in_ff); 
    throttle_thrust = get_throttle();
    throttle_avg_max = _throttle_avg_max;
    
    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // calculate the throttle setting
    thrust_out = throttle_avg_max;
    
    // set the throttle out value
    _throttle_out = thrust_out;

    // limit yaw_thrust, if thrust commanded for yaw is greater than the thrust to be sent out
    if (fabsf(yaw_thrust) > thrust_out) {
        yaw_thrust = constrain_float(yaw_thrust, -thrust_out, thrust_out);
        limit.yaw = true;
    }
    
    // send thrust output to cw and ccw motors
    if (launch_detected != 0) {
        // Launch phase: asymmetric yaw control to minimize wake interactions
        // Keep one motor at base thrust, ramp the other based on yaw demand
        // This assumes vehicle spins CCW during launch, so yaw_thrust < 0 means CCW needs more thrust
        if (yaw_thrust < 0.0f) {
            // CW motor needs more thrust: keep CCW at base, increase CW
            _thrust_yt_ccw = thrust_out;
            _thrust_yt_cw = thrust_out - yaw_thrust; // yaw_thrust is negative, so this adds
        } else {
            // CCW motor needs more thrust: keep CW at base, increase CCW
            _thrust_yt_ccw = thrust_out + yaw_thrust;
            _thrust_yt_cw = thrust_out;
        }
    } else {
        // Normal flight: symmetric yaw mixing
        _thrust_yt_ccw = thrust_out + (0.5f * yaw_thrust);
        _thrust_yt_cw = (yaw_trim * thrust_out) - (0.5f * yaw_thrust);
    }

    // Independently constrain both motor outputs to yaw_thrust_limit limit
    if (_thrust_yt_ccw > yaw_thrust_limit) {
        _thrust_yt_ccw = yaw_thrust_limit;
    }
    if (_thrust_yt_cw > yaw_thrust_limit) {
        _thrust_yt_cw = yaw_thrust_limit;
    }

    // calculate the actuator outputs for roll and pitch
    _actuator_out[1] = roll_thrust * _scale_servo_output; 
    _actuator_out[2] = pitch_thrust * _scale_servo_output;
 
    // Apply per-axis attitude error feedforward scaling if enabled
    // Convert error thresholds from degrees to radians
    float min_err_rad = radians(_ff_min_err.get());
    float max_err_rad = radians(_ff_max_err.get());
    float min_scale = _ff_min_scale.get();
    float max_scale = _ff_max_scale.get();
    
    // Calculate linear scaling function: if error is between min and max, interpolate scaling
    auto calc_scale = [](float err, float min_err, float max_err, float min_sc, float max_sc) {
        if (err < min_err) return min_sc;
        if (err >= max_err) return max_sc;
        float progress = (err - min_err) / (max_err - min_err);
        return min_sc + progress * (max_sc - min_sc);
    };
    
    // Get absolute roll and pitch errors in radians
    float roll_err = fabsf(_attitude_error.x);
    float pitch_err = fabsf(_attitude_error.y);
    
    // Calculate per-axis scaling factors
    float roll_scale = calc_scale(roll_err, min_err_rad, max_err_rad, min_scale, max_scale);
    float pitch_scale = calc_scale(pitch_err, min_err_rad, max_err_rad, min_scale, max_scale);
    
    // Apply independent per-axis scaling
    _actuator_out[1] *= roll_scale;
    _actuator_out[2] *= pitch_scale;

    // limit roll and pitch commands if absolute value is greater than maximum which is 1
    if (fabsf(_actuator_out[1]) > 1.0f) {
        limit.roll = true;
        _actuator_out[1] = constrain_float(_actuator_out[1], -1.0f, 1.0f);
    }
    if (fabsf(_actuator_out[2]) > 1.0f) {
        limit.pitch = true;
        _actuator_out[2] = constrain_float(_actuator_out[2], -1.0f, 1.0f);
    }
    
    // Limit roll and pitch actuator outputs at the beginning of the launch
    if (launch_detected != 0){
        _actuator_out[1] = constrain_float(_actuator_out[1], -roll_actuator_limit, roll_actuator_limit);
        _actuator_out[2] = constrain_float(_actuator_out[2], -pitch_actuator_limit, pitch_actuator_limit);
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "a0, a1, yt: %d, %.2f, %.2f, %.2f", launch_detected, _actuator_out[0], _actuator_out[1], yaw_thrust);
    
    _actuator_out[4] = -_actuator_out[1];
    _actuator_out[5] = -_actuator_out[2];



}


// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsCoax::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // flap servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // flap servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // flap servo 4
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // motor 1
            rc_write(AP_MOTORS_MOT_5, pwm);
            break;
        case 6:
            // motor 2
            rc_write(AP_MOTORS_MOT_6, pwm);
            break;
        default:
            // do nothing
            break;
    }
}


