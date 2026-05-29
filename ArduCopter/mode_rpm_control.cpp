#include "Copter.h"
#include <AP_BLHeli/AP_BLHeli.h>

#if MODE_RPM_CONTROL_ENABLED == ENABLED

/*
 * Init and run calls for RPM Control flight mode
 */

const AP_Param::GroupInfo ModeRpmControl::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: RPM Control Enable
    // @Description: Enable RPM control mode. When enabled, the mode will maintain target RPM using bdshot telemetry feedback
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, ModeRpmControl, rpm_enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: _TARGET
    // @DisplayName: Target RPM
    // @Description: Target RPM for motor speed control. The controller will attempt to maintain this RPM on all motors
    // @Range: 0 100000
    // @Units: rpm
    // @User: Standard
    AP_GROUPINFO("_TARGET", 2, ModeRpmControl, rpm_target, 5000),

    // @Param: _KP
    // @DisplayName: RPM Control Proportional Gain
    // @Description: Proportional gain for RPM controller. Higher values provide faster response but may cause oscillation
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("_KP", 3, ModeRpmControl, rpm_kp, 0.1f),

    // @Param: _KI
    // @DisplayName: RPM Control Integral Gain
    // @Description: Integral gain for RPM controller. Helps eliminate steady-state error
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("_KI", 4, ModeRpmControl, rpm_ki, 0.05f),

    // @Param: _KD
    // @DisplayName: RPM Control Derivative Gain
    // @Description: Derivative gain for RPM controller. Helps reduce overshoot
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("_KD", 5, ModeRpmControl, rpm_kd, 0.0f),

    // @Param: _KI_MAX
    // @DisplayName: RPM Control Integral Limit
    // @Description: Maximum value for the integral term to prevent windup
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("_KI_MAX", 6, ModeRpmControl, rpm_ki_max, 0.3f),

    // @Param: _FILT_HZ
    // @DisplayName: RPM Filter Frequency
    // @Description: Low-pass filter frequency for measured RPM (Hz). Higher values pass more noise, lower values add more lag
    // @Range: 0.1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 7, ModeRpmControl, rpm_filter_freq, 10.0f),

    // @Param: _CORR_MAX
    // @DisplayName: Maximum RPM Correction
    // @Description: Maximum throttle correction magnitude as a fraction of total throttle (0.0 to 1.0)
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("_CORR_MAX", 8, ModeRpmControl, rpm_corr_max, 0.2f),

    // @Param: _TELEM_TOUT
    // @DisplayName: RPM Telemetry Timeout
    // @Description: Time in milliseconds without RPM telemetry before disabling control (safety feature)
    // @Range: 0 10000
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("_TELEM_TOUT", 9, ModeRpmControl, rpm_telemetry_timeout_ms, 1000),

    // @Param: _DEADBAND
    // @DisplayName: RPM Error Deadband
    // @Description: RPM error magnitude below which control is not applied (reduces chatter from measurement noise)
    // @Range: 0 5000
    // @Units: rpm
    // @User: Standard
    AP_GROUPINFO("_DEADBAND", 10, ModeRpmControl, rpm_deadband, 100),

    AP_GROUPEND
};

ModeRpmControl::ModeRpmControl(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeRpmControl::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "RPM Control Mode: Initializing");
    
    // Reset controller state
    rpm_integrator = 0.0f;
    rpm_last_error = 0.0f;
    rpm_last_update_ms = AP_HAL::millis();
    rpm_measured_filtered = 0.0f;
    rpm_telemetry_last_ms = AP_HAL::millis();
    rpm_last_p_term = 0.0f;
    rpm_last_i_term = 0.0f;
    rpm_last_d_term = 0.0f;
    rpm_error = 0.0f;
    rpm_error_scaled = 0.0f;
    rpm_throttle_correction = 0.0f;
    
    // Initialize output throttle to the current vehicle throttle output for bumpless transfer
    rpm_throttle_output = motors->get_throttle_out();
    // if (rpm_throttle_output < 0.0f) {
    //     rpm_throttle_output = 0.0f;
    // }
    
    return true;
}

void ModeRpmControl::exit()
{
    gcs().send_text(MAV_SEVERITY_INFO, "RPM Control Mode: Exiting");
}

void ModeRpmControl::run()
{
    // if (!rpm_enabled) {
    //     static uint32_t last_disabled_warning_ms = 0;
    //     uint32_t now_ms = AP_HAL::millis();
    //     if (now_ms - last_disabled_warning_ms > 5000) {
    //         gcs().send_text(MAV_SEVERITY_WARNING, "RPM Control: Mode disabled via parameter");
    //         last_disabled_warning_ms = now_ms;
    //     }
    //     zero_throttle_and_relax_ac();
    //     return;
    // }

    // Handle arming/disarming
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        rpm_integrator = 0.0f;
        rpm_measured_filtered = 0.0f;
        // Keep refreshing the telemetry watchdog timer while disarmed
        rpm_telemetry_last_ms = AP_HAL::millis();
    } else if (!copter.throw_enable_throttle) {
        // keep motors at ground idle until throttle unlimited is enabled
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        rpm_telemetry_last_ms = AP_HAL::millis(); // Keep refreshing
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        rpm_telemetry_last_ms = AP_HAL::millis(); // Keep refreshing
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Clear landing flag so land detector doesn't trigger flow_of_control internal error
        set_land_complete(false);
        // Apply RPM control
        apply_rpm_control();
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // Check if RPM telemetry is available
    if (check_rpm_telemetry_timeout()) {
        // gcs().send_text(MAV_SEVERITY_INFO, "LE WAY - RPM Control: Telemetry timeout - disabling control");
        static uint32_t last_timeout_warning_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_timeout_warning_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RPM Control: Telemetry timeout - disabling control");
            last_timeout_warning_ms = now_ms;
        }
        zero_throttle_and_relax_ac();
        return;
    }

    // Log data for tuning
    log_rpm_data();
}

float ModeRpmControl::get_rpm_from_telemetry(uint8_t motor_index)
{
    uint32_t motor_mask = motors->get_motor_mask();
    uint8_t active_motor_count = 0;
    float rpm_sum = 0.0f;
    uint32_t now_ms = AP_HAL::millis();

    uint8_t motor_poles = 14; // default to 14 as in AP_BLHeli
#if defined(HAVE_AP_BLHELI_SUPPORT)
    AP_BLHeli *blh = AP_BLHeli::get_singleton();
    if (blh) {
        motor_poles = blh->get_motor_poles();
    }
#endif
    if (motor_poles == 0) {
        motor_poles = 14; // safe guard against division by zero
    }

    bool telemetry_alive = false;

    for (uint8_t i = 0; i < 16; i++) {
        if (motor_mask & (1U << i)) {
            uint16_t erpm = hal.rcout->get_erpm(i);
            float error_rate = hal.rcout->get_erpm_error_rate(i);
            
            // Telemetry link is alive if we are successfully receiving packets
            if (error_rate < 100.0f) {
                telemetry_alive = true;
            }

            // erpm must not be 0xFFFF (0 is a valid reading when stopped)
            if (erpm != 0xFFFF) {
                float rpm = (float)erpm * 200.0f / (float)motor_poles;
                rpm_sum += rpm;
                active_motor_count++;
            }
        }
    }

    if (telemetry_alive) {
        rpm_telemetry_last_ms = now_ms;
    }

    if (active_motor_count > 0) {
        return rpm_sum / active_motor_count;
    }

    return 0.0f;
}

float ModeRpmControl::update_rpm_filter(float new_rpm, float current_filtered)
{
    // Simple first-order low-pass filter
    // Using exponential moving average: filtered = alpha * new_rpm + (1 - alpha) * current_filtered
    
    if (rpm_filter_freq <= 0.0f) {
        // No filtering if frequency is 0 or negative
        return new_rpm;
    }

    // Calculate filter time constant and alpha
    float dt = (AP_HAL::millis() - rpm_last_update_ms) * 0.001f; // seconds
    if (dt > 1.0f) dt = 1.0f; // Clamp dt to prevent large jumps
    
    float time_constant = 1.0f / (2.0f * M_PI * rpm_filter_freq);
    float alpha = dt / (time_constant + dt);
    
    // Apply low-pass filter
    float filtered = alpha * new_rpm + (1.0f - alpha) * current_filtered;
    
    return filtered;
}

float ModeRpmControl::compute_pid_correction()
{
    // Read average measured RPM from motor telemetry
    float rpm_measured = get_rpm_from_telemetry(0);
    
    // Update filter
    rpm_measured_filtered = update_rpm_filter(rpm_measured, rpm_measured_filtered);
    
    // Calculate error
    rpm_error = rpm_target - rpm_measured_filtered;

    // Freeze integral accumulation if within deadband
    bool inside_deadband = fabsf(rpm_error) < rpm_deadband;

    // Scale the error by a factor that makes sense for the PID loop. 
    // We want the PID correction to be in the same range as the throttle output.
    rpm_error_scaled = rpm_error * 0.001f;
    
    // Calculate time since last update
    uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - rpm_last_update_ms) * 0.001f;
    rpm_last_update_ms = now_ms;
    
    if (dt > 1.0f) dt = 1.0f; // Clamp dt
    if (dt < 0.001f) dt = 0.001f; // Minimum dt to avoid division issues
    
    // Proportional term
    rpm_last_p_term = inside_deadband ? 0.0f : (rpm_kp * rpm_error_scaled);
    
    // Integral term with anti-windup
    if (!inside_deadband) {
        rpm_integrator += rpm_error_scaled * dt;
        rpm_integrator = constrain_float(rpm_integrator, -rpm_ki_max, rpm_ki_max);
    }
    rpm_last_i_term = rpm_ki * rpm_integrator;
    
    // Derivative term
    rpm_last_d_term = inside_deadband ? 0.0f : (rpm_kd * (rpm_error_scaled - rpm_last_error) / dt);
    rpm_last_error = rpm_error_scaled;
    
    // Compute total correction
    float correction = (rpm_last_p_term + rpm_last_i_term + rpm_last_d_term);
    
    // Clamp correction
    correction = constrain_float(correction, -rpm_corr_max, rpm_corr_max);
    
    rpm_throttle_correction = correction;
    
    return correction;
}

bool ModeRpmControl::check_rpm_telemetry_timeout()
{
    uint32_t now_ms = AP_HAL::millis();
    
    if ((now_ms - rpm_telemetry_last_ms) > (uint32_t)rpm_telemetry_timeout_ms) {
        return true; // Timeout occurred
    }
    
    return false;
}

void ModeRpmControl::apply_rpm_control()
{
    // Compute PID correction based on RPM error
    float throttle_correction = compute_pid_correction();
    
    // Integrate the correction directly into the actual collective throttle state
    rpm_throttle_output += throttle_correction;
    
    // Clamp output throttle to valid range
    rpm_throttle_output = constrain_float(rpm_throttle_output, 0.0f, 1.0f);
    
    // Pilot attitude inputs (for roll, pitch, and yaw tracking)
    // update_simple_mode();

    // float target_roll, target_pitch;
    // get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // Set motors to full range
    // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Call attitude controller for stabilization
    // attitude_control->input_euler_angle_roll_pitch_bf_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    attitude_control->input_euler_angle_roll_pitch_bf_rate_yaw(0.0f, 0.0f, 0.0f);
    
    // Set collective throttle output
    attitude_control->set_throttle_out(rpm_throttle_output, false, g.throttle_filt);
}

void ModeRpmControl::log_rpm_data()
{
#if HAL_LOGGING_ENABLED
    static uint32_t last_log_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    
    // Log every 20ms (50Hz) to capture high-rate PID dynamics for tuning
    if (now_ms - last_log_ms >= 20) {
        last_log_ms = now_ms;
        
        copter.Log_Write_RPMF(
            rpm_target,
            rpm_measured_filtered,
            rpm_error,
            rpm_last_p_term,
            rpm_last_i_term,
            rpm_last_d_term,
            rpm_error_scaled,
            rpm_throttle_correction,
            rpm_throttle_output
        );
    }
#endif
}

#endif // MODE_RPM_CONTROL_ENABLED
