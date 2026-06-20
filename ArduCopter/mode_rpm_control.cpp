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

    // @Param: _TGT_UPP
    // @DisplayName: Target RPM Upper Rotor
    // @Description: Target RPM for upper (CCW) motor speed control
    // @Range: 0 100000
    // @Units: rpm
    // @User: Standard
    AP_GROUPINFO("_TGT_UPP", 2, ModeRpmControl, rpm_target_upper, 5000),

    // @Param: _KP
    // @DisplayName: RPM Control Proportional Gain
    // @Description: Proportional gain for RPM controller. Higher values provide faster response but may cause oscillation
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("_KP", 3, ModeRpmControl, rpm_kp, 0.0004f),

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
    AP_GROUPINFO("_DEADBAND", 10, ModeRpmControl, rpm_deadband, 10),

    // @Param: _TGT_LOW
    // @DisplayName: Target RPM Lower Rotor
    // @Description: Target RPM for lower (CW) motor speed control
    // @Range: 0 100000
    // @Units: rpm
    // @User: Standard
    AP_GROUPINFO("_TGT_LOW", 11, ModeRpmControl, rpm_target_lower, 5000),

    AP_GROUPEND
};

ModeRpmControl::ModeRpmControl(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeRpmControl::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "RPM Control Mode: Initializing");
    
    // Reset controller states
    rpm_integrator_upper = 0.0f;
    rpm_integrator_lower = 0.0f;
    rpm_last_error_upper = 0.0f;
    rpm_last_error_lower = 0.0f;
    rpm_measured_filtered_upper = 0.0f;
    rpm_measured_filtered_lower = 0.0f;
    rpm_throttle_output_upper = 0.0f;
    rpm_throttle_output_lower = 0.0f;

    rpm_last_update_ms = AP_HAL::millis();
    rpm_telemetry_last_ms = AP_HAL::millis();

    rpm_last_p_term_upper = 0.0f;
    rpm_last_i_term_upper = 0.0f;
    rpm_last_d_term_upper = 0.0f;
    rpm_last_p_term_lower = 0.0f;
    rpm_last_i_term_lower = 0.0f;
    rpm_last_d_term_lower = 0.0f;

    return true;
}

void ModeRpmControl::exit()
{
    gcs().send_text(MAV_SEVERITY_INFO, "RPM Control Mode: Exiting");
}

void ModeRpmControl::run()
{
    // Handle arming/disarming
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        rpm_integrator_upper = 0.0f;
        rpm_integrator_lower = 0.0f;
        rpm_measured_filtered_upper = 0.0f;
        rpm_measured_filtered_lower = 0.0f;
        rpm_throttle_output_upper = 0.0f;
        rpm_throttle_output_lower = 0.0f;
        // Keep refreshing the telemetry watchdog timer while disarmed
        rpm_telemetry_last_ms = AP_HAL::millis();
    } else if (!copter.throw_enable_throttle) {
        // keep motors at ground idle until throttle unlimited is enabled
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        static uint32_t last_idle_msg_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_idle_msg_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_INFO, "RPM Control: Waiting for throttle to be enabled");
            last_idle_msg_ms = now_ms;
        }
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        // static uint32_t last_active_msg_ms = 0;
        // uint32_t now_ms = AP_HAL::millis();
        // if (now_ms - last_active_msg_ms > 5000) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "RPM Control: Throttle enabled, starting control");
        //     last_active_msg_ms = now_ms;
        // }
    }
    
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        rpm_telemetry_last_ms = AP_HAL::millis(); // Keep refreshing
        rpm_integrator_upper = 0.0f;
        rpm_integrator_lower = 0.0f;
        rpm_throttle_output_upper = 0.0f;
        rpm_throttle_output_lower = 0.0f;
        // zero_throttle_and_relax_ac();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        rpm_telemetry_last_ms = AP_HAL::millis(); // Keep refreshing
        rpm_integrator_upper = 0.0f;
        rpm_integrator_lower = 0.0f;
        rpm_throttle_output_upper = 0.0f;
        rpm_throttle_output_lower = 0.0f;
        // zero_throttle_and_relax_ac();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Clear landing flag so land detector doesn't trigger flow_of_control internal error
        set_land_complete(false);
        // Apply RPM control
        apply_rpm_control();
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // Check if RPM telemetry is available
    if (check_rpm_telemetry_timeout()) {
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

float ModeRpmControl::get_rpm_from_telemetry(uint8_t motor_channel)
{
    uint32_t motor_mask = motors->get_motor_mask();
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

    if (motor_mask & (1U << motor_channel)) {
        uint16_t erpm = hal.rcout->get_erpm(motor_channel);
        float error_rate = hal.rcout->get_erpm_error_rate(motor_channel);
        
        // Telemetry link is alive if we are successfully receiving packets
        if (error_rate < 100.0f) {
            telemetry_alive = true;
        }

        // erpm must not be 0xFFFF (0 is a valid reading when stopped)
        if (erpm != 0xFFFF) {
            float rpm = (float)erpm * 200.0f / (float)motor_poles;
            if (telemetry_alive) {
                rpm_telemetry_last_ms = now_ms;
            }
            return rpm;
        }
    }

    return 0.0f;
}

float ModeRpmControl::compute_pid_correction(float target_rpm, float rpm_measured, float &measured_filtered, float &integrator, float &last_error, float &last_p_term, float &last_i_term, float &last_d_term)
{
    uint32_t now_ms = AP_HAL::millis();
    
    // Update filter
    measured_filtered = update_rpm_filter(rpm_measured, measured_filtered);
    
    // Calculate error
    float rpm_error = target_rpm - measured_filtered;

    // Freeze integral accumulation if within deadband
    bool inside_deadband = fabsf(rpm_error) < rpm_deadband;

    // Scale the error by a factor that makes sense for the PID loop. 
    // We want the PID correction to be in the same range as the throttle output.
    float error_scaled = rpm_error * 0.001f;
    
    // Calculate time since last update
    float dt = (now_ms - rpm_last_update_ms) * 0.001f;
    if (dt > 1.0f) dt = 1.0f; // Clamp dt
    if (dt < 0.001f) dt = 0.001f; // Minimum dt to avoid division issues
    
    // Proportional term
    last_p_term = inside_deadband ? 0.0f : (rpm_kp * error_scaled);
    
    // Integral term with anti-windup
    if (!inside_deadband) {
        integrator += error_scaled * dt;
        integrator = constrain_float(integrator, -rpm_ki_max, rpm_ki_max);
    }
    last_i_term = rpm_ki * integrator;
    
    // Derivative term
    last_d_term = inside_deadband ? 0.0f : (rpm_kd * (error_scaled - last_error) / dt);
    last_error = error_scaled;
    
    // Compute total correction
    float correction = (last_p_term + last_i_term + last_d_term);
    
    // Clamp correction
    correction = constrain_float(correction, -rpm_corr_max, rpm_corr_max);
    
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
    // Fetch individual rotor telemetry values and multiply by 1.16f to 
    // convert esc measured rpm into actual rotor rpm. 1.16f was obtained 
    // by comparing esc measured rpm with actual rotor rpm using optical tachometer.
    float rpm_measured_upper = get_rpm_from_telemetry(0)*1.16f;
    float rpm_measured_lower = get_rpm_from_telemetry(3)*1.16f;

    // Compute PID correction for upper CCW motor (channel 0)
    float correction_upper = compute_pid_correction(
        rpm_target_upper,
        rpm_measured_upper,
        rpm_measured_filtered_upper,
        rpm_integrator_upper,
        rpm_last_error_upper,
        rpm_last_p_term_upper,
        rpm_last_i_term_upper,
        rpm_last_d_term_upper
    );

    // Compute PID correction for lower CW motor (channel 3)
    float correction_lower = compute_pid_correction(
        rpm_target_lower,
        rpm_measured_lower,
        rpm_measured_filtered_lower,
        rpm_integrator_lower,
        rpm_last_error_lower,
        rpm_last_p_term_lower,
        rpm_last_i_term_lower,
        rpm_last_d_term_lower
    );

    // Update last update time after running both controllers
    rpm_last_update_ms = AP_HAL::millis();

    // Integrate the corrections directly into the collective throttle states
    rpm_throttle_output_upper += correction_upper;
    rpm_throttle_output_lower += correction_lower;

    // Clamp output throttles to valid range
    rpm_throttle_output_upper = constrain_float(rpm_throttle_output_upper, 0.0f, 1.0f);
    rpm_throttle_output_lower = constrain_float(rpm_throttle_output_lower, 0.0f, 1.0f);

    // Find CX_YW_TRM parameter dynamically to access yaw_trim from AP_MotorsCoax
    ap_var_type vtype;
    AP_Float* cx_yw_trm_param = (AP_Float*)AP_Param::find("CX_YW_TRM", &vtype);
    float yaw_trim = (cx_yw_trm_param != nullptr) ? cx_yw_trm_param->get() : 1.0f;

    // Map upper/lower throttle targets to throttle average and differential yaw
    float throttle_avg = (rpm_throttle_output_upper + rpm_throttle_output_lower) / (1.0f + yaw_trim);
    float throttle_diff = 2.0f * (rpm_throttle_output_upper - throttle_avg);

    // Clamp throttle_avg to valid range
    throttle_avg = constrain_float(throttle_avg, 0.0f, 1.0f);

    // Call attitude controller for stabilization
    attitude_control->input_euler_angle_roll_pitch_bf_rate_yaw(0.0f, 0.0f, 0.0f);
    
    // Set collective throttle output
    attitude_control->set_throttle_out(throttle_avg, false, g.throttle_filt);

    // Command differential thrust using set_yaw on the motors library
    motors->set_yaw(throttle_diff);
}

void ModeRpmControl::log_rpm_data()
{
#if HAL_LOGGING_ENABLED
    static uint32_t last_log_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    
    // Log every 20ms (50Hz) to capture high-rate PID dynamics for tuning
    if (now_ms - last_log_ms >= 20) {
        last_log_ms = now_ms;
        
        // Log upper rotor (ID 0)
        copter.Log_Write_RPMF(
            0,
            rpm_target_upper,
            rpm_measured_filtered_upper,
            (rpm_target_upper - rpm_measured_filtered_upper),
            rpm_last_p_term_upper,
            rpm_last_i_term_upper,
            rpm_last_d_term_upper,
            (rpm_target_upper - rpm_measured_filtered_upper) * 0.001f,
            rpm_last_p_term_upper + rpm_last_i_term_upper + rpm_last_d_term_upper,
            rpm_throttle_output_upper
        );

        // Log lower rotor (ID 3)
        copter.Log_Write_RPMF(
            3,
            rpm_target_lower,
            rpm_measured_filtered_lower,
            (rpm_target_lower - rpm_measured_filtered_lower),
            rpm_last_p_term_lower,
            rpm_last_i_term_lower,
            rpm_last_d_term_lower,
            (rpm_target_lower - rpm_measured_filtered_lower) * 0.001f,
            rpm_last_p_term_lower + rpm_last_i_term_lower + rpm_last_d_term_lower,
            rpm_throttle_output_lower
        );
    }
}
#endif

void ModeRpmControl::output_to_motors()
{
    // Execute standard output first (keeps flap servos and spool states updated)
    Mode::output_to_motors();

    // If in active control, overwrite the motor channels with raw, unconstrained throttle
    if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        uint16_t pwm_min = motors->get_pwm_output_min();
        uint16_t pwm_max = motors->get_pwm_output_max();
        
        uint16_t pwm_upper = pwm_min + (uint16_t)(rpm_throttle_output_upper * (pwm_max - pwm_min));
        uint16_t pwm_lower = pwm_min + (uint16_t)(rpm_throttle_output_lower * (pwm_max - pwm_min));

        motors->rc_write(0, pwm_upper);
        motors->rc_write(3, pwm_lower);
    }
}

#endif // MODE_RPM_CONTROL_ENABLED
