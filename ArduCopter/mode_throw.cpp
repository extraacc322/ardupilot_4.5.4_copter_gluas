#include "Copter.h"

#if MODE_THROW_ENABLED == ENABLED

// throw_init - initialise throw controller
bool ModeThrow::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // init state
    stage = Throw_Disarmed;
    nextmode_attempted = false;
    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void ModeThrow::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    if (!motors->armed()) {
        // state machine entry is always from a disarmed state
        stage = Throw_Disarmed;
        motors->set_launch_detected(0);
        time_since_launch = 0;
        launch_time = 0;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO,"armed - waiting for throw");
        if (motors->get_launch_detected() == 0) {
            time_of_arm = AP_HAL::millis();
            motors->set_launch_detected(2);
        }
        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        launch_time = AP_HAL::millis();
        copter.set_land_complete(false);
        motors->set_launch_detected(1);
        stage = Throw_Wait_Throttle_Unlimited;
        gcs().send_text(MAV_SEVERITY_INFO,"throw detected - waiting for throttle trigger");
        // copter.set_land_complete(false);

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        time_since_launch = AP_HAL::millis() - launch_time; // this is recorded about ~50ms after throttle was actually enabled (artifact of state machine processing time)
        gcs().send_text(MAV_SEVERITY_INFO,"throttle enabled after %.2d ms", int(time_since_launch));
        stage = Throw_Uprighting;
    }

    // Throw State Processing
    switch (stage) {
    case Throw_Disarmed:
        // prevent motors from rotating before the throw is detected unless enabled by the user
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        break;

    case Throw_Detecting:
        // prevent motors from rotating before the throw is detected
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;
        break;

    case Throw_Wait_Throttle_Unlimited:
        // set motors to full range only when throttle unlimited is enabled
        if (copter.throw_enable_throttle) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        } else {
            // keep motors at ground idle until throttle unlimited is enabled
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        }
        break;

    case Throw_Uprighting:
        // set motors to full range
        // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // output a greater than hover throttle with no angle boost
        attitude_control->set_throttle_out(g.throw_mode_throttle, false, g.throttle_filt);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_bf_rate_yaw(0.0f, 0.0f, 0.0f);
        break;
    }

#if HAL_LOGGING_ENABLED
    // log at 20hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 200) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = inertial_nav.get_velocity_neu_cms().length();
        const float velocity_z = inertial_nav.get_velocity_z_up_cms();
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const uint32_t throw_detect = launch_time;
        const uint32_t attitude_ok = time_since_launch;


// @LoggerMessage: THRO
// @Description: Throw Mode messages
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Throw Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Throw: Time at which throw was detected
// @Field: AttOk: Time at which throttle was enabled after throw

        AP::logger().WriteStreaming(
            "THRO",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk",
            "s-nnooss",
            "F-0000CC",
            "QBffffII",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity,
            (double)velocity_z,
            (double)accel,
            (double)ef_accel_z,
            throw_detect,
            attitude_ok);
    }
#endif  // HAL_LOGGING_ENABLED
}

bool ModeThrow::throw_detected()
{
    // For testing, we can simulate a throw after 10 seconds
    // return (AP_HAL::millis() - time_of_arm > 10000);

    // Check for a sufficient acceleration magnitude to indicate a launch
    return copter.ins.get_accel().length() >= g.throw_launch_g_threshold * GRAVITY_MSS;
}

bool ModeThrow::throw_attitude_good() const
{
    // Check that we have uprighted the copter
    // const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return false;
}

bool ModeThrow::throw_height_good() const
{
    // Check that we are within 0.5m of the demanded height
    return true;
}

bool ModeThrow::throw_position_good() const
{
    // check that our horizontal position error is within 50cm
    return true; //(pos_control->get_pos_error_xy_cm() < 50.0f);
}

#endif
