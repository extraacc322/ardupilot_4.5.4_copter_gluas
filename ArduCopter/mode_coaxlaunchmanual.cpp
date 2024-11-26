#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */
// throw_init - initialise throw controller
bool ModeCoaxLaunchManual::init(bool ignore_checks)
{
    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }
    gcs().send_text(MAV_SEVERITY_INFO,"LAUNCH NOT DETECTED INIT");

    return true;
}


// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeCoaxLaunchManual::run()
{
    // apply simple mode transform to pilot inputs
    // gcs().send_text(MAV_SEVERITY_INFO,"LAUNCH NOT DETECTED - RUN");

    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {  // if vehicle is not armed
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        // reset launch parameters: set climb_rate_after_arm_indicator to zero, set time of launch to -1
        climb_rate_after_arm_indicator = 0;
        time_of_launch = -1;
        launch_over_msg_sent = false;
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        launch_detected();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        
        
        // if (climb_rate_after_arm_indicator == 0 and arm_indicator){
        //     gcs().send_text(MAV_SEVERITY_INFO,"LAUNCH NOT DETECTED - RUN climb cond");
        // }
        launch_detected();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}

void ModeCoaxLaunchManual::launch_detected(){
    /*
        check climb rate
            if the climb rate goes below 100, then mark that the launch has begun: set launch_detected to 1
            mark the time at which the launch began
            get the shutdown_spoolstate_tracker
            if the motors were spun for the first time after arming (shutdown_spoolstate_tracker==1)
                get the time they were spun (t_first)
                check if 0.5s has passed ever since they were spun
                    if so, set launch_detected to zero
            
        */ 
    climb_rate_launch_mode = int16_t(inertial_nav.get_velocity_z_up_cms());
    // gcs().send_text(MAV_SEVERITY_INFO,"climb_rate: %d, %d", (int16_t) int16_t(inertial_nav.get_velocity_z_up_cms()), (int16_t) climb_rate_launch_mode);
    
    // CHECK IF VEHICLE HAS BEEN LAUNCHED
    if (abs(climb_rate_launch_mode) >= 100){ // 100 m/s is based on baro data from tests which have shown that at launch, the climb rate is usually -200 to -600 m/s. This is obviously false, but since it happens consistently, I can use it to record when the launch begins.
        climb_rate_after_arm_indicator = climb_rate_after_arm_indicator + 1;
        // gcs().send_text(MAV_SEVERITY_INFO,"climb_rate more than 100");
        
        // VEHICLE HAS BEEN LAUNCHED
        if (climb_rate_after_arm_indicator == 1){
            time_of_launch = AP_HAL::millis();
            motors->set_launch_detected(1);
            // vehicle_launched = true;
            gcs().send_text(MAV_SEVERITY_INFO,"LAUNCH DETECTED");
        }

        // cap climb_rate_after_arm_indicator at 100 and reset to 2
        if (climb_rate_after_arm_indicator >= g2.launch_initial_baro_climb_rate){
            climb_rate_after_arm_indicator = 2;
        }
    }

    /*CHECK IF LAUNCH PHASE IS OVER
    i.e., time_for_imu_to_recover=1.5 sec have passed ever since motors were first spun after arming */ 
    uint8_t shutdown_spoolstate_tracker_ = motors->get_shutdown_spoolstate_tracker();
    uint32_t t_first_ = motors->get_t_first();
    if (shutdown_spoolstate_tracker_ == 1){ // first time motors are spun after arming
        t_first_ = motors->get_t_first();
    }
    if (climb_rate_after_arm_indicator != 0){
        if (t_first_ != -1){
            if (((AP_HAL::millis() - t_first_) >= g2.time_for_imu_to_recover_after_launch) && (!launch_over_msg_sent)){
                motors->set_launch_detected(0);
                gcs().send_text(MAV_SEVERITY_INFO,"LAUNCH OVER");
                gcs().send_text(MAV_SEVERITY_INFO,"t_first, t_now: %ld, %ld", t_first_, AP_HAL::millis());
                launch_over_msg_sent = true;
            }
        }
    }
}
