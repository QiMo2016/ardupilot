#include "Copter.h"
// cruise_init - initialise cruise controller
bool Copter::cruise_init(bool ignore_checks)
{
	if (position_ok() || ignore_checks) {
	        // initialise yaw
	        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
	        // start in position control mode
	        guided_pos_control_start();
	        return true;
	    }else{
	        return false;
	    }
}
// cruise_run - runs the cruise controller
// should be called at 100hz or more
void Copter::cruise_run() {
	// call the correct auto controller
	switch (cruise_mode) {

	case Cruise_TakeOff:
		// run take-off controller
		cruise_takeoff_run();
		break;

    case Cruise_WP:
        // run position controller
        cruise_pos_control_run();
        break;
	}
}

// cruise_takeoff_start - initialises waypoint controller to implement take-off
bool Copter::cruise_takeoff_start(float final_alt_above_home)
{
	cruise_mode = Cruise_TakeOff;

    // initialise wpnav destination
    Location_Class target_loc = current_loc;
    target_loc.set_alt_cm(final_alt_above_home, Location_Class::ALT_FRAME_ABOVE_HOME);

    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();

    return true;
}

//
//
void Copter::cruise_takeoff_run()
{
	// if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
	if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
		// initialise wpnav targets
		wp_nav->shift_wp_origin_to_current_pos();
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // reset attitude control targets
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
		// clear i term when we're taking off
		set_throttle_takeoff();
		return;
	}

	// process pilot's yaw input
	float target_yaw_rate = 0;
	if (!failsafe.radio) {
		// get pilot's desired yaw rate
		target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
	}

#if FRAME_CONFIG == HELI_FRAME
    // helicopters stay in landed state until rotor speed runup has finished
    if (motors->rotor_runup_complete()) {
        set_land_complete(false);
    } else {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
    }
#else
    set_land_complete(false);
#endif

	// set motors to full range
	motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

	// run waypoint controller
	failsafe_terrain_set_status(wp_nav->update_wpnav());

	// call z-axis position controller (wpnav should have already updated it's alt target)
	pos_control->update_z_controller();

	// call attitude controller
	auto_takeoff_attitude_run(target_yaw_rate);
}

// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Copter::cruise_set_destination(const Vector3f& destination)
{
    // ensure we are in position control mode
    if (cruise_mode != Cruise_WP) {
    	cruise_pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    Location_Class dest_loc(destination);
    if (!fence.check_destination_within_fence(dest_loc)) {
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);

    // log target
    Log_Write_GuidedTarget(guided_mode, destination, Vector3f());
    return true;
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool Copter::cruise_set_destination(const Location_Class& dest_loc)
{
    // ensure we are in position control mode
    if (cruise_mode != Cruise_WP) {
        cruise_pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!fence.check_destination_within_fence(dest_loc)) {
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // log target
    Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}

// initialise cruise mode's position controller
void Copter::cruise_pos_control_start()
{
    // set to position control mode
    cruise_mode = Cruise_WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    stopping_point.z = inertial_nav.get_altitude();
    wp_nav->get_wp_stopping_point_xy(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::cruise_pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
}
