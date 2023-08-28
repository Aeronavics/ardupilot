#include "Copter.h"

#if MODE_HASTEN_ENABLED == ENABLED

/*
 * Init and run calls for hasten flight mode
 */

// hasten_init - initialise hasten controller
bool ModeHasten::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, hasten_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        hasten_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        hasten_nav->clear_pilot_desired_acceleration();
    }
    hasten_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

#if PRECISION_LANDING == ENABLED
bool ModeHasten::do_precision_hasten()
{
    if (!_precision_hasten_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (hasten_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeHasten::precision_hasten_xy()
{
    hasten_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel_rel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos.x = inertial_nav.get_position().x;
        target_pos.y = inertial_nav.get_position().y;
    }
    if (!copter.precland.get_target_velocity_relative_cms(target_vel_rel)) {
        target_vel_rel.x = -inertial_nav.get_velocity().x;
        target_vel_rel.y = -inertial_nav.get_velocity().y;
    }
    pos_control->set_pos_target_xy_cm(target_pos.x, target_pos.y);
    pos_control->override_vehicle_velocity_xy(-target_vel_rel);
}
#endif

// hasten_run - runs the hasten controller
// should be called at 100hz or more
void ModeHasten::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        hasten_nav->clear_pilot_desired_acceleration();
    }

    // relax hasten target if we might be landed
    if (copter.ap.land_complete_maybe) {
        hasten_nav->soften_for_landing();
    }

    // Hasten State Machine Determination
    AltHoldModeState hasten_state = get_alt_hold_state(target_climb_rate);

    // Hasten State Machine
    switch (hasten_state) {

    case AltHold_MotorStopped:
        _landing = false;
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        hasten_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(hasten_nav->get_thrust_vector(), target_yaw_rate);
        break;

    case AltHold_Takeoff:
        // initiate take-off
        _landing = false;
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        if (target_climb_rate >= 0) {
            // get takeoff speed from parameter
            target_climb_rate = g.pilot_takeoff_spd;
            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
        } else if (target_climb_rate >= -get_pilot_speed_dn()*0.8) {
            target_climb_rate = 0;
        }

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run hasten controller
        hasten_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(hasten_nav->get_thrust_vector(), target_yaw_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        _landing = false;
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        _landing = false;
        attitude_control->reset_rate_controller_I_terms_smoothly();
        hasten_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(hasten_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, hasten_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        hasten_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

#if PRECISION_LANDING == ENABLED
        if (do_precision_hasten()) {
            precision_hasten_xy();
        }
#endif

        // run hasten controller
        hasten_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(hasten_nav->get_thrust_vector(), target_yaw_rate);

        
        if (
            _landing || (
                copter.rangefinder_state.enabled && 
                copter.rangefinder_state.alt_healthy && 
                copter.rangefinder_state.alt_cm_glitch_protected <= (g.pilot_takeoff_alt + copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270) + 10) && 
                target_climb_rate <= -get_pilot_speed_dn()*0.99 && 
                copter.gps.ground_speed_cm() <= 50
            )
        ) {

            if (!_landing) {
                _land_start_time = millis();
                _land_pause = true;
                _landing = true;
            }

            // pause before beginning land descent
            if (_land_pause && millis()-_land_start_time >= LAND_WITH_DELAY_MS) {
                _land_pause = false;
            }

            // cancel landing if throttle is not at minimum and the landing descent has not yet started
            if (_land_pause && (target_climb_rate > -get_pilot_speed_dn()*0.99 || target_yaw_rate < -500 || target_yaw_rate > 500 || copter.gps.ground_speed_cm() >= 50)) {
                _landing = false;
            }
            
            // cancel landing if landing descent has started and high throttle cancels landing is set and throttle is high
            if (!_land_pause && (g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
                _landing = false;
            }

            if (_landing) {
                auto_yaw.set_mode(AUTO_YAW_HOLD);
                land_run_horizontal_control();
                land_run_vertical_control(_land_pause);
            }

        }
        else 
        {
            Matrix3f rngRotMatrix = {
                    Vector3f{copter.ahrs.cos_pitch(),   copter.ahrs.sin_pitch() * copter.ahrs.sin_roll(),   copter.ahrs.sin_pitch() * copter.ahrs.cos_roll()}, 
                    Vector3f{0,                         copter.ahrs.cos_roll(),                             -copter.ahrs.sin_roll()},
                    Vector3f{-copter.ahrs.sin_pitch(),  copter.ahrs.cos_pitch() * copter.ahrs.sin_roll(),   copter.ahrs.cos_pitch() * copter.ahrs.cos_roll()}
                };
                
            int16_t fc_height_rng = copter.rangefinder_state.alt_cm_glitch_protected - (rngRotMatrix * copter.rangefinder.get_pos_offset_orient(ROTATION_PITCH_270)).z;

            if (copter.rangefinder_state.enabled && copter.rangefinder_state.alt_healthy && fc_height_rng < (g.pilot_takeoff_alt + copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270))*0.85) {
                target_climb_rate = MAX(target_climb_rate, MIN(powF(fc_height_rng - g.pilot_takeoff_alt, 2) / (powF(g.pilot_takeoff_alt, 2)), 1) * g.pilot_speed_up);
            }
            else if (copter.rangefinder_state.enabled && copter.rangefinder_state.alt_healthy && target_climb_rate < 0) {

                //target_climb_rate = MAX(target_climb_rate, MIN(((fc_height_rng * fc_height_rng) / (3 * ((g.pilot_takeoff_alt * g.pilot_takeoff_alt) + ((get_pilot_speed_dn() * get_pilot_speed_dn()) / 2)))), 1) * -get_pilot_speed_dn());
                target_climb_rate = MAX(target_climb_rate, MIN(powF((fc_height_rng - (g.pilot_takeoff_alt * 0.85)) / (2 * get_pilot_speed_dn()), 2) + 0.05, 1) * -get_pilot_speed_dn());

                if (fc_height_rng <= g.pilot_takeoff_alt + copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270)) {
                    target_climb_rate = 0;
                }
            }
            
            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // update the vertical offset based on the surface measurement
            copter.surface_tracking.update_surface_offset();

            // Send the commanded climb rate to the position controller
            pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        }
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

uint32_t ModeHasten::wp_distance() const
{
    return hasten_nav->get_distance_to_target();
}

int32_t ModeHasten::wp_bearing() const
{
    return hasten_nav->get_bearing_to_target();
}

bool ModeHasten::is_landing() const
{
    return _landing;
}

void ModeHasten::exit()
{
    _landing = false;
}

#endif
