#include "Copter.h"

#if MODE_TARLAND_ENABLED

#define MIN_DESCENT_ALT_OFF 1.5f   // switch to LAND at 0.5 m above target
#define MIN_FOLLOW_DIST     0.5f   // horizontal error must be <= 0.5 m
#define HOVER_ALT           20.0f  // normal follow altitude
#define INTERMEDIATE_ALT    10.0f
#define INTERMEDIATE_DIST   5.0f

#define TARLAND_SOFT_DESCENT_SPEED_CMS 30.0f  // Downward speed: cm/s

/**
 * mode_tarland.cpp - follow another mavlink enabled vehicle and land on it
 * Use AP_FOLLOW library to achieve follow the target
 */
bool ModeTarLand::init(bool ignore_checks)
{

    if(!ignore_checks){
        if(!AP::ahrs().home_is_set()){
            return false;
        }
    }

    if(!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

    // re use guided mode
    return ModeGuided::init(ignore_checks);
}


void ModeTarLand::run()
{
    if (!motors->armed()) {
        return;
    }

    // Temporary landing/disarm diagnostics
    // TEMPORARY: landing/disarm diagnostics once per second
    static uint32_t last_land_diag_ms = 0;
    const uint32_t now = AP_HAL::millis();

    if ((now - last_land_diag_ms) >= 1000U) {
        last_land_diag_ms = now;

        GCS_SEND_TEXT(
            MAV_SEVERITY_WARNING,
            "TL LAND: complete=%d maybe=%d spool=%d manual=%d",
            (int)copter.ap.land_complete,
            (int)copter.ap.land_complete_maybe,
            (int)motors->get_spool_state(),
            (int)has_manual_throttle()
        );
    }
    
    // Terminal landing state: do not issue more tracking
    // or descent commands after confirmed touchdown.
    if (copter.ap.land_complete) {
        make_safe_ground_handling();

        // Follow the same touchdown/disarm condition used
        // by native Copter LAND mode.
        if (motors->get_spool_state() ==
            AP_Motors::SpoolState::GROUND_IDLE) {

            GCS_SEND_TEXT(
                MAV_SEVERITY_WARNING,
                "TL: explicit disarm attempt"
            );

            const bool disarmed = copter.arming.disarm(AP_Arming::Method::LANDED);

            GCS_SEND_TEXT(
                MAV_SEVERITY_WARNING,
                "TL: disarm result=%d armed=%d",
                (int)disarmed,
                (int)motors->armed()

            );
        }

        return;
    }

    get_target_info();

    if (!has_target_info) {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
        //     "Has Target Info=%d ",
        //     (int)has_target_info);
        return;
    }

    // Actual physical horizontal distance between UAV and target
    const float actual_dist_xy = dist_vec.xy().length();

    // Horizontal distance after applying follow offset
    const float offset_dist_xy = dist_vec_offs.xy().length();

    // Debug output
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "ACTUAL XY: %.3f | OFFSET XY: %.3f | Z: %.3f",
                  actual_dist_xy,
                  offset_dist_xy,
                  dist_vec.z);

    // =========================================================
    // PHASE 1
    //
    // Horizontal distance > 5 m
    //
    // Follow target at normal altitude = 40 m
    // =========================================================
    if (actual_dist_xy > INTERMEDIATE_DIST) {

        // Vector3f offset = g2.follow.get_offset();
        Vector3f offset = {0, 0, -HOVER_ALT};

        g2.follow.set_offset(offset, offset_type);

        follow_target();

        return;
    }

    // =========================================================
    // PHASE 2
    //
    // 0.5 m < horizontal distance <= 5 m
    //
    // Reduce follow altitude from 40 m to 10 m
    // while continuing to track the moving target.
    // =========================================================
    
    if (actual_dist_xy > MIN_FOLLOW_DIST) {

        // Vector3f offset = g2.follow.get_offset();
        Vector3f offset = {0, 0, -INTERMEDIATE_ALT};

        // offset.z = -INTERMEDIATE_ALT;

        g2.follow.set_offset(offset, offset_type);

        follow_target();

        return;
    }

    // =========================================================
    // PHASE 3
    //
    // Horizontal distance <= 0.5 m
    //
    // Continue tracking the moving target and descend toward
    // the offset zero altitude of 1 m.
    // =========================================================
    
    if (dist_vec.z > MIN_DESCENT_ALT_OFF) {

        // Vector3f offset = g2.follow.get_offset();
        Vector3f offset = {0, 0, -MIN_DESCENT_ALT_OFF};

        // offset.z = -MIN_DESCENT_ALT_OFF;

        g2.follow.set_offset(offset, offset_type);

        follow_target();

        return;
    }

    // if (actual_dist_xy <= MIN_FOLLOW_DIST &&
    //     dist_vec.z <= MIN_DESCENT_ALT_OFF) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,
    //                 "LAND CMD PASSED | XY: %.3f m | Z: %.3f m",
    //                 actual_dist_xy,
    //                 dist_vec.z);

    //     gcs().send_named_float("TL_EVT", 1);  // 1 = TarLand -> LAND handoff
    //     set_mode(Mode::Number::LAND, ModeReason::UNKNOWN);

    //     return;
    // }

    // =========================================================
    // PHASE 4: SOFT LANDING
    //
    // Target-relative horizontal distance <= 0.5 m
    // Target-relative vertical distance <= 1.5 m
    // =========================================================
   
    if (actual_dist_xy <= MIN_FOLLOW_DIST &&
        dist_vec.z <= MIN_DESCENT_ALT_OFF) {

        soft_land_target();
        return;
    }
}


void ModeTarLand::get_target_info()
{

    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
    //         "Call follow mode");
    has_target_info = g2.follow.get_target_dist_and_vel_NED_m(dist_vec, dist_vec_offs, target_vel);
}


void ModeTarLand::follow_target(bool soft_landing)
{

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert pos_err_off to cm in NEU frame
    const Vector3f pos_err_off_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);
    get_desired_vel_neu_cms(desired_velocity_neu_cms, pos_err_off_neu, dist_vec, dist_vec_offs);

    limit_desired_velocity_xy(desired_velocity_neu_cms, pos_err_off_neu);
    add_feedforward_velocity_xy(desired_velocity_neu_cms);

    limit_desired_velocity_z(desired_velocity_neu_cms, pos_err_off_neu);
    add_feedforward_velocity_z(desired_velocity_neu_cms);
    
    if (soft_landing) {
        // NEU frame: negative Z means downward.
        desired_velocity_neu_cms.z = -TARLAND_SOFT_DESCENT_SPEED_CMS - target_vel.z * 100.0f;
    }
    
    // limit the velocity for obstacle/fence avoidance
    copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_NE_p().kP().get(), pos_control->get_max_accel_NE_cmss(), pos_control->get_pos_U_p().kP().get(), pos_control->get_max_accel_U_cmss(), G_Dt);

    ModeTarLand::yaw_behaviour yaw_info = get_yaw_behaviour();

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 1000) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
        Location current_loc;
        hal.console->printf("Mode Tarland\n");
        hal.console->printf(
            "ACTUAL target: X=%f Y=%f Z=%f Dist=%f\n",
            dist_vec.x,
            dist_vec.y,
            dist_vec.z,
            dist_vec.xy().length()
        );

        hal.console->printf(
            "OFFSET target: X=%f Y=%f Z=%f Dist=%f\n",
            dist_vec_offs.x,
            dist_vec_offs.y,
            dist_vec_offs.z,
            dist_vec_offs.xy().length()
        );
        hal.console->printf("Drone z pos: %f \n", current_loc.alt*0.01f);
    }

    log_tarland_diagnostics();

    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, yaw_info.use_yaw, yaw_info.yaw_cd, false, 0.0f, false, log_request);
    
    ModeGuided::run();
}

/**
 * Calculates the desired velocity from position error using P controller
 */
void ModeTarLand::get_desired_vel_neu_cms(Vector3f &desired_vel_neu_cms, const Vector3f pos_err_off_neu, Vector3f pos_err, Vector3f pos_err_off)
{

    // calculate the desired relative velocity vector in cm/s in NEU frame
const float kp = g2.follow.get_pos_p().kP();
    desired_vel_neu_cms = pos_err_off_neu*kp;
}

void ModeTarLand::limit_desired_velocity_xy(Vector3f &desired_vel_neu_cms, Vector3f pos_err_off_neu)
{
    // create horizontal unit vector towards target (required for slow down calculations)
    Vector2f dir_to_target_xy(desired_vel_neu_cms.x, desired_vel_neu_cms.y);
    if (!dir_to_target_xy.is_zero()) {
        dir_to_target_xy.normalize();
    }

    // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
    const float dist_to_target_xy = Vector2f(pos_err_off_neu.x, pos_err_off_neu.y).length();
    // apply sqrt controller to limit velocity
    Vector2f desired_vel_xy_cms(desired_vel_neu_cms.x, desired_vel_neu_cms.y);
    copter.avoid.limit_velocity_2D(pos_control->get_pos_NE_p().kP().get(), pos_control->get_max_accel_NE_cmss() * 0.5f, desired_vel_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
    desired_vel_neu_cms.xy() = desired_vel_xy_cms;
}

void ModeTarLand::limit_desired_velocity_z(Vector3f &desired_vel_neu_cms, Vector3f pos_err_off_neu)
{
    // limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
    const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_U_p().kP().get(), pos_control->get_max_accel_U_cmss() * 0.5f, fabsf(pos_err_off_neu.z), copter.G_Dt);
    desired_vel_neu_cms.z = constrain_float(desired_vel_neu_cms.z, -des_vel_z_max, des_vel_z_max);
}

void ModeTarLand::add_feedforward_velocity_xy(Vector3f &desired_vel_neu_cms)
{
    desired_vel_neu_cms.xy() += target_vel.xy()*100.0f;
    desired_vel_neu_cms.xy().limit_length(pos_control->get_max_speed_NE_cms());
}

void ModeTarLand::add_feedforward_velocity_z(Vector3f &desired_vel_neu_cms)
{
    desired_vel_neu_cms.z += -target_vel.z * 100.0f;
    // limit desired velocity to be between maximum climb and descent rates
    desired_vel_neu_cms.z = constrain_float(desired_vel_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

}

ModeTarLand::yaw_behaviour ModeTarLand::get_yaw_behaviour()
{
    bool use_yaw = false;
    float yaw_cd = 0.0f;
    // calculate vehicle heading
    switch (g2.follow.get_yaw_behave()) {
        case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
            if (dist_vec.xy().length_squared() > 1.0) {
                yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
            float target_hdg = 0.0f;
            if (g2.follow.get_target_heading_deg(target_hdg)) {
                yaw_cd = target_hdg * 100.0f;
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
            if (desired_velocity_neu_cms.xy().length_squared() > (100.0 * 100.0)) {
                yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_neu_cms.xy());
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_NONE:
        default:
            // do nothing
            break;

    }

    ModeTarLand::yaw_behaviour yaw_info;
    yaw_info.use_yaw = use_yaw;
    yaw_info.yaw_cd = yaw_cd;

    return yaw_info;
}

void ModeTarLand::soft_land_target()
{
    // Track the target's zero-offset position during descent.
    const Vector3f offset = {0.0f, 0.0f, 0.0f};
    g2.follow.set_offset(offset, offset_type);

    // Refresh the target-relative estimates after changing offset.
    get_target_info();

    if (!has_target_info) {
        return;
    }

    // Reuse the existing controller with soft descent enabled.
    follow_target(true);
}

void ModeTarLand::log_tarland_diagnostics()
{
    // Limit diagnostic traffic to 2 Hz.
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_diag_ms < 500) {
        return;
    }
    last_diag_ms = now_ms;

    // Target-relative position (NED, metres).
    gcs().send_named_float("TL_DN", dist_vec.x);
    gcs().send_named_float("TL_DE", dist_vec.y);
    gcs().send_named_float("TL_DD", dist_vec.z);

    // Offset-relative position (NED, metres).
    gcs().send_named_float("TL_ON", dist_vec_offs.x);
    gcs().send_named_float("TL_OE", dist_vec_offs.y);
    gcs().send_named_float("TL_OD", dist_vec_offs.z);

    // Target velocity. Verify the getter's frame and units
    // in your checkout before interpreting these values.
    gcs().send_named_float("TL_TVX", target_vel.x);
    gcs().send_named_float("TL_TVY", target_vel.y);
    gcs().send_named_float("TL_TVZ", target_vel.z);

    // Final velocity setpoint sent to Guided, in NEU cm/s.
    gcs().send_named_float("TL_CVX", desired_velocity_neu_cms.x);
    gcs().send_named_float("TL_CVY", desired_velocity_neu_cms.y);
    gcs().send_named_float("TL_CVZ", desired_velocity_neu_cms.z);

    // Current phase/state, represented numerically.
    gcs().send_named_float("TL_STATE", static_cast<int32_t>(_state));
}

#endif