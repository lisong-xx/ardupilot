#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED
/*
 * Init and run calls for guided flight mode
 */

static uint32_t update_time_ms;  // system time of last target update to pos_vel_accel, vel_accel or accel controller

// init - initialise guided controller
bool ModeNarrowGap::init(bool ignore_checks) {

    path_num = 0;                     // 航点号清零，从而切到其他模式再切回来后，可以飞出一个新的航线
    float takeoff_hight_cm = 100.0;   //并不起作用，在guided模式下，飞机不会自动起飞，仍需要手动输入命令行 takeoff + altitude（m）

    // start in velaccel control mode
    do_user_takeoff_start(takeoff_hight_cm);

    // clear pause state when entering guided mode
    _paused = false;
    update_time_ms = millis();
    return true;
}

//设置过窄缝航线
void ModeNarrowGap::generate_path(){

    float distance_forward_cm = 100.0;   //航线向前飞行的距离

    wp_nav->get_wp_stopping_point(path[0]);  //获取当前悬停点的位置坐标

    path[1] = path[0] + Vector3f(0,0,-0.5f) * distance_forward_cm;  //下降0.5m
    path[2] = path[1] + Vector3f(2.0f,0,0) * distance_forward_cm;   //前飞1m
    path[3] = path[2] + Vector3f(0,0.5f,0) * distance_forward_cm;   //向右飞0.5m
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeNarrowGap::run() {

    // set velocity to zero and stop rotating if no updates received for 10 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        _paused = false;
#if LANDING_GEAR_ENABLED == ENABLED
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();
#endif
    }

    // run pause control if the vehicle is paused
    if (_paused) {
        pause_control_run();
        return;
    }

    // call the correct auto controller
    if (guided_mode == SubMode::TakeOff) {
        // run takeoff controller
        takeoff_run();
    } else {

    if (path_num < 3) {  // 航线尚未走完
        if (wp_nav->reached_wp_destination()) {  // 到达某个端点
            path_num++;
            wp_nav->set_wp_destination(path[path_num], false);  // 将下一个航点位置设置为导航控制模块的目标位置
        }
    } else if ((path_num == 3) && wp_nav->reached_wp_destination()) {  // 航线运行完成，自动进入Land模式
        gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into land mode");
        copter.set_mode(Mode::Number::LAND, ModeReason::MISSION_END);  // 切换到land模式
    }
        // run position controller
        wp_control_run();
    }
}

// return guided mode timeout in milliseconds. 
uint32_t ModeNarrowGap::get_timeout_ms() const
{
    return 10.0 * 1000; // 悬停时间（ms）
}

// takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void ModeNarrowGap::takeoff_run() {
    auto_takeoff_run();
    update_time_ms = millis(); //获取当前时刻的系统时间
    if (auto_takeoff_complete && !takeoff_complete) {
        takeoff_complete = true;
        _paused = true;
        generate_path();           //生成接下来的运动轨迹
        wp_control_start();        //航迹控制初始化
    }
}

// 这一段没有修改，是对takeoff.cpp里的重写override
// initialises position controller to implement take-off
// takeoff_alt_cm is interpreted as alt-above-home (in cm) or alt-above-terrain
// if a rangefinder is available
bool ModeNarrowGap::do_user_takeoff_start(float takeoff_alt_cm) {
    guided_mode = SubMode::TakeOff;

    // calculate target altitude and frame (either alt-above-ekf-origin or
    // alt-above-terrain)
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() ==
            AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_cm <
            copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        // provide target altitude as alt-above-terrain
        alt_target_cm = takeoff_alt_cm;
        alt_target_terrain = true;
    } else {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,
                                   alt_target_cm)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff_start(alt_target_cm, alt_target_terrain);

    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}

// 这一段没有修改，
bool ModeNarrowGap::is_taking_off() const {
    return guided_mode == SubMode::TakeOff && !takeoff_complete;
}


// 将stopping point 给到 path[0]，由于后续航点的计算
// initialise guided mode's waypoint navigation controller
void ModeNarrowGap::wp_control_start()
{
    // set to position control mode
    guided_mode = SubMode::WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    wp_nav->get_wp_stopping_point(path[0]);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    //auto_yaw.set_mode_to_default(false);
    auto_yaw.set_mode(AUTO_YAW_FIXED);
}

//这一段也没有修改
// run guided mode's waypoint navigation controller
void ModeNarrowGap::wp_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
//    if (path_num >= 1 && path_num < 3) {
//        auto_yaw.set_mode(AUTO_YAW_FIXED);
//        auto_yaw.set_fixed_yaw(90.0f, 360.0f, (int8_t)(1), false);
//    } else {
        if (!copter.failsafe.radio && use_pilot_yaw()) {
            // get pilot's desired yaw rate
            target_yaw_rate =
                get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
            if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            }
        }
//    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
    }
}


// 这一段也没有修改
// pause_control_run - runs the guided mode pause controller
// called from guided_run
void ModeNarrowGap::pause_control_run() {
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock
        // enabled
        make_safe_ground_handling(copter.is_tradheli() &&
                                  motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(
        AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set the horizontal velocity and acceleration targets to zero
    Vector2f vel_xy, accel_xy;
    pos_control->input_vel_accel_xy(vel_xy, accel_xy, false);

    // set the vertical velocity and acceleration targets to zero
    float vel_z = 0.0;
    pos_control->input_vel_accel_z(vel_z, 0.0, false, false);

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(
        pos_control->get_thrust_vector(), 0.0);
}

//这一段也没有修改
//bool ModeNarrowGap::allows_arming(AP_Arming::Method method) const
//{
    // always allow arming from the ground station
    //if (method == AP_Arming::Method::MAVLINK) {
    //   return true;
    //}

    // optionally allow arming from the transmitter
   // return (copter.g2.guided_options & (uint32_t)Options::AllowArmingFromTX) != 0;
//};

#endif
