﻿//
// Created by User on 2019/4/2.
//

#include "pm1_sdk.h"
#include "pm1_sdk_native.h"

#include <algorithm>
#include <functional>
#include <thread>

#include "internal/serial/serial.h"
#include "internal/time_extensions.h"

extern "C"
{
#include "internal/control_model/chassis_config_t.h"
}

autolabor::pm1::result<void>
on_native(autolabor::pm1::native::handler_t handler) {
    using namespace autolabor::pm1;
    
    auto error = std::string(native::get_error_info(handler));
    native::remove_error_info(handler);
    return {error};
}

std::vector<std::string> autolabor::pm1::serial_ports() {
    auto                     info = serial::list_ports();
    std::vector<std::string> result(info.size());
    std::transform(info.begin(), info.end(), result.begin(),
                   [](const serial::PortInfo &it) { return it.port; });
    return result;
}

autolabor::pm1::result<std::string>
autolabor::pm1::initialize(const std::string &port,
                           double *progress) {
    double _progress;
    auto   handler = native::initialize(port.c_str(),
                                        progress ? *progress : _progress);
    auto   error   = std::string(native::get_error_info(handler));
    native::remove_error_info(handler);
    
    return {error, std::string(native::get_current_port())};
}

autolabor::pm1::result<void>
autolabor::pm1::shutdown() {
    return on_native(native::shutdown());
}

double
autolabor::pm1::get_defualt_parameter(autolabor::pm1::parameter_id id) {
    return native::get_default_parameter(static_cast<native::handler_t>(id));
}

autolabor::pm1::result<double>
autolabor::pm1::get_parameter(autolabor::pm1::parameter_id id) {
    double value;
    auto   result = on_native(
        native::get_parameter(
            static_cast<native::handler_t>(id), value));
    return {result.error_info, value};
}

autolabor::pm1::result<void>
autolabor::pm1::set_parameter(autolabor::pm1::parameter_id id, double value) {
    return on_native(
        native::set_parameter(
            static_cast<native::handler_t>(id), value));
}

autolabor::pm1::result<void>
autolabor::pm1::reset_parameter(autolabor::pm1::parameter_id id) {
    return on_native(
        native::reset_parameter(
            static_cast<native::handler_t>(id)));
}

autolabor::pm1::result<autolabor::pm1::odometry>
autolabor::pm1::get_odometry() {
    double   _;
    odometry temp{};
    
    auto handler = native::get_odometry(_, _,
                                        temp.x, temp.y, temp.yaw,
                                        temp.vx, temp.vy, temp.w);
    auto error   = std::string(native::get_error_info(handler));
    native::remove_error_info(handler);
    return {error, temp};
}

autolabor::pm1::result<void>
autolabor::pm1::reset_odometry() {
    return on_native(native::reset_odometry());
}

autolabor::pm1::result<void>
autolabor::pm1::lock() {
    return on_native(native::set_enabled(false));
}

autolabor::pm1::result<void> autolabor::pm1::unlock() {
    return on_native(native::set_enabled(true));
}

autolabor::pm1::chassis_state
autolabor::pm1::check_state() {
    return static_cast<chassis_state>(native::check_state());
}

void autolabor::pm1::delay(double time) {
    std::this_thread::sleep_for(seconds_duration(time));
}

autolabor::pm1::result<void>
autolabor::pm1::drive_physical(double speed, double rudder) {
    return on_native(native::drive_physical(speed, rudder));
}

autolabor::pm1::result<void>
autolabor::pm1::drive_wheels(double left, double right) {
    return on_native(native::drive_wheels(left, right));
}

autolabor::pm1::result<void>
autolabor::pm1::drive_velocity(double v, double w) {
    return on_native(native::drive_velocity(v, w));
}

autolabor::pm1::result<void>
autolabor::pm1::drive(double v, double w) {
    return on_native(native::drive_velocity(v, w));
}

constexpr auto
    infinite_action = "action never complete",
    negative_target = "action target argument must be positive";

autolabor::pm1::result<void>
autolabor::pm1::go_straight(double speed,
                            double meters,
                            double *progress) {
    if (meters == 0)
        return {};
    if (speed == 0)
        return {infinite_action};
    if (meters < 0)
        return {negative_target};
    
    return drive_spatial(speed, 0,
                         calculate_spatium(meters, 0),
                         progress);
}

autolabor::pm1::result<void>
autolabor::pm1::go_straight_timing(double speed,
                                   double seconds,
                                   double *progress) {
    if (seconds == 0)
        return {};
    if (seconds < 0)
        return {negative_target};
    
    return drive_timing(speed, 0,
                        seconds,
                        progress);
}

autolabor::pm1::result<void>
autolabor::pm1::turn_around(double speed,
                            double rad,
                            double *progress) {
    if (rad == 0)
        return {};
    if (speed == 0)
        return {infinite_action};
    if (rad < 0)
        return {negative_target};
    
    return drive_spatial(0, speed,
                         calculate_spatium(0, rad),
                         progress);
}

autolabor::pm1::result<void>
autolabor::pm1::turn_around_timing(double speed,
                                   double seconds,
                                   double *progress) {
    if (seconds == 0)
        return {};
    if (seconds < 0)
        return {negative_target};
    
    return drive_timing(0, speed,
                        seconds,
                        progress);
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_vs(double v,
                          double r,
                          double s,
                          double *progress) {
    if (r == 0)
        return {"illegal action parameter"};
    if (s == 0)
        return {};
    if (v == 0)
        return {infinite_action};
    if (s < 0)
        return {negative_target};
    
    return drive_spatial(v, v / r,
                         calculate_spatium(s, s / r),
                         progress);
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_va(double v,
                          double r,
                          double a,
                          double *progress) {
    if (r == 0)
        return {"illegal action parameter"};
    if (a == 0)
        return {};
    if (v == 0)
        return {infinite_action};
    if (a < 0)
        return {negative_target};
    
    return drive_spatial(v, v / r,
                         calculate_spatium(a * r, a),
                         progress);
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_ws(double w,
                          double r,
                          double s,
                          double *progress) {
    if (r == 0)
        return {"illegal action parameter"};
    if (s == 0)
        return {};
    if (w == 0)
        return {infinite_action};
    if (s < 0)
        return {negative_target};
    
    return drive_spatial(w * r, w,
                         calculate_spatium(s, s / r),
                         progress);
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_wa(double w,
                          double r,
                          double a,
                          double *progress) {
    if (r == 0)
        return {"illegal action parameter"};
    if (a == 0)
        return {};
    if (w == 0)
        return {infinite_action};
    if (a < 0)
        return {negative_target};
    
    return drive_spatial(w * r, w,
                         calculate_spatium(a * r, a),
                         progress);
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_vt(double v,
                          double r,
                          double t,
                          double *progress) {
    if (r == 0)
        return {"illegal action parameter"};
    if (t == 0)
        return {};
    if (t < 0)
        return {negative_target};
    
    return drive_timing(v, v / r, t, progress);
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_wt(double w,
                          double r,
                          double t,
                          double *progress) {
    if (r == 0)
        return {"illegal action parameter"};
    if (t == 0)
        return {};
    if (t < 0)
        return {negative_target};
    
    return drive_timing(w * r, w, t, progress);
}

void autolabor::pm1::pause() {
    native::set_paused(true);
}

void autolabor::pm1::resume() {
    native::set_paused(false);
}

bool autolabor::pm1::is_paused() {
    return native::is_paused();
}

void autolabor::pm1::cancel_action() {
    native::cancel_action();
}

double
autolabor::pm1::calculate_spatium(double spatium, double angle) {
    return native::calculate_spatium(spatium, angle);
}

autolabor::pm1::result<void>
autolabor::pm1::drive_spatial(double v,
                              double w,
                              double spatium,
                              double *progress) {
    double _progress;
    return on_native(
        native::drive_spatial(
            v, w, spatium,
            progress ? *progress : _progress));
}

autolabor::pm1::result<void>
autolabor::pm1::drive_timing(double v,
                             double w,
                             double time,
                             double *progress) {
    double _progress;
    return on_native(
        native::drive_timing(
            v, w, time,
            progress ? *progress : _progress));
}

autolabor::pm1::result<void>
autolabor::pm1::adjust_rudder(double offset, double *progress) {
    double _progress;
    return on_native(
        native::adjust_rudder(
            offset,
            progress ? *progress : _progress));
}
