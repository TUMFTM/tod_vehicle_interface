// Copyright 2020 Hoffmann
#pragma once
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "tod_helper_functions/VehicleModelHelpers.h"

class VehicleModel {
public:
    VehicleModel(float distance_front_axle, float distance_rear_axle,
                 float max_road_wheel_angle, float max_steering_wheel_angle);
    ~VehicleModel() {}

    void resetInitialPosition(const double xIn, const double yIn, const double yawIn);
    void updatePosition(const double v, const double dt, const double d);

    double getPsi_p();
    double getPsi();
    double getX();
    double getY();
    double getX_p();
    double getY_p();
    double getVx();
    double getVy();
    double getSteeringAngle();

private:
    void limitAngle(double angle);

    double xp{0.0f}, yp{0.0f}, vx{0.0f}, vy{0.0f};
    double steeringAngle{0.0f}, beta{0.0f}, yawRate{0.0f};
    double x, y, yaw;

    double veh_lf{0.0f}, veh_lr{0.0f}; // distance between CoM and front/rear axle
    float _max_road_wheel_angle{0.0f}, _max_steering_wheel_angle{0.0f};
};
