// Copyright 2020 Hoffmann
#include "VehicleModel.h"

VehicleModel::VehicleModel(float distance_front_axle, float distance_rear_axle,
                           float max_road_wheel_angle, float max_steering_wheel_angle) {
    resetInitialPosition(0.0, 0.0, 0.0); // initial position
    veh_lf = distance_front_axle;
    veh_lr = distance_rear_axle;
    _max_road_wheel_angle = max_road_wheel_angle;
    _max_steering_wheel_angle = max_steering_wheel_angle;
}

void VehicleModel::resetInitialPosition(const double xIn, const double yIn, const double yawIn) {
    x = xIn;
    y = yIn;
    yaw = yawIn;
}

void VehicleModel::updatePosition(const double desiredVelocity,  const double steeringWheelAngle, const double dt) {
    steeringAngle = VehicleModelHelpers::swa2rwa(steeringWheelAngle, _max_steering_wheel_angle, _max_road_wheel_angle);
    beta = std::atan(std::tan(steeringAngle) * veh_lr / (veh_lr + veh_lf));

    // velocity in vehicle frame
    vx = desiredVelocity * std::cos(beta);
    vy = desiredVelocity * std::sin(beta);

    // velocity / position in odom frame
    xp = desiredVelocity * std::cos(beta + yaw);
    x =  xp * dt + x;

    yp = desiredVelocity * std::sin(beta + yaw);
    y =  yp * dt + y;

    yawRate = std::sin(beta) * desiredVelocity / veh_lr;
    yaw = yawRate * dt + yaw;
    limitAngle(yaw); // limit from -pi to pi
}

void VehicleModel::limitAngle(double angle) {
    const double pi = 3.1415;
    int n = std::floor(std::abs(angle) / (2 * pi));
    n = (angle > 0) ? n : -1 * n;
    angle = angle - n * (2 * pi); // remove full turns

    if (angle > pi) angle = angle - 2 * pi;
    if (angle < -pi) angle = angle + 2 * pi;
}

double VehicleModel::getPsi() { return yaw; }
double VehicleModel::getPsi_p() { return yawRate; }
double VehicleModel::getX() { return x; }
double VehicleModel::getY() { return y; }
double VehicleModel::getX_p() { return xp; }
double VehicleModel::getY_p() { return yp; }
double VehicleModel::getVx() { return vx; }
double VehicleModel::getVy() { return vy; }
double VehicleModel::getSteeringAngle() { return steeringAngle; }
