// Copyright 2020 Hoffmann
#pragma once
#include <stdio.h>
#include <math.h>
#include "tod_helper/vehicle/Model.h"
#include <tod_msgs/VehicleEnums.h>
#include <algorithm>

class VehicleModel {
public:
    VehicleModel() { resetInitialPosition(0.0, 0.0, 0.0); }
    ~VehicleModel() {}
    void setParams(const float lf, const float lr, const float maxRWA, const float maxSWA);

    void resetInitialPosition(const double xIn, const double yIn, const double yawIn);
    void updatePosition(const double desiredVelocity, const double swa, const uint8_t gearPosition, const double dt);

    double getPsi_p();
    double getPsi();
    double getX();
    double getY();
    double getX_p();
    double getY_p();
    double getVx();
    double getVy();
    double getAx();
    double getAy();
    double getSteeringAngle();

private:
    void limitAngle(double angle);

    double _x, _y, _yaw;
    double _xp{0.0f}, _yp{0.0f}, _vx{0.0f}, _vy{0.0f}, _ax{0.0f}, _ay{0.0f};
    double _rwa{0.0f}, _yawRate{0.0f};
    const double _minAcceleration{-8.0}, _maxAcceleration{4.0};
    const double _maxRWARate{tod_helper::Vehicle::Model::deg2rad(90.0)};

    double _lf{0.0f}, _lr{0.0f}; // distance between CoM and front/rear axle
    float _maxRWA{0.0f}, _maxSWA{0.0f};
};
