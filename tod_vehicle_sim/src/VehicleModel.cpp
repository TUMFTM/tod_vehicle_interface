// Copyright 2020 Hoffmann
#include "VehicleModel.h"

void VehicleModel::setParams(const float lf, const float lr, const float maxRWA, const float maxSWA) {
    _lf = lf;
    _lr = lr;
    _maxRWA = maxRWA;
    _maxSWA = maxSWA;
}

void VehicleModel::resetInitialPosition(const double xIn, const double yIn, const double yawIn) {
    _x = xIn;
    _y = yIn;
    _yaw = yawIn;
    _xp = _yp = _vx = _vy = _ax = _ay = _rwa = _yawRate = 0.0f;
}

void VehicleModel::updatePosition(const double desiredVelocity, const double swa,
                                  const uint8_t gearPosition, const double dt) {
    double previousVelocity = std::sqrt(std::pow(_vx, 2) + std::pow(_vy, 2));
    double maxVelocity = previousVelocity + dt * _maxAcceleration;
    double minVelocity = previousVelocity + dt * _minAcceleration;
    double currentVelocity = std::clamp(desiredVelocity, minVelocity, maxVelocity);

    double minRWA = std::max(_rwa - dt * _maxRWARate, double(-_maxRWA));
    double maxRWA = std::min(_rwa + dt * _maxRWARate, double(+_maxRWA));
    _rwa = std::clamp(tod_helper::Vehicle::Model::swa2rwa(swa, _maxSWA, _maxRWA),
                      minRWA, maxRWA);
    double beta = std::atan(std::tan(_rwa) * _lr / (_lr + _lf));

    // velocities in vehicle frame
    _vx = currentVelocity * std::cos(beta);
    _vy = currentVelocity * std::sin(beta);
    _yawRate = std::sin(beta) * currentVelocity / _lr;
    // velocity / position in odom frame
    _xp = currentVelocity * std::cos(beta + _yaw);
    _yp = currentVelocity * std::sin(beta + _yaw);

    // change sign in gear position reverse
    if (gearPosition == eGearPosition::GEARPOSITION_REVERSE) {
        _vx *= (-1);
        _vy *= (-1);
        _yawRate *= (-1);
        _xp *= (-1);
        _yp *= (-1);
    }

    // integrate positions
    _x += _xp * dt;
    _y += _yp * dt;
    _yaw += _yawRate * dt;
    limitAngle(_yaw); // limit from -pi to pi

    double radius = (_lf + _lr) / std::tan(_rwa);
    _ay = currentVelocity * currentVelocity / radius;
    _ax = (currentVelocity - previousVelocity) / dt;
}

void VehicleModel::limitAngle(double angle) {
    const double pi = 3.1415;
    int n = std::floor(std::abs(angle) / (2 * pi));
    n = (angle > 0) ? n : -1 * n;
    angle = angle - n * (2 * pi); // remove full turns

    if (angle > pi) angle = angle - 2 * pi;
    if (angle < -pi) angle = angle + 2 * pi;
}

double VehicleModel::getPsi() { return _yaw; }
double VehicleModel::getPsi_p() { return _yawRate; }
double VehicleModel::getX() { return _x; }
double VehicleModel::getY() { return _y; }
double VehicleModel::getX_p() { return _xp; }
double VehicleModel::getY_p() { return _yp; }
double VehicleModel::getVx() { return _vx; }
double VehicleModel::getVy() { return _vy; }
double VehicleModel::getAx() { return _ax; }
double VehicleModel::getAy() { return _ay; }
double VehicleModel::getSteeringAngle() { return _rwa; }
