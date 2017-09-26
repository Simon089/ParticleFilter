//
//  singleTrackModel.cpp
//  ParticleFilter
//
//  Created by Simon Weigl on 04.07.17.
//  Copyright © 2017 Simon Weigl. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "singleTrackModel.h"

// Static distributions for the velocity and the measurement distance which be both need for our particles
static std::normal_distribution<double> velocityDistribution(0.0, MOVEMENT_ERROR);
static std::normal_distribution<double> distanceDistribution(0.0, measurementNoise);

// We use this rect to check if a particle is still in the world after a movement
static auto rect = cv::Rect(0, 0, WIDTH, HEIGTH);

/*
 * Measures the distance to the obstacles. Is used by the car and the particles.
 * Writes the distances to the obstacles in the measurement vector.
 * If the distance to an obstacle is bigger than the max measurement distance the value is 0.
 * The i-th measurement belongs to the i-th obstacle.
 *   IN obstacles: Our obstacles.
 */
void SingleTrackModel::measure(std::vector<cv::Rect> *obstacles) {
    measurements.clear();
    for (size_t i = 0; i < obstacles->size(); i++) {
        cv::Point centerRect = ((obstacles->at(i).br() + obstacles->at(i).tl()) * .5);
        auto distanceToObstacle = cv::norm(centerRect - position) + distanceDistribution(generator);
        measurements.push_back(distanceToObstacle > measurementDist ? 0.0 : distanceToObstacle);
    }
}

/*
 * This function draws our car onto the scene.
 * It also draws a circle to display the measurement distance and displays which obstacles are detected by the car.
 *   OUT scene: We draw the car and the measurements on this scene.
 *   IN obstacles: We need the obstacles to draw lines to them if we detect them.
 */
void Car::draw(cv::Mat &scene, std::vector<cv::Rect> *obstacles) {
    // Draw the car
    cv::circle(scene, worldToImage(position), CAR_RADIUS, cv::Scalar(0, 255), 2);

    // Draw the orientation
    auto ov = getOrientationVec();
    cv::line(scene, worldToImage(position),
             worldToImage(position.x + ov[0], position.y + ov[1]),
             cv::Scalar(0, 255), 2);

    // Draw the measurement distance
    cv::circle(scene, worldToImage(position), measurementDist, cv::Scalar(0, 0, 255));

    // Draw lines to the dectected obstacles
    for(size_t i = 0; i < measurements.size(); i++) {
        if(measurements.at(i) > 0) {
            auto mid = cv::Point((obstacles->at(i).br() + obstacles->at(i).tl()) * .5);
            cv::line(scene, worldToImage(position), worldToImage(mid), cv::Scalar(0, 0, 255, 0.5));
        }
    }
}

/*
 * A particle is a simple small circle.
 *   OUT: Scene to draw the particles on.
 */
void Particle::draw(cv::Mat &scene) {
    cv::circle(scene, worldToImage(position), 2, cv::Scalar(0, 125, 125), CV_FILLED);
}

/*
 * Movement of a singletrack model.
 *   IN d: the distance in pixel.
 *   IN steeringAngle: current steering angle in radians.
 */
void Car::move(double d, double steeringAngle) {
    double deltaOrientation = (d * tan(steeringAngle)) / AXLE_DISTANCE;
    orientation += deltaOrientation;

    double deltaX = d * cos(orientation);
    double deltaY = d * sin(orientation);
    position.x += deltaX;
    position.y += deltaY;

    orientation = normalizeAngle(orientation);
}

/*
 * Movement of a singletrack model but with a small error.
 *   IN d: the distance in pixel.
 *   IN steeringAngle: current steering angle in radians.
 */
void Particle::move(double d, double steeringAngle) {
    d += velocityDistribution(generator);

    double deltaOrientation = (d * tan(steeringAngle)) / AXLE_DISTANCE;
    orientation += deltaOrientation;

    double deltaX = d * cos(orientation);
    double deltaY = d * sin(orientation);
    position.x += deltaX;
    position.y += deltaY;

    orientation = normalizeAngle(orientation);
}

/*
 * Calculates the weight of an particle depending on its measurements.
 * If the particle is outside our world of inside an obstacle its weight is zero.
 * Also if either the particle or the car detects an obstacle and the other one doesn't, the weight is zero.
 * If both, the particle and the car, detect an obstacle a weight according the do difference is added.
 *   IN obstacles: We need the obstacles to test is the particle is inside of one.
 *   IN robotMeasurements: The measurements of the car to the obstacles.
 * Return the particles weight.
 */
double Particle::calculateWeight(std::vector<cv::Rect> *obstacles, std::vector<float> &robotMeasurements) {
    // Perform some checks
    if(!position.inside(rect)) {
        weight = 0;
        return 0;
    }
    if (robotMeasurements.size() != measurements.size()) {
        weight = 0;
        return 0;
    }

    // Test if point is inside an obstacle
    for(auto o = obstacles->begin(); o != obstacles->end(); ++o) {
        if (position.inside(*o)) {
            weight = 0;
            return 0;
        }
    }

    // Calculate weigth
    weight = 0.0;
    for (size_t i = 0; i < measurements.size(); i++) {
        auto myMeasurement = measurements.at(i);
        auto robotMeasurement = robotMeasurements.at(i);
        if ((myMeasurement > 0) && (robotMeasurement > 0)) { // only if both measurements are not zero
            weight += measurementDist - std::abs(robotMeasurement - myMeasurement);
        } else if((myMeasurement > 0) || (robotMeasurement > 0)) { // one measurement is zero, the other one isn't
            weight = 0;
            return 0;
        }
    }

    return weight;
}

/*
 * Sets the particles position and orientation to the new values.
 */
void Particle::reposition(double x, double y, double orientation_) {
    position.x = x;
    position.y = y;
    orientation = orientation_;
}

/*
 * Returns and input angle, but inside the interval 0 to 2*PI.
 *   IN angle: The angle we want to normalize in radians.
 * Returns the same angle.
 */
double normalizeAngle(double angle) {
    if (angle < 0) {
        angle += PI * 2;
    } else if (angle > (PI * 2)) {
        angle -= PI * 2;
    }
    return angle;
}
