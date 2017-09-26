//
//  singleTrackModel.h
//  ParticleFilter
//
//  Created by Simon Weigl on 04.07.17.
//  Copyright © 2017 Simon Weigl. All rights reserved.
//

#ifndef singleTrackModel_h
#define singleTrackModel_h

#include <random>
#include <chrono>

#define CAR_RADIUS          10      // Radius of circle to draw the car
#define AXLE_DISTANCE       30      // Axle distance (L) for single track model

#define MOVEMENT_ERROR      1.0     // movements are between 15 - 30

#define WIDTH               1280    // Window width
#define HEIGTH              700     // Window height
#define PI                  3.1415927

// This are the enviroment variables which the user can adjust with the trackbars
static int numParticles = 1000;
static int numObstacles = 30;
static int measurementDist = 150;
static int measurementNoise = 5;

// Seeded random number generator which we need in several functions
static std::default_random_engine generator((unsigned)std::chrono::system_clock::now().time_since_epoch().count());

// Transforms a point (as cv::Point of by x and y coordinate) to a point in the opencv coordinate system (cv::Mat).
// In the cv::Mat the point (0,0) is on the top left and y increases to the bottom.
// We need this for drawing our points (in world coordinates) to the image.
inline cv::Point worldToImage(double x, double y) { return cv::Point(x, HEIGTH - y); }
inline cv::Point worldToImage(cv::Point &worldPoint) { return worldToImage(worldPoint.x, worldPoint.y); }

// Returns the angle, but ensures it's between 0 and 2*PI
double normalizeAngle(double angle);

/*
 * The SingleTrackModel is the base for our car and the particles.
 * As usual it has a position (x and y) and an orientation in radians.
 * Besides the typical move function, it can measure its distance to all obstacles (in the given measurement distance).
 */
class SingleTrackModel {
protected:
    // Position, orientation and orientationVec are in world coordinates
    cv::Point position;
    double orientation;
    cv::Vec2d orientationVec;

    // Vector with the current measurements to the obstacles
    std::vector<float> measurements;

public:
    SingleTrackModel(cv::Point position_, double orientation_)
    : position(position_)
    , orientation(orientation_)
    { }

    inline cv::Point getPosition() const        { return position; }
    inline double getX() const                  { return position.x; }
    inline double getY() const                  { return position.y; }
    inline double getOrientation() const        { return orientation; }
    inline cv::Vec2d getOrientationVec() const  { return cv::Vec2d(AXLE_DISTANCE * cos(orientation), AXLE_DISTANCE * sin(orientation)); }

    virtual void move(double distance, double steeringAngle) = 0;
    void measure(std::vector<cv::Rect> *obstacles);
};

/*
 * Our car is a SingleTrackModel and also provides a function for returning its measurements as well as drawing and moving it.
 */
class Car : public SingleTrackModel {
public:
    using SingleTrackModel::SingleTrackModel; // inherit ctor

    inline std::vector<float> getMeasurements() const { return measurements; }
    
    void draw(cv::Mat &scene, std::vector<cv::Rect> *obstacles);
    void move(double distance, double steeringAngle);
};

/*
 * A particle is also based on the SingleTrackModel.
 * Further it has a weight and functions to calculate this weight and repositoning itself.
 */
class Particle : public SingleTrackModel {
    double weight;

public:
    using SingleTrackModel::SingleTrackModel; // inherit ctor

    inline double getWeigth() const { return weight; }
    
    void draw(cv::Mat &scene);
    void move(double distance, double steeringAngle);
    double calculateWeight(std::vector<cv::Rect> *obstacles, std::vector<float> &robotMeasurements);
    void reposition(double x, double y, double orientation);
};

#endif /* singleTrackModel_h */
