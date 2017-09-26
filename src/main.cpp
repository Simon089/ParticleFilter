//
//  main.cpp
//  ParticleFilter
//
//  Created by Simon Weigl on 04.07.17.
//  Copyright © 2017 Simon Weigl. All rights reserved.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <random>

#include "singleTrackModel.h"

// Parameters for the path ellipse
#define ELLIPSE_CENTER      cv::Point(WIDTH / 2, HEIGTH / 2)
#define ELLIPSE_WIDTH       WIDTH * 0.3
#define ELLIPSE_HEIGHT      HEIGTH * 0.2
#define ELLIPSE_ANGLE       0
#define ELLIPSE_ARC_START   0
#define ELLIPSE_ARC_END     360

// Width and height of an obstacle in pixel
#define OBSTACLE_DIMENSION  20

// Forwards declerations for our helper functions
void startNewSimulation();
void createPathAndTrack(std::vector<cv::Point> &path, std::vector<cv::Point> &track);
void createObstacles(std::vector<cv::Rect> &obstacles, std::vector<cv::Point> &track);
void drawObstacles(cv::Mat &scene, std::vector<cv::Rect> &obstacles);
void createParticles(std::vector<Particle> &particles, std::vector<cv::Rect> &obstacles);
size_t resampleParticles(std::vector<Particle> *particles, double sumWeights);
void drawInfoWindow(size_t relevantParticles);
static void onTrackbar(int, void*);

// Name and matrix for the information window
std::string infoWindowName = "Information";
cv::Mat infoWin;

/*
 * The main loop displays a window with some trackbars for adjusting some paramters.
 * Then it starts the application.
 * Loop continues until the user presses 'q'.
 */
int main(int argc, char** argv) {
    // We will return from this endless loop with a return after the user presses 'q'
    while (true) {
        std::string inputWindowName = "Inputs";
        cv::namedWindow(inputWindowName, CV_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
        cv::createTrackbar("Number Obstacles", inputWindowName, &numObstacles, 80, onTrackbar);
        cv::createTrackbar("Number Particles", inputWindowName, &numParticles, 5000, onTrackbar);
        cv::createTrackbar("Measurement Distance", inputWindowName, &measurementDist, 300, onTrackbar);
        cv::createTrackbar("Measurment Noise", inputWindowName, &measurementNoise, 20, onTrackbar);
        onTrackbar(0, 0);

        auto pressedKey = cvWaitKey(0);
        cv::destroyWindow(inputWindowName);

        // 'q' quits the program, everything else starts the app interactive
        switch (pressedKey) {
            case 'q':
                cv::destroyAllWindows();
                return 0;
                break;
            default:
                startNewSimulation();
                break;
        }
    }
}

/*
 * startNewSimulation sets up the enviroment for the particle filter and calls the functions for the single particle filter steps in a loop.
 */
void startNewSimulation() {
    // Create the window and the scene
    std::string windowName = "Particle filter";
    cv::Mat scene = cv::Mat(HEIGTH, WIDTH, CV_8UC3);
    cv::Mat enviromentScene;
    scene.setTo(cv::Scalar(255, 255, 255));

    // Some vectors where we store out path, the track, particles, and obstacles
    std::vector<cv::Point> plannedPath;
    std::vector<cv::Point> actuallPath;
    std::vector<cv::Point> track;
    std::vector<cv::Rect> obstacles;
    std::vector<Particle> particles;

    // Create path, track and obstacles
    createPathAndTrack(plannedPath, track);
    createObstacles(obstacles, track);

    // Draw obstacles and track
    drawObstacles(scene, obstacles);
    for(auto p : track) {
        cv::circle(scene, p, 1, cv::Scalar(125, 125));
    }

    // Store the basic setup, so we don't have to redraw it after every step
    enviromentScene = scene.clone();

    // Create and draw particles
    createParticles(particles, obstacles);
    for(auto p = particles.begin(); p != particles.end(); ++p) {
        p->draw(scene);
    }

    // Calculate orientation of car in radians
    double orientation = atan2(plannedPath.at(0).y - plannedPath.at(1).y, plannedPath.at(0).x - plannedPath.at(1).x);
    orientation -= PI; // We subtract PI here, because of the different y-axis in opencv
    orientation = normalizeAngle(orientation);

    // Create and draw car
    Car myCar(plannedPath.at(0), orientation);
    myCar.draw(scene, &obstacles);
    actuallPath.push_back(myCar.getPosition());

    // Show the start enviroment
    cv::imshow(windowName, scene);
    cv::moveWindow(windowName, 120, 0);
    cv::waitKey(0);

    // Main loop
    for (size_t i = 1; i < plannedPath.size(); i++) {
        // Set scene to saved enviroment-scene
        scene = enviromentScene.clone();

        // Get next planned position
        cv::Point currentGoal = plannedPath.at(i);
        cv::Point currentPosition = myCar.getPosition();

        // Calculate steering angle
        float steeringAngle = atan2(currentPosition.y - currentGoal.y, currentPosition.x - currentGoal.x);
        steeringAngle -= myCar.getOrientation(); // the cars orientation is already in our used (normal) coordinate system
        steeringAngle -= PI; // We subtract PI here, because of the different y-axis in opencv
        steeringAngle = normalizeAngle(steeringAngle); // We want our angle between 0 and 2*PI

        // Calculate distance to current goal
        double distance = cv::norm(currentGoal - currentPosition);

        // Move the car, save the position and measure distances to obstacles
        myCar.move(distance, steeringAngle);
        actuallPath.push_back(myCar.getPosition());
        myCar.measure(&obstacles);

        // For each particle: move, measure distances, calculate weight
        double sumWeights = 0.0; // We sum up all weights here, so we can normalize them in the resampling step
        auto robotMeasurements = myCar.getMeasurements();
        for(auto p = particles.begin(); p != particles.end(); ++p) {
            p->move(distance, steeringAngle);
            p->measure(&obstacles);
            sumWeights += p->calculateWeight(&obstacles, robotMeasurements);
        }

        // Resample particles
        auto relevantParticles = resampleParticles(&particles, sumWeights);

        // Draw on the enviroment-scene: droven path, particles, car
        for (size_t i = 1; i < actuallPath.size(); i++) {
            cv::line(scene, worldToImage(actuallPath.at(i - 1)), worldToImage(actuallPath.at(i)), cv::Scalar(255));
        }

        for(auto p = particles.begin(); p != particles.end(); ++p) {
            p->draw(scene);
        }

        myCar.draw(scene, &obstacles);

        // Write number of particles which were used to resample (weight of particle greater than 0)
//        std::stringstream ss;
//        ss << "Number of resampled particles: " << std::to_string(relevantParticles) << " / " << numParticles;
//        cv::putText(scene, ss.str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());

        drawInfoWindow(relevantParticles);

        // Show the image
        cv::imshow(windowName, scene);


        auto k = cv::waitKey(0);
        if(k == 'q') { // If user wants to quit
            cv::destroyWindow(windowName);
            return;
        }
    }

    cv::destroyWindow(windowName);
}

/*
 * Creates obstacles at random positions, but not on the track.
 *   OUT obstacles: vector of cv::Rect where obstacles are stored in
 *   IN  track:     vector of cv::Points which specify the track
 */
void createObstacles(std::vector<cv::Rect> &obstacles, std::vector<cv::Point> &track) {
    // Distributions for obstacle position
    std::uniform_int_distribution<int> xDistribution(0, WIDTH - OBSTACLE_DIMENSION);
    std::uniform_int_distribution<int> yDistribution(0, HEIGTH - OBSTACLE_DIMENSION);

    // Create the obstacles (cv::Rect)
    for(int i = 0; i < numObstacles;) {
        // Create a cv::Rect at a random position
        auto rect = cv::Rect(xDistribution(generator), yDistribution(generator), OBSTACLE_DIMENSION, OBSTACLE_DIMENSION);
        bool valid = true;
        // Check if the new obstacle would intersect with the track
        for(size_t j = 0; j < track.size() && valid; j++) {
            if(track.at(j).inside(rect))
                valid = false;
        }
        // If it doesn't intersect, add it to the output-vector
        if(valid) {
            obstacles.push_back(rect);
            i++;
        }
    }
}

/*
 * Draws all obstacles to on the scene.
 *  IN scene: Scene were obstacles should be drawn on.
 *  IN obstacles: The obstacles we want to draw.
 */
void drawObstacles(cv::Mat &scene, std::vector<cv::Rect> &obstacles) {
    for(auto r : obstacles) {
        cv::Point br = r.br();
        cv::Point tl = r.tl();
        cv::rectangle(scene, worldToImage(br), worldToImage(tl), cv::Scalar(125, 125, 125, 0.6), CV_FILLED);
    }
}

/*
 * Resamples all particles according to their weights.
 * The xxx method is used for this. todo
 *   IN/OUT particles: Our particles. Their positions and orientations will be resampled here.
 *   IN sumWeights: The sum of all weights from our particles. We need this for normalizing the weights.
 * Returns number of relevant particles (weight > 0)
 */
size_t resampleParticles(std::vector<Particle> *particles, double sumWeights) {
    double maxWeight = 0.0;
    size_t relevantParticles = 0; // A particle if relevant if its weights is greater zero
    // We save the normalized weights, the positions, and the orientations from all relevant particles here,
    // so we can change the positons and orientation of the existing particles one by one.
    // The i-th element of each of this vectors belongs to one (relevant) particle.
    std::vector<double> normalizedWeights;
    std::vector<double> xPositions;
    std::vector<double> yPositions;
    std::vector<double> orientations;

    // Normalize weights
    for(auto p = particles->begin(); p != particles->end(); ++p) {
        auto w = p->getWeigth();
        if(w > 0) { // Particle is relevant
            normalizedWeights.push_back(w / sumWeights);
            maxWeight = MAX(maxWeight, w / sumWeights);
            xPositions.push_back(p->getX());
            yPositions.push_back(p->getY());
            orientations.push_back(p->getOrientation());
        }
    }

    relevantParticles = normalizedWeights.size();
    if(relevantParticles == 0) {
        // All particles have weights zero, so we hope that at least one particle will become relevant later again, so our filter doesn't fail
        return 0;
    }

    // Resample particles according to their weights.
    // The method used here is a "resampling wheel as described here: https://www.youtube.com/watch?v=wNQVo6uOgYA
    std::uniform_real_distribution<double> betaDistribution(0.0, 2.0 * maxWeight);
    std::uniform_int_distribution<size_t> indexDistribution(0, relevantParticles - 1);
    std::normal_distribution<double> orientationDistribution(1.0, .1);

    size_t index = indexDistribution(generator);
    double beta = 0.0;
    for (int i = 0; i < numParticles; i++) {
        beta += betaDistribution(generator);
        while (beta > normalizedWeights.at(index)) {
            beta -= normalizedWeights.at(index);
            index = (index + 1) % relevantParticles;
        }
        particles->at(i).reposition(xPositions.at(index),
                                    yPositions.at(index),
                                    orientations.at(index) * orientationDistribution(generator));
    }

    return relevantParticles;
}

/*
 * This creates the track and the path for the car.
 * Basicaly these are three ellipses with a slightly different width and height.
 * We need the track, so we can later ensure that no obstacle is on the path, when we create them.
 *  OUT path: The points of our cars path will be inserted here.
 *  OUT track: This vector will contain the points of the two track-ellipses.
 */
void createPathAndTrack(std::vector<cv::Point> &path, std::vector<cv::Point> &track) {
    // Path
    cv::ellipse2Poly(ELLIPSE_CENTER, cv::Size(ELLIPSE_WIDTH, ELLIPSE_HEIGHT), ELLIPSE_ANGLE, ELLIPSE_ARC_START, ELLIPSE_ARC_END, 5, path);

    // Track
    std::vector<cv::Point> tmp;
    cv::ellipse2Poly(ELLIPSE_CENTER, cv::Size(ELLIPSE_WIDTH - CAR_RADIUS, ELLIPSE_HEIGHT - CAR_RADIUS), ELLIPSE_ANGLE, ELLIPSE_ARC_START, ELLIPSE_ARC_END, 3, track);
    cv::ellipse2Poly(ELLIPSE_CENTER, cv::Size(ELLIPSE_WIDTH + CAR_RADIUS, ELLIPSE_HEIGHT + CAR_RADIUS), ELLIPSE_ANGLE, ELLIPSE_ARC_START, ELLIPSE_ARC_END, 3, tmp);

    track.insert(track.end(), tmp.begin(), tmp.end());
}

/*
 * Creates our particles at random positions on the map, but not in obstacles.
 *  OUT particles: The creates particles.
 *  IN obstacles: We need our obstacles here, so we don't place the particles inside them.
 */
void createParticles(std::vector<Particle> &particles, std::vector<cv::Rect> &obstacles) {
    // Distributions
    std::uniform_int_distribution<int> xDistribution(0, WIDTH);
    std::uniform_int_distribution<int> yDistribution(0, HEIGTH);
    std::uniform_real_distribution<double> orientationDist(0.0, 2.0 * PI);

    for(int i = 0; i < numParticles;) {
        // Create a random points in the map
        auto p = cv::Point(xDistribution(generator), yDistribution(generator));
        bool valid = true;
        // For each obstacle test if the point is inside it
        for(size_t j = 0; j < obstacles.size() && valid; j++) {
            if(p.inside(obstacles.at(j)))
                valid = false;
        }
        // If the point is not inside any obstacle, create a particle at this point
        if(valid) {
            particles.push_back(Particle(p, orientationDist(generator)));
            i++;
        }
    }
}

/*
 * This function writes some information onto the information window.
 *   IN relevantParticles: Number of relevant particles from the current resampling step.
 */
void drawInfoWindow(size_t relevantParticles) {
    infoWin.setTo(cv::Scalar(255, 255, 255));

    // Write enviroment information on image, so the user doesn't forget the input
    std::stringstream ss;
    ss << "Number of resampled particles: " << std::to_string(relevantParticles) << " / " << numParticles;
    cv::putText(infoWin, ss.str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    ss.str("");
    ss << "Number of Obstacles: " << std::to_string(numObstacles);
    cv::putText(infoWin, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    ss.str("");
    ss << "Measurement Distance: " << std::to_string(measurementDist);
    cv::putText(infoWin, ss.str(), cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    ss.str("");
    ss << "Measurement Noise: " << std::to_string(measurementNoise);
    cv::putText(infoWin, ss.str(), cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    cv::putText(infoWin,
                "Press 'q' to quit this simulation, anything else for a step.",
                cv::Point(20, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());

    cv::imshow(infoWindowName, infoWin);
}

/*
 * This callback function displays the current selected enviroment variables in the information window.
 */
static void onTrackbar(int, void*) {
    // Write some instruction on the information window
    infoWin = cv::Mat(120, WIDTH, CV_8UC3);
    infoWin.setTo(cv::Scalar(255, 255, 255));

    cv::putText(infoWin,
                "Set the enviroment variables. Press 'q' for quit, anything else to start the simulation with the given input.",
                cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    std::stringstream ss;
    ss << "Measurement Noise: " << std::to_string(measurementNoise);
    cv::putText(infoWin, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    ss.str("");
    ss << "Measurement Distance: " << std::to_string(measurementDist);
    cv::putText(infoWin, ss.str(), cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    ss.str("");
    ss << "Number of Particles: " << std::to_string(numParticles);
    cv::putText(infoWin, ss.str(), cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());
    ss.str("");
    ss << "Number of Obstacles: " << std::to_string(numObstacles);
    cv::putText(infoWin, ss.str(), cv::Point(20, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar());

    cv::imshow(infoWindowName, infoWin);
    cv::moveWindow(infoWindowName, 100, HEIGTH);
}
