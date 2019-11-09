#include "ekfslam.h"
#include <iostream>

// Default Constructor
EKFSLAM::EKFSLAM(){}

// Overloaded Constructor
EKFSLAM::EKFSLAM(unsigned int landmark_size, unsigned int robot_pose_size, 
                 float _motion_noise){
    // 3 * 1
    this->robotSigma = Eigen::MatrixXd(robot_pose_size, 1);
    // 3 * 2N
    this->robMapSigma = Eigen::MatrixXd(robot_pose_size, 2 * landmark_size);
    // 2N * 2N
    this->mapSigma = Eigen::MatrixXd(2 * landmark_size, 2 * landmark_size);
    // (3 + 2N) * (3 + 2N)
    this->Sigma = Eigen::MatrixXd(robot_pose_size + 2 * landmark_size, 
                                  robot_pose_size + 2 * landmark_size);
    // (3 + 2N)
    this->mu = Eigen::VectorXd(robot_pose_size + 2 * landmark_size);
    // (3 + 2N) * (3 + 2N)
    this->R = Eigen::MatrixXd(robot_pose_size + 2 * landmark_size, 
                              robot_pose_size + 2 * landmark_size);
    // (3 + 2N) * (3 + 2N)
    this->Q = Eigen::MatrixXd(robot_pose_size + 2 * landmark_size, 
                              robot_pose_size + 2 * landmark_size);
    
    // Init False * N
    for (unsigned int i = 0; i < landmark_size; i++)
        this->observedLandmarks.push_back(false);
}

EKFSLAM::~EKFSLAM(){}

/****** TODO *********/
// Description: Prediction step for the EKF based off an odometry model
// Inputs:
// motion - struct with the control input for one time step
void EKFSLAM::Prediction(const OdoReading& motion){

}

/****** TODO *********/
// Description: Correction step for EKF
// Inputs:
// observation - vector containing all observed landmarks from a laser scanner
void EKFSLAM::Correction(const vector<LaserReading>& observation){

}