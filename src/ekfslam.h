#ifndef EKFSLAM_H
#define EKFSLAM_H
#include <vector>
#include "../include/sensor_info.h"
#include "../include/common.h"
#include "../include/Eigen/Dense"
#define INF 1000

class EKFSLAM {
 private:
    // Covariance Matrix for robot state variables
    Eigen::MatrixXd robotSigma;
    // Covariance Matrix for robot to landmarks
    Eigen::MatrixXd robMapSigma;
    // Covariances of landmark positions wrt to each other
    Eigen::MatrixXd mapSigma;
    // Full Covariance Matrix
    Eigen::MatrixXd Sigma;
    // Full State Vector
    Eigen::VectorXd mu;
    // Noise Matrix due to motion
    Eigen::MatrixXd R;
    // Noise Matrix due to sensors
    Eigen::MatrixXd Q;
    // Vector of observed landmarks
    vector<bool> observedLandmarks;

    int lm_size;

 public:
    // Default Constructor
    EKFSLAM();

    /****** TODO *********/
    // Overloaded Constructor
    // Inputs:
    // landmark_size - number of landmarks on the map
    // robot_pose_size - number of state variables to track on the robot
    // motion_noise - amount of noise to add due to motion
    EKFSLAM(unsigned int landmark_size,
        unsigned int robot_pose_size = 3,
        float _motion_noise = 0.1);

    // Standard Destructor
    ~EKFSLAM();

    float AngleNormalization(float angle);
    void SetState(float x, float y, float theta);
    void SetLandmark(int id, float x, float y);
    void SetRobotSigma(float r1, float t, float theta);
    void SetRobotMapSigma(const LaserReading& obs, float theta);
    void SetMapSigma();
    void SetSigma();
    /****** TODO *********/
    // Description: Prediction step for the EKF based off an odometry model
    // Inputs:
    // motion - struct with the control input for one time step
    void Prediction(const OdoReading& motion);

    Eigen::MatrixXd GetMtxz(float range, float bearing, float q, float x_sigma, float y_sigma, float theta);
    Eigen::MatrixXd GetMtxF(uint64_t obs_id);
    Eigen::MatrixXd GetMtxh(float q, float x_sigma, float y_sigma);
    Eigen::MatrixXd GetMtxH(float q, Eigen::MatrixXd F, Eigen::MatrixXd h);
    Eigen::MatrixXd GetKalmanGain(Eigen::MatrixXd H);
    /****** TODO *********/
    // Description: Correction step for EKF
    // Inputs:
    // observation - vector containing all observed landmarks from a laser scanner
    void Correction(const vector<LaserReading>& observation);

    VectorXd getMu() const {
        return mu;
    }

    MatrixXd getSigma() const {
        return Sigma;
    }
};

#endif  // EKFSLAM_H
