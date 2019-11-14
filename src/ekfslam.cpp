#include "ekfslam.h"
#include <math.h> 
#include <iostream>

// Default Constructor
EKFSLAM::EKFSLAM(){}

// Overloaded Constructor
EKFSLAM::EKFSLAM(unsigned int landmark_size, unsigned int robot_pose_size, 
                 float _motion_noise){
    cout << "##### Init EFKSLAM #####" << endl;
    cout << "Landmark Size :" << landmark_size
         << " Robot Pose Size : " << robot_pose_size
         << " Motion noise: " << _motion_noise << endl;
    
    // 3 * 1
    this->robotSigma = Eigen::MatrixXd::Zero(robot_pose_size, 1);
    // 3 * 2N
    this->robMapSigma = Eigen::MatrixXd::Zero(robot_pose_size, 2 * landmark_size);
    // 2N * 2N
    this->mapSigma = Eigen::MatrixXd::Zero(2 * landmark_size, 2 * landmark_size);
    // (3 + 2N) * (3 + 2N)
    this->Sigma = Eigen::MatrixXd::Zero(robot_pose_size + 2 * landmark_size, 
                                        robot_pose_size + 2 * landmark_size);
    // (3 + 2N)
    this->mu = Eigen::VectorXd::Zero(robot_pose_size + 2 * landmark_size);
    // (3 + 2N) * (3 + 2N)
    this->R = Eigen::MatrixXd::Zero(robot_pose_size + 2 * landmark_size, 
                                    robot_pose_size + 2 * landmark_size);
    // (3 + 2N) * (3 + 2N)
    this->Q = Eigen::MatrixXd::Zero(robot_pose_size + 2 * landmark_size, 
                                    robot_pose_size + 2 * landmark_size);
    
    // Init False * N
    for (unsigned int i = 0; i < landmark_size; i++)
        this->observedLandmarks.push_back(false);
}

EKFSLAM::~EKFSLAM(){}


void EKFSLAM::SetPredictionRobotSigma(float r1, float t, float r2) {
    // Robot State at Predicted
    float x_pred, y_pred, theta_pred;
    // Current Robot State
    float x = this->mu(0);
    float y = this->mu(1);;
    float theta = this->mu(2);;

    theta_pred = r1 + r2 + theta;
    x_pred = t * sin(r1 + theta) + x;
    y_pred = t * cos(r1 + theta) + y;

    this->robotSigma = Eigen::MatrixXd::Identity(3, 3);
    this->robotSigma(0, 2) += t * cos(r1 + theta);
    this->robotSigma(1, 2) += t * (-1) * sin(r1 + theta);

    // FIXME: Better way to do this
    this->mu(0) = x_pred;
    this->mu(1) = y_pred;
    this->mu(2) = theta_pred;
}

// Normalization Angle btw -PI ~ PI
float EKFSLAM::AngleNormalization(float angle) {    
    while (angle > M_PI) {
        cout << "Found Big Angle " << angle << endl;
        angle -= 2 * M_PI;
    }

    while (angle < -1 * M_PI) {
        cout << "Found Small Angle " << angle << endl;
        angle += 2 * M_PI;
    }

    return angle;
}
/****** TODO *********/
// Description: Prediction step for the EKF based off an odometry model
// Inputs:
// motion - struct with the control input for one time step
void EKFSLAM::Prediction(const OdoReading& motion){
    cout << "##### Prediction #####" << endl;
    float r1 = motion.r1;
    float r2 = motion.r2;
    float t = motion.t;
    r1 = AngleNormalization(r1);
    r2 = AngleNormalization(r2);

    this->SetPredictionRobotSigma(r1, t, r2);
    
    // FIXME: Delete this
    // cout << "Robot Sigma" << endl;
    // cout << this->robotSigma << endl;
    cout << "State - " << this->mu << endl;
}

Eigen::MatrixXd EKFSLAM::GetMtxz(float q, float x_sigma, float y_sigma, float theta) {
    Eigen::MatrixXd z(1,2);
    z(0,0) = sqrt(q);
    z(0,1) = atan2(y_sigma, x_sigma) - theta;
    return z;
}

Eigen::MatrixXd EKFSLAM::GetMtxF(uint64_t obs_id) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(5, 2 * this->observedLandmarks.size() + 3);
    F(0,0) = 1;
    F(1,1) = 1;
    F(2,2) = 1;
    F(3,obs_id * 2 + 3) = 1;
    F(4,obs_id * 2 + 4) = 1;
    return F;
}

Eigen::MatrixXd EKFSLAM::GetMtxh(float q, float x_sigma, float y_sigma) {
    Eigen::MatrixXd h(2, 5);
    h(0,0) = -sqrt(q) * x_sigma;
    h(0,1) = -sqrt(q) * y_sigma;
    h(0,2) = 0;
    h(0,3) = sqrt(q) * x_sigma;
    h(0,4) = sqrt(q) * y_sigma;
    h(1,0) = y_sigma;
    h(1,1) = -x_sigma;
    h(1,2) = -q;
    h(1,3) = -y_sigma;
    h(1,4) = x_sigma;
    return h;
}

Eigen::MatrixXd EKFSLAM::GetMtxH(float q, Eigen::MatrixXd F, Eigen::MatrixXd h) {
    return (1 / q) * h * F;
}

Eigen::MatrixXd EKFSLAM::GetKalmanGain(Eigen::MatrixXd H) {
    return this->Sigma * H.transpose() * (H * this->Sigma * H.transpose() + this->Q).inverse();
}

/****** TODO *********/
// Description: Correction step for EKF
// Inputs:
// observation - vector containing all observed landmarks from a laser scanner
void EKFSLAM::Correction(const vector<LaserReading>& observation){
    cout << "##### Correction #####" << endl;
    float x = this->mu(0);
    float y = this->mu(1);
    float theta = this->mu(2);
    float obs_x_pred, obs_y_pred, x_sigma, y_sigma, q;
    Eigen::VectorXd mu_copy = this->mu;
    Eigen::MatrixXd cov_copy = this->mapSigma;


    for (LaserReading obs: observation)
    {
        // cout << "ID: " << obs.id 
        //      << " Range: " << obs.range
        //      << " Bearing: " << obs.bearing << endl;

        // First Time observed => set by observation
        if (this->observedLandmarks.at(obs.id) == false)
        {
            mu_copy(obs.id * 2 + 3) = x + obs.range * cos(obs.bearing + theta);
            mu_copy(obs.id * 2 + 4) = y + obs.range * sin(obs.bearing + theta);
        }

        obs_x_pred = mu_copy(obs.id * 2 + 3);
        obs_y_pred = mu_copy(obs.id * 2 + 4);
        x_sigma = obs_x_pred - x;
        y_sigma = obs_y_pred - y;
        q = pow(x_sigma, 2) + pow(y_sigma, 2);

        Eigen::MatrixXd z(1,2);
        z(0,0) = obs.range;
        z(0,1) = obs.bearing;
        Eigen::MatrixXd z_pred = GetMtxz(q, x_sigma, y_sigma, theta);
        Eigen::MatrixXd F = GetMtxF(obs.id);
        Eigen::MatrixXd h = GetMtxh(q, x_sigma, y_sigma);
        Eigen::MatrixXd H = GetMtxH(q, F, h);
        Eigen::MatrixXd K = GetKalmanGain(H);
        
        mu_copy = mu_copy + K * (z - z_pred);
        cov_copy = cov_copy - K * H * cov_copy;
    }

    this->mu = mu_copy;
    this->Sigma = cov_copy;
}