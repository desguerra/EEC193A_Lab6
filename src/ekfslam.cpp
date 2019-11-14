#include "ekfslam.h"
#include <math.h>
#include <iostream>

// Default Constructor
EKFSLAM::EKFSLAM() {}

// Overloaded Constructor
EKFSLAM::EKFSLAM(unsigned int landmark_size, unsigned int robot_pose_size,
                 float _motion_noise)
{
    cout << "##### Init EFKSLAM #####" << endl;
    cout << "Landmark Size :" << landmark_size
         << " Robot Pose Size : " << robot_pose_size
         << " Motion noise: " << _motion_noise << endl;

    // 3 * 3
    this->robotSigma = Eigen::MatrixXd::Zero(robot_pose_size, robot_pose_size);
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
    this->Q = Eigen::MatrixXd::Zero(2, 2);
    this->Q(0, 0) = 1;
    this->Q(1, 1) = 1;

    // Init False * N
    for (unsigned int i = 0; i < landmark_size; i++)
        this->observedLandmarks.push_back(false);
    
    this->lm_size = this->observedLandmarks.size();
}

EKFSLAM::~EKFSLAM() {}


// Description: Normalization Angle btw -PI ~ PI
// Inputs:
// angle := float -> raw angle reading from sensor_info.h
float EKFSLAM::AngleNormalization(float angle)
{
    while (angle > M_PI)
    {
        cout << "Found Big Angle " << angle << endl;
        angle -= 2 * M_PI;
    }

    while (angle < -1 * M_PI)
    {
        cout << "Found Small Angle " << angle << endl;
        angle += 2 * M_PI;
    }

    return angle;
}

// Description: Set x,y,theta, also known as the robot state
// Inputs:
// x := float -> new x
// y := float -> new y
// theta := float -> new theta
void EKFSLAM::SetState(float x, float y, float theta) {
    this->mu(0) = x;
    this->mu(1) = y;
    this->mu(2) = theta;
}

// Description: Set x,y location of a given landmark
// Inputs:
// id := int -> landmark id
// x := float -> new x
// y := float -> new y
void EKFSLAM::SetLandmark(int id, float x, float y) {
    this->mu(id * 2 + 3) = x;
    this->mu(id * 2 + 4) = y;
}

// Description: Set robotSigma (dim: 3 x 3)
// Inputs:
// r1 := int -> rotaion angle 1
// t := float -> translation
// theta := float -> current theta
void EKFSLAM::SetRobotSigma(float r1, float t, float theta)
{
    this->robotSigma = Eigen::MatrixXd::Identity(3, 3);
    this->robotSigma(0, 2) += (-1) * t * sin(AngleNormalization(r1 + theta));
    this->robotSigma(1, 2) += t * cos(AngleNormalization(r1 + theta));
}

// Description: Set robMapSigma (dim: 3 x 2N) for given landmark
// Inputs:
// obs := const LaserReading ref -> information of sensor observation
// theta := float -> current theta
void EKFSLAM::SetRobotMapSigma(const LaserReading& obs, float theta) {
    // dlx / dx
    this->robMapSigma(0, obs.id * 2) = 1;
    // dlx / dy
    this->robMapSigma(1, obs.id * 2) = 0;
    // dlx / dtheta 
    this->robMapSigma(2, obs.id * 2) = (-1) * obs.range * sin(AngleNormalization(obs.bearing + theta));

    // dly / dx
    this->robMapSigma(0, obs.id * 2 + 1) = 0;
    // dly / dy
    this->robMapSigma(1, obs.id * 2 + 1) = 1;
    // dly / dtheta
    this->robMapSigma(2, obs.id * 2 + 1) = obs.range * cos(AngleNormalization(obs.bearing + theta));
}

// Description: Set mapSigma (dim: 2N x 2N)
// Inputs:
// obs := const LaserReading ref -> information of sensor observation
// theta := float -> current theta
void EKFSLAM::SetMapSigma() {
    this->mapSigma = Eigen::MatrixXd::Identity(this->lm_size * 2, this->lm_size * 2);
}

void EKFSLAM::SetSigma() {
    // dx / dx
    this->Sigma.topLeftCorner(3, 3) = this->robotSigma;
    // dx / dlm
    this->Sigma.topRightCorner(3, this->lm_size * 2) = this->robMapSigma;
    // dlm / dx
    this->Sigma.bottomRightCorner(this->lm_size * 2, 3) = this->robMapSigma.transpose();
    // dlm / dlm
    this->Sigma.bottomLeftCorner(this->lm_size * 2 + 3, this->lm_size * 2 + 3) = this->mapSigma;
}
/****** TODO *********/
// Description: Prediction step for the EKF based off an odometry model
// Inputs:
// motion - struct with the control input for one time step
void EKFSLAM::Prediction(const OdoReading &motion)
{
    cout << "##### Prediction #####" << endl;
    // Reading of Odemotry Model
    float r1 = motion.r1;
    float r2 = motion.r2;
    float t = motion.t;
    r1 = AngleNormalization(r1);
    r2 = AngleNormalization(r2);

    // Current Robot State
    float x = this->mu(0);
    float y = this->mu(1);
    float theta = this->mu(2);
    
    // Predicted Robot State
    float x_pred = t * cos(AngleNormalization(r1 + theta)) + x;
    float y_pred = t * sin(AngleNormalization(r1 + theta)) + y;
    float theta_pred = AngleNormalization(r1 + r2 + theta);


    SetRobotSigma(r1, t, theta);
    SetState(x_pred, y_pred, theta_pred);

    // FIXME: Delete this
    cout << this->mu << endl;
}

Eigen::MatrixXd EKFSLAM::GetMtxz(float range, float bearing, float q, float x_sigma, float y_sigma, float theta)
{   
    Eigen::MatrixXd z(2, 1);
    z(0, 0) = range;
    z(1, 0) = bearing;

    Eigen::MatrixXd z_pred(2, 1);
    z_pred(0, 0) = sqrt(q);
    z_pred(1, 0) = AngleNormalization(atan2(y_sigma, x_sigma) - theta);

    z = z - z_pred;
    z(1, 0) = AngleNormalization(z(1, 0));
    return z;
}

Eigen::MatrixXd EKFSLAM::GetMtxF(uint64_t obs_id)
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(5, 2 * this->lm_size + 3);
    F(0, 0) = 1;
    F(1, 1) = 1;
    F(2, 2) = 1;
    F(3, (obs_id - 1) * 2 + 3) = 1;
    F(4, (obs_id - 1) * 2 + 4) = 1;
    return F;
}

Eigen::MatrixXd EKFSLAM::GetMtxh(float q, float x_sigma, float y_sigma)
{
    Eigen::MatrixXd h(2, 5);
    h(0, 0) = (-1) * sqrt(q) * x_sigma;
    h(0, 1) = (-1) * sqrt(q) * y_sigma;
    h(0, 2) = 0;
    h(0, 3) = sqrt(q) * x_sigma;
    h(0, 4) = sqrt(q) * y_sigma;
    h(1, 0) = y_sigma;
    h(1, 1) = (-1) * x_sigma;
    h(1, 2) = (-1) * q;
    h(1, 3) = (-1) * y_sigma;
    h(1, 4) = x_sigma;
    return h;
}

Eigen::MatrixXd EKFSLAM::GetMtxH(float q, Eigen::MatrixXd F, Eigen::MatrixXd h)
{
    return (1 / q) * h * F;
}

Eigen::MatrixXd EKFSLAM::GetKalmanGain(Eigen::MatrixXd H)
{
    Eigen::Matrix2d tmp(2, 2); // Error with inverse of MatrixXd
    tmp = H * this->Sigma * H.transpose() + this->Q;
    return this->Sigma * H.transpose() * tmp.inverse();
}


/****** TODO *********/
// Description: Correction step for EKF
// Inputs:
// observation - vector containing all observed landmarks from a laser scanner
void EKFSLAM::Correction(const vector<LaserReading> &observation)
{
    cout << "##### Correction #####" << endl;
    float x = this->mu(0);
    float y = this->mu(1);
    float theta = this->mu(2);
    float obs_x_pred, obs_y_pred, x_sigma, y_sigma, q;
    Eigen::VectorXd mu_copy = this->mu;
    Eigen::MatrixXd cov_copy = this->Sigma;

    for (LaserReading obs : observation)
    {
        cout << "ID: " << obs.id
             << " Range: " << obs.range
             << " Bearing: " << obs.bearing << endl;

        // First Time observed => set by observation
        if (this->observedLandmarks.at(obs.id - 1) == false)
        {
            mu_copy((obs.id - 1) * 2 + 3) = x + obs.range * cos(AngleNormalization(obs.bearing + theta));
            mu_copy((obs.id - 1) * 2 + 4) = y + obs.range * sin(AngleNormalization(obs.bearing + theta));
            this->observedLandmarks.at(obs.id - 1) = true;
        }

        obs_x_pred = mu_copy((obs.id - 1) * 2 + 3);
        obs_y_pred = mu_copy((obs.id - 1) * 2 + 4);
        x_sigma = obs_x_pred - x;
        y_sigma = obs_y_pred - y;
        q = pow(x_sigma, 2) + pow(y_sigma, 2);

        Eigen::MatrixXd z = GetMtxz(obs.range, obs.bearing, q, x_sigma, y_sigma, theta);
        Eigen::MatrixXd F = GetMtxF(obs.id);
        Eigen::MatrixXd h = GetMtxh(q, x_sigma, y_sigma);
        Eigen::MatrixXd H = GetMtxH(q, F, h);
        Eigen::MatrixXd K = GetKalmanGain(H);

        mu_copy = mu_copy + K * z;
        cov_copy = cov_copy - K * H * cov_copy;
    }
    this->mu = mu_copy;
    this->Sigma = cov_copy;

    // FIXME: Delete this
    cout << this->mu << endl;
    cout << this->Sigma << endl;
}