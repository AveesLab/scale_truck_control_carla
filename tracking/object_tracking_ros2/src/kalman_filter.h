#pragma once

#include "eigen3/Eigen/Dense"

using namespace std;

class KalmanFilter{
public:
    explicit KalmanFilter(unsigned int num_states, unsigned int num_obs);
    
    virtual ~KalmanFilter() = default;
    
    virtual void Coast();
    void Predict();

    virtual Eigen::VectorXd PredictionToObservation(const Eigen::VectorXd &state);
    virtual void Update(const Eigen::VectorXd &z);

    float CalculateLogLikelihood(const Eigen::VectorXd& y, const Eigen::VectorXd& S);
    Eigen::VectorXd x_, x_predict_;
    Eigen::MatrixXd P_, P_predict_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    
    unsigned int num_states_, num_obs_;
    float log_likelihood_delta;
    float NIS_;
};