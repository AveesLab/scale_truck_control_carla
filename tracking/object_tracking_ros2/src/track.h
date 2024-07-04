#pragma once

#include "kalman_filter.h"
#include "type.h"
using namespace std;


class Track {
public:
    Track();
    ~Track() = default;
    void Init(const object_info& obj);
    void Predict();
    void Update(const object_info& obj);
    
    box GetStateAsBbox() const;
    float GetNIS() const;

    int coast_cycles_ = 0;
    int hit_streak_ = 0;
    Eigen::VectorXd ConvertBboxToObservation(const box& bbox) const;
    box ConvertStateToBbox(const Eigen::VectorXd &state) const;
    KalmanFilter kf_;
    int class_id;
    float distance, velocity;

};