#include "track.h"
#include <cmath>

Track::Track() : kf_(7, 4) {
    kf_.F_ <<
            1, 0, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 1;
    
    kf_.P_ <<
            10, 0, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0, 0,
            0, 0, 0, 10, 0, 0, 0,
            0, 0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0, 10000;
    
    kf_.Q_ <<
            1, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0.5, 0, 0,
            0, 0, 0, 0, 0, 0.5, 0,
            0, 0, 0, 0, 0, 0, 0.25;
    
    kf_.H_ <<
            1, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0;
    
    kf_.R_ <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 10, 0,
            0, 0, 0, 10;
}

void Track::Predict() {
    kf_.Predict();
    if (coast_cycles_ > 0) {
        hit_streak_ = 0;
    }
    coast_cycles_++;
}


extern "C" void Track::Update(const box& bbox) {
        coast_cycles_ = 0;
        hit_streak_++;
        Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
        kf_.Update(observation);
}

extern "C" void Track::Init(const box& bbox) {
    kf_.x_.head(4) << ConvertBboxToObservation(bbox);
    hit_streak_++;
}

extern "C" box Track::GetStateAsBbox() const {
        return ConvertStateToBbox(kf_.x_);
}

float Track::GetNIS() const {
        return kf_.NIS_;
}

// bbox [cx, cy, w, h] .. floating type & yolo_format -> Eigen [cx ]
extern "C" Eigen::VectorXd Track::ConvertBboxToObservation(const box& bbox) const {
        Eigen::VectorXd observation = Eigen::VectorXd::Zero(4);
        float center_x = bbox.x;
        float center_y = bbox.y;
        auto areas = static_cast<float>(bbox.w * bbox.h);
        auto ratio = static_cast<float>(bbox.h / bbox.w);
        observation << center_x, center_y, areas, ratio;
        return observation;  
}


// cx, cy, s, r, u_dot, v_dot, s_dot (Eigen) -> box
extern "C" box Track::ConvertStateToBbox(const Eigen::VectorXd &state) const{
        box bbox;
        bbox.x = max(0.0, state[0]);
        bbox.y = max(0.0, state[1]);
        bbox.w = max(0.0, sqrt(state[2] * state[3])); // w
        bbox.h = max(0.0, sqrt(state[2] / state[3])); // h

        return bbox;
}