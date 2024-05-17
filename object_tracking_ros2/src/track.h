#include "kalman_filter.h"
#include "object_tracking_node.hpp"

using namespace std;


class Track {
public:
    Track();
    ~Track() = default;
    void Init(const box& bbox);
    void Predict();
    void Update(const box& bbox);
    
    box GetStateAsBbox() const;
    float GetNIS() const;

    int coast_cycles_ = 0;
    int hit_streak_ = 0;
    Eigen::VectorXd ConvertBboxToObservation(const box& bbox) const;
    box ConvertStateToBbox(const Eigen::VectorXd &state) const;
    KalmanFilter kf_;
};