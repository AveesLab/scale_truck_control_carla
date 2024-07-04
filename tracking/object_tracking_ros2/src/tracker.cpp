#include "tracker.h"


// Tracker tracker1;

Tracker::Tracker() {
    id_ = 0;
    std::map<int, Track> tracks_;
}


void Tracker::HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                                size_t nrows, size_t ncols,
                                std::vector<std::vector<float>>& association) {
                                    
    Matrix<float> matrix(nrows, ncols);
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            if (iou_matrix[i][j] != 0) {
                matrix(i, j) = -iou_matrix[i][j];
            }
            else {
                matrix(i, j) = 1.0f;
            }
        }
    }
    Munkres<float> m;
    m.solve(matrix);
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}


float Tracker::CalculateIoU(const object_info& obj, const Track& track) {
    auto trk = track.GetStateAsBbox();

    // cout << bbox.x << ", " << trk.x << endl;
    float bbox_x1 = obj.x - obj.w / 2;
    float bbox_x2 = obj.x + obj.w / 2;
    float bbox_y1 = obj.y - obj.h / 2;
    float bbox_y2 = obj.y + obj.h / 2;
    float trk_x1 = trk.x - trk.w / 2;
    float trk_x2 = trk.x + trk.w / 2;
    float trk_y1 = trk.y - trk.h / 2;
    float trk_y2 = trk.y + trk.h / 2;
    // cout << "bbox : " << bbox_x1 << ", " << bbox_x2 << ", " << bbox_y1 << ", " << bbox_y2 << endl;
    // cout << "trk : " << trk_x1 << ", " << trk_x2 << ", " << trk_y1 << ", " << trk_y2 << endl;
    auto xx1 = std::max(bbox_x1, trk_x1);
    if (xx1 < 0.0f) xx1 = 0.0f;
    auto yy1 = std::max(bbox_y1, trk_y1);
    if (yy1 < 0.0f) yy1 = 0.0f;
    auto xx2 = std::min(bbox_x2, trk_x2);
    if (xx2 >= 1.0f) xx2 = 1.0f;
    auto yy2 = std::min(bbox_y2, trk_y2);
    if (yy2 >= 1.0f) yy2 = 1.0f;
    // cout << xx1 << ", " << xx2 << ", " << yy1 << ", " << yy2 << endl;
    auto w = std::max(0.0f, xx2 - xx1);
    auto h = std::max(0.0f, yy2 - yy1);
    // cout << w << h << endl;
    float det_area = obj.w * obj.h;
    float trk_area = trk.w * trk.h;
    auto intersection_area = w * h;
    float union_area = det_area + trk_area - intersection_area;
    auto iou = intersection_area / union_area;
    // cout << "intersection_area : " << intersection_area << endl;
    // cout << "union_area : " << union_area << endl;
    // cout << "iou : " << iou << endl;
    // if (iou <= -2.0f) iou = (-0.55 * iou) / 2;
    return iou;
}

// typedef struct object_info {
//     float x, y, w, h, distance, velocity;
//     int class_id;
// } object_info;


// detected_object -> class_id, bbox, velocity, distance
void Tracker::AssociateDetectionsToTrackers(const vector<object_info>& detected_object,
                                            std::map<int, Track>& tracks,
                                            std::map<int, object_info>& matched,
                                            std::vector<object_info>& unmatched_det,
                                            float iou_threshold) {
    
    // class_id, bbox, velocity, distance
    if (tracks.empty()) {
        for (const auto& det : detected_object) {
            unmatched_det.push_back(det);
        }
        return;
    }
    std::vector<std::vector<float>> iou_matrix;
    iou_matrix.resize(detected_object.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    association.resize(detected_object.size(), std::vector<float>(tracks.size()));

    for (size_t i = 0; i < detected_object.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            iou_matrix[i][j] = CalculateIoU(detected_object[i], trk.second);
            j++;
        }
    }

    HungarianMatching(iou_matrix, detected_object.size(), tracks.size(), association);

    for (size_t i = 0; i < detected_object.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            // cout << "check_point 1" << endl;
            if (0 == association[i][j]) {
                // cout << "check_point 2" << endl;
                // cout << "iou_matrix[i][j] : " << iou_matrix[i][j] << endl; // -> 0
                if (iou_matrix[i][j] >= iou_threshold) {
                    // cout << "iou_point" << endl;
                    matched[trk.first] = detected_object[i];
                    matched_flag = true;
                }
                break;
            }
            j++;
        }
        if (!matched_flag) {
            // cout << "check_point 3" << endl;
            unmatched_det.push_back(detected_object[i]);
        }
    }
}



void Tracker::Run(const vector<object_info>& detected_object) {
    for (auto &track : tracks_) {
        track.second.Predict();
    }
    std::map<int, object_info> matched;
    std::vector<object_info> unmatched_det;

    if (!detected_object.empty()) {
        AssociateDetectionsToTrackers(detected_object, tracks_, matched, unmatched_det);
    }

    for (const auto &match : matched) {
        // cout << "Matchend!" << endl;
        const auto &ID = match.first;
        tracks_[ID].Update(match.second);
    }

    // Initialization + Update
    // class_id, bbox, velocity, distance
    for (const auto &unmatch : unmatched_det) {
        Track tracker;
        // cout << "Unmatched!" << endl;
        tracker.Init(unmatch);
        // state + (class_id, velocity, distance)
        tracks_[id_++] = tracker;
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (it->second.coast_cycles_ > kMaxCoastCycles) {
            it = tracks_.erase(it);
        } else {
            it++;
        }
    }
}


std::map<int, Track> Tracker::GetTracks() {
    return tracks_;
}



// void Tracker::AssociateDetectionsToTrackers(const vector<box>& detected_object,
//                                             std::map<int, Track>& tracks,
//                                             std::map<int, box>& matched,
//                                             std::vector<box>& unmatched_det,
//                                             float iou_threshold) {
    
//     // class_id, bbox, velocity, distance
//     if (tracks.empty()) {
//         for (const auto& det : detected_object) {
//             unmatched_det.push_back(det);
//         }
//         return;
//     }

//     std::vector<std::vector<float>> iou_matrix;
//     iou_matrix.resize(detected_object.size(), std::vector<float>(tracks.size()));

//     std::vector<std::vector<float>> association;
//     association.resize(detected_object.size(), std::vector<float>(tracks.size()));

//     for (size_t i = 0; i < detected_object.size(); i++) {
//         size_t j = 0;
//         for (const auto& trk : tracks) {
//             iou_matrix[i][j] = CalculateIoU(detected_object[i], trk.second);
//             j++;
//         }
//     }

//     HungarianMatching(iou_matrix, detected_object.size(), tracks.size(), association);

//     for (size_t i = 0; i < detected_object.size(); i++) {
//         bool matched_flag = false;
//         size_t j = 0;
//         for (const auto& trk : tracks) {
//             // cout << "check_point 1" << endl;
//             if (0 == association[i][j]) {
//                 // cout << "check_point 2" << endl;
//                 // cout << "iou_matrix[i][j] : " << iou_matrix[i][j] << endl; // -> 0
//                 if (iou_matrix[i][j] >= iou_threshold) {
//                     // cout << "iou_point" << endl;
//                     matched[trk.first] = detected_object[i];
//                     matched_flag = true;
//                 }
//                 break;
//             }
//             j++;
//         }
//         if (!matched_flag) {
//             // cout << "check_point 3" << endl;
//             unmatched_det.push_back(detected_object[i]);
//         }
//     }
// }



// void Tracker::Run(const vector<box>& detected_object) {

//     for (auto &track : tracks_) {
//         track.second.Predict();
//     }

//     std::map<int, box> matched;
//     std::vector<box> unmatched_det;

//     if (!detected_object.empty()) {
//         AssociateDetectionsToTrackers(detected_object, tracks_, matched, unmatched_det);
//     }

//     for (const auto &match : matched) {
//         // cout << "Matchend!" << endl;
//         const auto &ID = match.first;
//         tracks_[ID].Update(match.second);
//     }

//     // Initialization + Update
//     // class_id, bbox, velocity, distance
//     for (const auto &unmatch : unmatched_det) {
//         Track tracker;
//         // cout << "Unmatched!" << endl;
//         tracker.Init(unmatch);
//         // state + (class_id, velocity, distance)
//         tracks_[id_++] = tracker;
//     }

//     for (auto it = tracks_.begin(); it != tracks_.end();) {
//         if (it->second.coast_cycles_ > kMaxCoastCycles) {
//             it = tracks_.erase(it);
//         } else {
//             it++;
//         }
//     }
// }





