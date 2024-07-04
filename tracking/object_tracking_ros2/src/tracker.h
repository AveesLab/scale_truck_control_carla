#include <vector>
#include <map>

#include "track.h"
#include "munkres.h"
#include "tracker_utils.h"

using namespace std;
extern "C"
class Tracker {
public:
    Tracker();
    ~Tracker() = default;
    // cv::Rect -> box bbox change
    static float CalculateIoU(const object_info& obj, const Track& track);

    static void HungarianMatching(const vector<vector<float>>& iou_matrix,
                                size_t nrows, size_t ncols,
                                vector<vector<float>>& association);

    static void AssociateDetectionsToTrackers(const vector<object_info>& detected_object,
                                            map<int, Track>& tracks,
                                            map<int, object_info>& matched,
                                            vector<object_info>& unmatched_det,
                                            float iou_threshold = 0.3);
    
    void Run(const vector<object_info>& detected_object);

    std::map<int, Track> GetTracks();
    std::map<int, Track> tracks_;
private:

    int id_;
};

std::map<int, Track> fnGetTracks();
void fnRun(const vector<box>& detected_object);