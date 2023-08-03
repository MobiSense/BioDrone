#ifndef _OBSTACLE_AVOIDANCE
#define _OBSTACLE_AVOIDANCE

#include <BioDrone/DataStorage.h>
#include <string>

class ObstacleAvoidance {

public:
    bool shouldBreak = false;
    void run(StereoDataStorage &data);

    ObstacleAvoidance();

private:
    StereoDataStorage *data = nullptr;

    // General Computing
    void compute();

    // Object Detection
    double predictedDepth = .0; // unit:(m)
    vector<double> leftImuImage, rightImuImage, leftStereoImage, rightStereoImage;
    vector<int> leftObjectImage, rightObjectImage;
    void computeMotionCompensation(vector<int>& eventCount, bool isLeft);
    bool computeStereoConstraint();
    void computeObjectDetection(bool useStereo);

    // Depth Estimation
    void depthEstimation();

    // Empty Vectors
    template <typename T>
    static void clear(vector<T> &vec);
};

#endif