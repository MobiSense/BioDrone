#include <ros/ros.h>

#include <BioDrone/CameraConfig.h>
#include <BioDrone/ObstacleAvoidance.h>

#include <string>
#include <mutex>
#include <chrono>
#include <thread>
#include <memory.h>

#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

void little_sleep(std::chrono::microseconds us);

ObstacleAvoidance::ObstacleAvoidance():
leftImuImage(Cam::imageSize, .0), rightImuImage(Cam::imageSize, .0),
leftStereoImage(Cam::imageSize, .0), rightStereoImage(Cam::imageSize, .0),
leftObjectImage(Cam::imageSize, 0), rightObjectImage(Cam::imageSize, 0) {
}

void ObstacleAvoidance::run(StereoDataStorage &data) {
    this->data = &data;
    while(!shouldBreak) {
        compute();
    }
}

void ObstacleAvoidance::compute() {
    if (!data->storage_lock.try_lock()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
    }

    using namespace Cam;
    static vector<int> leftEventCount(imageSize, 0), rightEventCount(imageSize, 0); // Imu event counter
    
    // Do jobs in parallel: motion-compensation, stereo-matching
    std::thread worker1(&ObstacleAvoidance::computeMotionCompensation, this, std::ref(leftEventCount), true);
    std::thread worker2(&ObstacleAvoidance::computeMotionCompensation, this, std::ref(rightEventCount), false);

    auto executedStereoConstraint = computeStereoConstraint();

    worker1.join();
    worker2.join();

    // Compute Detected Object
    computeObjectDetection(executedStereoConstraint);

    static int lastSeenId = 0;
    if (!data->left.packets.empty() && data->left.packets.back().id > lastSeenId) {
        std::cout << "id=" << lastSeenId++ << ",\td=" << predictedDepth << "\n";
    }

    // Calculating the depth of objects.
    depthEstimation();
    data->storage_lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(3)); // We don't have to sleep such a long duration when all algos are integrated.
}

void ObstacleAvoidance::computeMotionCompensation(vector<int>& eventCount, bool isLeft) {
    auto *result = &(isLeft? leftImuImage : rightImuImage);
    auto *storage = &(isLeft? data->left : data->right);

    auto &packets = storage->packets;
    auto &imus = storage->imus;

    if (packets.empty()) {
        return;
    }

    ObstacleAvoidance::clear(eventCount);
    ObstacleAvoidance::clear(*result);

    using namespace Imu;
    using namespace Cam;

    // Handle synchronize between events and imus
    auto lastPacketId = packets.back().id;

    Eigen::Vector3f imu(0, 0, 0);
    int imuCount = 0;
    for(auto it = imus.rbegin(); it != imus.rend(); it++) {
        if (it->id - lastPacketId <= maxAllowedLatency && lastPacketId - compensatedPackets - it->id <= maxAllowedLatency) {
            imuCount++;
            imu += Eigen::Vector3f(it->x, it->y, it->z);
        } else {
            break;
        }
    }

    auto norm = imu.norm();
    if (norm > .0) {
        imu /= imuCount;
    }

    for (auto packet = packets.rbegin(); packet != packets.rend(); packet++) {
        int diff = lastPacketId - packet->id;
        if (diff > compensatedPackets) {
            break;
        }

        bool identical = true;
        Eigen::Matrix3f projMat;

        if (norm > .0 && diff != 0) {
            auto rotVec = T_ci * (imu * (-diff * (double)Storage::eventPacketInterval) / 1000.0);
            projMat = K * Eigen::AngleAxisf(rotVec.norm(), rotVec / rotVec.norm()).toRotationMatrix() * K.inverse();
            identical = false;
        }

        for (auto &e : packet->events) {
            int x = e.x, y = e.y;
            if (!identical) {
                Eigen::Vector3f p = projMat * Eigen::Vector3f(e.x, e.y, 1);
                p /= p.z();
                x = p.x() + 0.5, y = p.y() + 0.5;
            }
            if((unsigned)x < width && (unsigned)y < height) {
                eventCount[at(x, y)]++;
                (*result)[at(x, y)] += e.t;
            }
        }
    }

    static const double deltaTime = compensatedPackets * (double)Storage::eventPacketInterval / 1000.0;
    double avgTime = .0;
    int candidatePixels = 0;

    for(int i = 0; i < imageSize; i++) {
        if (eventCount[i]) {
            candidatePixels++;
            (*result)[i] /= eventCount[i];
            avgTime += (*result)[i];
        }
    }

    if (candidatePixels) {
        avgTime /= candidatePixels;
    }

    // static const double threshold = 0.45;
    for(int i = 0; i < imageSize; i++) {
        (*result)[i] = std::max(.0, ((*result)[i] - avgTime) / deltaTime);
        (*result)[i] *= 2;
        (*result)[i] = std::min(1.0, std::max(-1.0, (*result)[i]));
    }
}

bool ObstacleAvoidance::computeStereoConstraint() {
    using namespace StereoConstraint;

    auto &leftTs = data->left.ts;
    auto &rightTs = data->right.ts;
    auto &leftPackets = data->left.packets;
    auto &rightPackets = data->right.packets;

    if (predictedDepth < rangeMin || predictedDepth > rangeMax || leftPackets.empty() || rightPackets.empty()) {
        return false;
    }

    unsigned lastId = std::max(leftPackets.back().id, rightPackets.back().id);
    rightTs.referenceTimestamp = leftTs.referenceTimestamp =
     (lastId + tsRefDeltaPackets) * 0.001 + Storage::systemBaseTimestamp;

    using namespace Cam;

    clear(leftStereoImage);
    clear(rightStereoImage);

    // Compensate for the left frame
    for (auto packet = leftPackets.rbegin(); packet != leftPackets.rend(); packet++) {
        if (lastId - packet->id > compensatedPackets) {
            break;
        }
        for(auto &e : packet->events) {
            auto loc = at(e.x, e.y);
            auto &leftPixel = leftStereoImage[loc];
            if (leftPixel == .0) {
                auto x = e.x - (int)(baseline / predictedDepth) - searchUncertainty / 2;
                auto y = e.y;
                for(int step = 0; step < searchUncertainty; step++) {
                    auto nextLoc = at(x + step, y);
                    if (in(x + step, y) && leftTs.polaritySurface[loc] == rightTs.polaritySurface[nextLoc]) {
                        double score = 1 - std::abs(leftTs.at(loc) - rightTs.at(nextLoc));
                        auto &rightPixel = rightStereoImage[nextLoc];
                        leftPixel = std::max(score, leftPixel);
                        rightPixel = std::max(score, rightPixel);
                    }
                }
            }
        }
    }
    return true;
}

void ObstacleAvoidance::computeObjectDetection(bool useStereo) {

    using namespace ObjectDetecion;
    using namespace Cam;

    double a = stereoWeight;
    double b = 1 - a;
    static vector<int> leftFilter(imageSize, 0), rightFilter(imageSize, 0);
    for(auto i = 0; i < Cam::imageSize; i++) {
        leftFilter[i] = !!((useStereo? (a * leftStereoImage[i] + b * leftImuImage[i]) : (leftImuImage[i])) > threshold) * 0xff;
        rightFilter[i] = !!((useStereo? (a * rightStereoImage[i] + b * rightImuImage[i]) : (rightImuImage[i])) > threshold) * 0xff;
    }

    // Try with a simple filter.
    for(auto i = 0; i < Cam::imageSize; i++) {
        int sx = X(i) - 1, sy = Y(i) - 1;
        int voteLeft = 0, voteRight = 0;
        for(int kx = 0; kx < 3; kx++) {
            for(int ky = 0; ky < 3; ky++) {
                if (in(sx + kx, sy + ky)) {
                    voteLeft += !!leftFilter[at(sx + kx, sy + ky)];
                    voteRight += !!rightFilter[at(sx + kx, sy + ky)];
                }
            }
        }
        leftObjectImage[i] = !!(voteLeft > 3) * 0xff;
        rightObjectImage[i] = !!(voteRight > 3) * 0xff;
    }
}

void ObstacleAvoidance::depthEstimation() {
    using namespace Cam;
    using namespace DepthEstimation;

    double estimatedDepth = 0.0;
    int validEvents = 0;

    auto &leftTs = data->left.ts;
    auto &rightTs = data->right.ts;
    auto &leftPackets = data->left.packets;
    auto &rightPackets = data->right.packets;

    if (leftPackets.empty() || rightPackets.empty()) {
        return;
    }

    unsigned lastId = std::max(leftPackets.back().id, rightPackets.back().id);
    rightTs.referenceTimestamp = leftTs.referenceTimestamp =
     (lastId + StereoConstraint::tsRefDeltaPackets) * 0.001 + Storage::systemBaseTimestamp;

    for (auto loc = 0; loc < Cam::imageSize; loc++) {
        if (leftObjectImage[loc]) {
            // find correspondence event
            auto x = X(loc), y = Y(loc);
            
#ifdef _BLOCK_MATCHING_MSE
            // block match
            double minScore = infty;  
            int bestDisparity = -1;
            int sx = x - (blockSize / 2), sy = y - (blockSize / 2);
            int totalPixels = blockSize * blockSize;

            for(int diff = 0; diff <= epipolarLineLength; diff++) {
                double score = 0;
                int validPixels = 0;
                
                // matching using MSE
                for(int i = sx; i < sx + blockSize; i++) {
                    // int j = e.y;
                    for(int j = sy; j < sy + blockSize; j++) {
                        if (in(i, j) && in(i - diff, j)) {
                            validPixels++;
                            score += pow(leftTs.at(at(i, j)) - rightTs.at(at(i - diff, j)), 2);
                        }
                    }
                }

                if (validPixels) {
                    score = score * totalPixels / validPixels;
                    if (score < minScore) {
                        minScore = score;
                        bestDisparity = diff;
                    }
                }
            }

            if (minScore < totalPixels * threshold && bestDisparity > 0) { // test score
                validEvents++;
                // convert disparity into depth
                double depth = (baseline) / (bestDisparity); // depth = true_depth / focal_length
                estimatedDepth += depth;
            }

#else
            // cell match
            double minScore = infty;  
            int bestDisparity = -1;
            int sx = x - (blockSize / 2), sy = y - (blockSize / 2);
            int totalPixels = blockSize *  4 * 0.5;

            for(int diff = 0; diff <= epipolarLineLength; diff++) {
                double score[5] = {1.0, .0, .0, .0, .0};
                int validPixels = 0;

                // matching using cell blocks
                auto inCell = [&](int i, int j) {
                    if (i == x) {
                        return 1;
                    } else if (j == y) {
                        return 2;
                    } else if (i - x == y - j) {
                        return 3;
                    } else if (i - x == j - y) {
                        return 4;
                    }
                    return 0;
                };

                for(int i = sx, t = 0; i < sx + blockSize; i++) {
                    // int j = e.y;
                    for(int j = sy; j < sy + blockSize; j++) {
                        if (in(i, j) && in(i - diff, j) && (t = inCell(i, j))) {
                            validPixels++;
                            score[t] += pow(leftTs.at(at(i, j)) - rightTs.at(at(i - diff, j)), 2);
                        }
                    }
                }

                if (validPixels) {
                    for(int i = 1; i < 5; i++) {   
                        score[i] /= blockSize;
                        score[0] *= score[i];
                    }
                    if (score[0] < minScore) {
                        minScore = score[0];
                        bestDisparity = diff;
                    }
                }
            }

            if (minScore < totalPixels * threshold && bestDisparity > 0) { // test score
                validEvents++;
                // convert disparity into depth
                double depth = (baseline) / (bestDisparity); // depth = true_depth / focal_length
                estimatedDepth += depth;
            }

#endif

        }
    }

    if (validEvents > 3) {
        estimatedDepth /= validEvents;
    } else if (validEvents < 1) {
        estimatedDepth = -1 / .0; // -infty
    } else {
        estimatedDepth = predictedDepth;
    }

    predictedDepth = estimatedDepth;

    static vector<double> historyDepth;
    double meanDepth = 0;
    if (estimatedDepth <= 0) {
        historyDepth.clear();
        predictedDepth = estimatedDepth;
    } else {
        historyDepth.push_back(estimatedDepth);
        for(auto &d : historyDepth) {
            meanDepth += d;
        }
        meanDepth /= (int)historyDepth.size();
        predictedDepth = std::min(estimatedDepth, meanDepth);
    } 
}

template <typename T>
void ObstacleAvoidance::clear(vector<T> &vec) {
    memset(vec.data(), 0, sizeof(T) * vec.size());
}