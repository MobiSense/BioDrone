#ifndef _CAMERA_CONFIG_H
#define _CAMERA_CONFIG_H

// #define _BLOCK_MATCHING_MSE

#ifndef _BLOCK_MATCHING_MSE
#define _BLOCK_MATCHING_CELL
#endif

#ifndef _SINGLE_PACKET
#define _MULTIPLE_PACKET
#endif

#include <Eigen/Eigen>

namespace Cam {
    const unsigned width = 346;
    const unsigned height = 260;
    const unsigned imageSize = width * height;

    const double imuTimeOffset = 0; // Sync imu with events

    // T_ci transforms w from ref_i to ref_c
    const Eigen::Affine3f T_ci((Eigen::Matrix4f() <<   0.99863559,  0.05017082,  0.01448639, -0.00157674,
                                                        -0.0503944,   0.99860904,  0.01550466, -0.00110388,
                                                        -0.01368836, -0.01621354,  0.99977485, -0.03332234,
                                                        0.        ,  0.         , 0.        , 1.        ).finished());

    // Cam intrinsic
    const Eigen::Matrix3f K = (Eigen::Matrix3f() << 337.07529747174203, 0                , 167.4994566882768,
                                                    0,                  336.1836394581704, 129.0404094871703,
                                                    0,                  0                , 1                ).finished();
    
    // Stereo Cam Baseline
    const double baseline   = 18.77403274063751; // baseline distance measured in pixels.

    inline int at(int x, int y) {
        return y * width + x;
    }

    inline int X(int i) {
        return i % width;
    }

    inline int Y(int i) {
        return i / width;
    }

    inline bool in(int x, int y) {
        return (unsigned)x < width && (unsigned)y < height;
    }
}

namespace Buffer {
    const unsigned maxAllowedBufferSize = 200;
}

namespace Storage {
    extern  double   systemBaseTimestamp; // Dataset Timestamp.
    const   unsigned eventPacketInterval = 1; // (ms)
    const   unsigned receivedEventArrayInterval = 1; // (ms)
    const   unsigned receivedPacketSize = receivedEventArrayInterval / eventPacketInterval;

    const   unsigned storageLength = 300; // only 300 items will be saved in each of the dataset.
    const   unsigned storagePreserveLength = storageLength * 0.4; // only 300 * 0.4 items will be saved in each of the dataset.

    const   double   decayRate = 0.03; //unit:(s)
}

namespace Imu {
    const unsigned compensatedPackets = 30; // the number of packets will be considered in a single imu-compensation process.
    const unsigned maxAllowedLatency = 10; // if latency <= maxAllowedLatency, the latency is neglected. otherwise, a synchronizing process is performed
}

namespace StereoConstraint {
    const unsigned compensatedPackets = 15; 
    const double   rangeMin = 0.1;
    const double   rangeMax = 10;

    const unsigned tsRefDeltaPackets = 3;
    const unsigned searchUncertainty = 5;
}

namespace ObjectDetecion {
    const double stereoWeight = 0.1;
    const double threshold    = 0.4; 
}

namespace DepthEstimation {
#ifdef _BLOCK_MATCHING_MSE
    const int blockSize             = 5; // block size used for matching
#else 
    const int blockSize             = 10;
#endif
    const int epipolarLineLength    = 50; // epipolar line length for search;
    const double infty              = 1e5;  // positive infinity
    const double threshold          = 0.15; 
}

#endif