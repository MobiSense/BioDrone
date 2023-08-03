#ifndef _DATA_STORAGE_H
#define _DATA_STORAGE_H

#include <vector>
#include <mutex>

#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>

using std::vector;

struct Event {
    int x;
    int y;
    int p;
    double t;
};

struct EventPacket {
    int id;
    vector<Event> events;
};

struct EnhancedTimeSurface {
    vector<int>     polaritySurface;    // maintained by storage
    vector<double>  timestampSurface;   // maintained by storage
    vector<double>  lastModifiedAt;     // maintained by user
    vector<double>  timeSurface;        // maintained by user

    double referenceTimestamp;
    EnhancedTimeSurface();

    void clear();

    inline double at(int loc) {  
        if (referenceTimestamp > lastModifiedAt[loc]) {
            lastModifiedAt[loc] = referenceTimestamp;
            timeSurface[loc] = std::exp((timestampSurface[loc] - referenceTimestamp) / Storage::decayRate);
        }
        return timeSurface[loc];
    }
};


struct ImuPacket {
    int id;
    double x;
    double y;
    double z;
};

struct DataStorage {
    vector<EventPacket> packets;
    vector<ImuPacket>   imus;
    EnhancedTimeSurface ts;
};

struct StereoDataStorage {
public:
    DataStorage left, right;
    std::mutex storage_lock;

    void init();


private:
    struct Buffer {
        vector<dvs_msgs::EventArray::ConstPtr>  leftEvents;
        vector<dvs_msgs::EventArray::ConstPtr>  rightEvents;
        vector<sensor_msgs::Imu::ConstPtr>      leftImus;
        vector<sensor_msgs::Imu::ConstPtr>      rightImus;

        std::recursive_mutex buffer_lock;
    } unprocessedEventsBuffer;


private:
    ros::NodeHandle nh;
    vector<ros::Subscriber> subs;
    ros::Timer bufferTimer;

public:
    void eventCallback(const dvs_msgs::EventArray::ConstPtr& msg, bool isLeftCam);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg, bool isLeftCam);
    void handleBuffer();
};

#endif