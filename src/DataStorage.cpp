#include <ros/ros.h>

#include <BioDrone/CameraConfig.h>
#include <BioDrone/DataStorage.h>

#include <memory.h>
#include <thread>
#include <mutex>
#include <utility>
#include <boost/bind.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>

EnhancedTimeSurface::EnhancedTimeSurface() : referenceTimestamp(.0),
                                             polaritySurface(Cam::imageSize, 0), timestampSurface(Cam::imageSize, .0),
                                             lastModifiedAt(Cam::imageSize, .0), timeSurface(Cam::imageSize, .0)
{
}

void EnhancedTimeSurface::clear()
{
    memset(polaritySurface.data(), 0, sizeof(int) * Cam::imageSize);
    memset(timestampSurface.data(), 0, sizeof(double) * Cam::imageSize);
    memset(lastModifiedAt.data(), 0, sizeof(double) * Cam::imageSize);
    memset(timeSurface.data(), 0, sizeof(double) * Cam::imageSize);
    referenceTimestamp = .0;
}

void StereoDataStorage::init()
{
    subs.push_back(nh.subscribe<dvs_msgs::EventArray>("/davis/left/events", 1000, boost::bind(&StereoDataStorage::eventCallback, this, _1, true)));
    subs.push_back(nh.subscribe<dvs_msgs::EventArray>("/davis/right/events", 1000, boost::bind(&StereoDataStorage::eventCallback, this, _1, false)));
    subs.push_back(nh.subscribe<sensor_msgs::Imu>("/davis/left/imu", 1000, boost::bind(&StereoDataStorage::imuCallback, this, _1, true)));
    subs.push_back(nh.subscribe<sensor_msgs::Imu>("/davis/right/imu", 1000, boost::bind(&StereoDataStorage::imuCallback, this, _1, false)));

    bufferTimer = nh.createTimer(ros::Duration(0.03), boost::bind(&StereoDataStorage::handleBuffer, this), false, true);
}

void StereoDataStorage::eventCallback(const dvs_msgs::EventArray::ConstPtr &msg, bool isLeftCam)
{
    std::lock_guard<std::recursive_mutex> lock(unprocessedEventsBuffer.buffer_lock);
    if (isLeftCam)
    {
        unprocessedEventsBuffer.leftEvents.push_back(msg);
    }
    else
    {
        unprocessedEventsBuffer.rightEvents.push_back(msg);
    }
    handleBuffer();
}

void StereoDataStorage::imuCallback(const sensor_msgs::Imu::ConstPtr &msg, bool isLeftCam)
{
    std::lock_guard<std::recursive_mutex> lock(unprocessedEventsBuffer.buffer_lock);
    if (isLeftCam)
    {
        unprocessedEventsBuffer.leftImus.push_back(msg);
    }
    else
    {
        unprocessedEventsBuffer.rightImus.push_back(msg);
    }
    handleBuffer();
}

void StereoDataStorage::handleBuffer()
{
    std::lock_guard<std::recursive_mutex> lock(unprocessedEventsBuffer.buffer_lock);
    if (!storage_lock.try_lock())
    {
        return;
    }

    // Only new incoming events and imus (in the buffer) will be processed, others will be neglected.
    using namespace ::Cam;
    using namespace ::Buffer;
    using namespace ::Storage;

#ifdef _SINGLE_PACKET

#else
    // left event packets
    int size = unprocessedEventsBuffer.leftEvents.size();
    for (int i = std::max(0, size - (int)maxAllowedBufferSize); i < size; i++)
    {
        auto &arr = unprocessedEventsBuffer.leftEvents[i]->events;

        if (!arr.empty())
        {
            auto idStart = (int)((arr[0].ts.toSec() - systemBaseTimestamp) * 1000);
            auto packetBase = left.packets.size();
            left.packets.resize(packetBase + receivedPacketSize);
            for (int b = packetBase; b < packetBase + receivedPacketSize; b++)
            {
                auto &packet = left.packets[b];
                packet.id = idStart + b - packetBase;
                packet.events.reserve((arr.size() / receivedPacketSize) + 1);
            }
            for (auto &e : arr)
            {
                auto &polarity = left.ts.polaritySurface[at(e.x, e.y)];
                polarity = e.polarity ? 1 : -1;
                auto &timestamp = left.ts.timestampSurface[at(e.x, e.y)];
                timestamp = e.ts.toSec();
                auto id = (int)((timestamp - systemBaseTimestamp) * 1000);
                if ((unsigned)(id - idStart) < receivedPacketSize)
                {
                    left.packets[packetBase + id - idStart].events.push_back(Event{e.x, e.y, polarity, timestamp});
                }
            }

            if (left.packets.size() > storageLength)
            {
                left.packets.erase(left.packets.begin(), left.packets.end() - storagePreserveLength);
            }
        }
    }

    size = unprocessedEventsBuffer.rightEvents.size();
    for (int i = std::max(0, size - (int)maxAllowedBufferSize); i < size; i++)
    {
        auto &arr = unprocessedEventsBuffer.rightEvents[i]->events;

        if (!arr.empty())
        {
            auto idStart = (int)((arr[0].ts.toSec() - systemBaseTimestamp) * 1000);
            auto packetBase = right.packets.size();
            right.packets.resize(packetBase + receivedPacketSize);
            for (int b = packetBase; b < packetBase + receivedPacketSize; b++)
            {
                auto &packet = right.packets[b];
                packet.id = idStart + b - packetBase;
                packet.events.reserve((arr.size() / receivedPacketSize) + 1);
            }
            for (auto &e : arr)
            {
                auto &polarity = right.ts.polaritySurface[at(e.x, e.y)];
                polarity = e.polarity ? 1 : -1;
                auto &timestamp = right.ts.timestampSurface[at(e.x, e.y)];
                timestamp = e.ts.toSec();
                auto id = (int)((timestamp - systemBaseTimestamp) * 1000);
                if ((unsigned)(id - idStart) < receivedPacketSize)
                {
                    right.packets[packetBase + id - idStart].events.push_back(Event{e.x, e.y, polarity, timestamp});
                }
            }

            if (right.packets.size() > storageLength)
            {
                right.packets.erase(right.packets.begin(), right.packets.end() - storagePreserveLength);
            }
        }
    }

    size = unprocessedEventsBuffer.leftImus.size();
    for (int i = std::max(0, size - (int)maxAllowedBufferSize); i < size; i++)
    {
        auto &msg = unprocessedEventsBuffer.leftImus[i];
        auto idStart = (int)((msg->header.stamp.toSec() - systemBaseTimestamp) * 1000);
        for (int b = 0; b < receivedPacketSize; b++)
        {
            left.imus.push_back(ImuPacket{idStart + b, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z});
        }

        if (left.imus.size() > storageLength)
        {
            left.imus.erase(left.imus.begin(), left.imus.end() - storagePreserveLength);
        }
    }

    size = unprocessedEventsBuffer.rightImus.size();
    for (int i = std::max(0, size - (int)maxAllowedBufferSize); i < size; i++)
    {
        auto &msg = unprocessedEventsBuffer.rightImus[i];
        auto idStart = (int)((msg->header.stamp.toSec() + imuTimeOffset - systemBaseTimestamp) * 1000);
        for (int b = 0; b < receivedPacketSize; b++)
        {
            right.imus.push_back(ImuPacket{idStart + b, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z});
        }

        if (right.imus.size() > storageLength)
        {
            right.imus.erase(right.imus.begin(), right.imus.end() - storagePreserveLength);
        }
    }

    // Clear buffer
    unprocessedEventsBuffer.leftEvents.clear();
    unprocessedEventsBuffer.rightEvents.clear();
    unprocessedEventsBuffer.leftImus.clear();
    unprocessedEventsBuffer.rightImus.clear();
#endif

    storage_lock.unlock();
}