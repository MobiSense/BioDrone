#include <ros/ros.h>

#include <BioDrone/CameraConfig.h>
#include <BioDrone/DataStorage.h>
#include <BioDrone/ObstacleAvoidance.h>

#include <iostream>
#include <thread>
#include <boost/bind.hpp>

#include <signal.h>
#include <cstdlib>

void signalHandler(int sig);
void signalRegister();

ObstacleAvoidance algo;

double Storage::systemBaseTimestamp;

int main(int argc, char **argv)
{
    if (argc <= 1) {
        std::cout << "Usage: rosrun BioDrone main TIMESTAMP\n" \
                  << "TIMESTAMP refers to the beginning timestamp of your bag file.\n";
        return 0;
    }

    Storage::systemBaseTimestamp = atof(argv[1]);

    ros::init(argc, argv, "BioDrone");

    ROS_INFO("Initialized.");

    signalRegister();

    StereoDataStorage storage;
    storage.init();

    std::thread worker(boost::bind(&ObstacleAvoidance::run, &algo, std::ref(storage)));
    worker.detach();

    ros::spin();
}

void signalRegister()
{
    signal(SIGINT, signalHandler);
}

void signalHandler(int sig)
{
    algo.shouldBreak = true;
    ros::shutdown();
}