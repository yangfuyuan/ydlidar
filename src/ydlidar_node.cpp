/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "CYdLidar.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

using namespace ydlidar;

#define ROSVerision "1.3.6"


std::vector<float> split(const std::string &s, char delim) {
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "ydlidar_node"); 

    std::string port;
    int baudrate=115200;
    std::string frame_id;
    bool angle_fixed,reversion, resolution_fixed;
    bool auto_reconnect, check_lidar_arc;
    double angle_max,angle_min;
    double check_angle_max,check_angle_min;
    double check_threshold;
    result_t op_result;
    std::string list;
    std::vector<float> ignore_array;  
    double max_range, min_range;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ydlidar"); 
    nh_private.param<int>("baudrate", baudrate, 115200); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("angle_fixed", angle_fixed, "true");
    nh_private.param<bool>("resolution_fixed", resolution_fixed, "true");
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true");
    nh_private.param<bool>("reversion", reversion, "false");
    nh_private.param<bool>("check_lidar_arc", check_lidar_arc, "false");
    nh_private.param<double>("angle_max", angle_max , 180);
    nh_private.param<double>("angle_min", angle_min , -180);
    nh_private.param<double>("check_angle_max", check_angle_max , 30);
    nh_private.param<double>("check_angle_min", check_angle_min , -30);
    nh_private.param<double>("check_threshold", check_threshold , 4);
    nh_private.param<double>("range_max", max_range , 16.0);
    nh_private.param<double>("range_min", min_range , 0.08);
    nh_private.param<std::string>("ignore_array",list,"");

    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be between -180 and 180");
        }
    }

    ROS_INFO("[YDLIDAR INFO]: ROS SDK VERSION: %s", ROSVerision);
    CYdLidar laser;
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setMaxRange(max_range);
    laser.setMinRange(min_range);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setReversion(reversion);
    laser.setFixedResolution(resolution_fixed);
    laser.setAutoReconnect(auto_reconnect);
    laser.setReversion(reversion);
    laser.setCheckLidarArc(check_lidar_arc);
    laser.setArcDetectMinAngle(check_angle_min);
    laser.setArcDetectMaxAngle(check_angle_max);
    laser.setThreshold(check_threshold);
    laser.setIgnoreArray(ignore_array);
    laser.initialize();

    ros::Rate rate(30);

    while (ros::ok()) {
        bool hardError;
        LaserScan scan;//原始激光数据
        if(laser.doProcessSimple(scan, hardError )){
            sensor_msgs::LaserScan scan_msg;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.system_time_stamp/1000000000ul;
            start_scan_time.nsec = scan.system_time_stamp%1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min = scan.config.min_angle;
            scan_msg.angle_max = scan.config.max_angle;
            scan_msg.angle_increment = scan.config.ang_increment;
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = scan.config.min_range;
            scan_msg.range_max = scan.config.max_range;
            
            scan_msg.ranges = scan.ranges;
            scan_msg.intensities =  scan.intensities;
            scan_pub.publish(scan_msg);

            if(laser.getCheckLidarArc()) {
                if(laser.getCheckOut()) {
                    ROS_INFO("SUCCESS");
                }else {
                    ROS_ERROR("FAILED");
                }
            }
        }  
        rate.sleep();
        ros::spinOnce();
    }

    laser.turnOff();
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
    laser.disconnecting();
    return 0;
}
