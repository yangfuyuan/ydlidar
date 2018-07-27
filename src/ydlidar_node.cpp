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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "CYdLidar.h"
#include "timer.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>


using namespace ydlidar;

#define ROSVerision "1.3.5"

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
    bool angle_fixed, intensities,low_exposure,reversion, resolution_fixed,heartbeat;
    bool auto_reconnect, debug, sync_imu;
    double angle_max,angle_min;
    int samp_rate;
    std::string list;
    std::vector<float> ignore_array;  
    double max_range , min_range;
    double _frequency, sensor_x, sensor_y, sensor_yaw;
    bool enable_pasing;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::Publisher sync_scan_pub = nh.advertise<sensor_msgs::LaserScan>("sync_scan", 1000);
	ros::Publisher marker_publisher_ = nh.advertise<visualization_msgs::Marker>("publish_line_markers", 1);

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ydlidar"); 
    nh_private.param<int>("baudrate", baudrate, 115200); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("angle_fixed", angle_fixed, "true");
    nh_private.param<bool>("resolution_fixed", resolution_fixed, "true");
    nh_private.param<bool>("heartbeat", heartbeat, "false");
    nh_private.param<bool>("low_exposure", low_exposure, "false");
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true");
    nh_private.param<bool>("debug", debug, "false");
    nh_private.param<bool>("enable_pasing", enable_pasing, "false");
    nh_private.param<bool>("intensity", intensities, "false");
    nh_private.param<double>("angle_max", angle_max , 180);
    nh_private.param<double>("angle_min", angle_min , -180);
    nh_private.param<int>("samp_rate", samp_rate, 4); 
    nh_private.param<double>("range_max", max_range , 16.0);
    nh_private.param<double>("range_min", min_range , 0.08);
    nh_private.param<double>("frequency", _frequency , 7.0);
    nh_private.param<double>("sensor_x", sensor_x , 0.0);
    nh_private.param<double>("sensor_y", sensor_y , 0.0);
    nh_private.param<double>("sensor_yaw", sensor_yaw , 0.0);

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

    CYdLidar laser;

    if(_frequency<5){
       _frequency = 7.0; 
    }
    if(_frequency>12){
        _frequency = 12;
    }
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setIntensities(intensities);
    laser.setMaxRange(max_range);
    laser.setMinRange(min_range);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setHeartBeat(heartbeat);
    laser.setFixedResolution(resolution_fixed);
    laser.setAutoReconnect(auto_reconnect);
    laser.setEnableDebug(debug);
    laser.setExposure(low_exposure);
    laser.setScanFrequency(_frequency);
    laser.setSampleRate(samp_rate);
    laser.setEnablCorrectionAngle(enable_pasing);
    laser.setIgnoreArray(ignore_array);

    //雷达相对机器人安装位置
    pose_info laser_pose;
    laser_pose.x = sensor_x;
    laser_pose.y = sensor_y;
    laser_pose.phi = sensor_yaw;
    laser.setSensorPose(laser_pose);

    printf("[YDLIDAR INFO] Current ROS Driver Version: %s\n",((std::string)ROSVerision).c_str());
    laser.initialize();

    ros::Rate rate(30);

    while (ros::ok()) {
        bool hardError;
        LaserScan scan;//原始激光数据
        LaserScan syncscan;//同步后激光数据
        PointCloud pc;//同步后激光点云数据
        std::vector<gline> lines;
        if(laser.doProcessSimple(scan, syncscan, pc, lines, hardError )){

            sensor_msgs::LaserScan scan_msg;
            sensor_msgs::LaserScan sync_scan_msg;

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

	    sync_scan_msg.header.stamp = start_scan_time;
            sync_scan_msg.header.frame_id = frame_id;
            sync_scan_msg.angle_min = syncscan.config.min_angle;
            sync_scan_msg.angle_max = syncscan.config.max_angle;
            sync_scan_msg.angle_increment = syncscan.config.ang_increment;
            sync_scan_msg.scan_time = syncscan.config.scan_time;
            sync_scan_msg.time_increment = syncscan.config.time_increment;
            sync_scan_msg.range_min = syncscan.config.min_range;
            sync_scan_msg.range_max = syncscan.config.max_range;
            
            sync_scan_msg.ranges = syncscan.ranges;
            sync_scan_msg.intensities =  syncscan.intensities;
	    sync_scan_pub.publish(sync_scan_msg);
            

            visualization_msgs::Marker marker_msg;
            marker_msg.ns = "laser_fit_lines";
	        marker_msg.id = 0;
	        marker_msg.type = visualization_msgs::Marker::LINE_LIST;
	        marker_msg.scale.x = 0.03;
	        marker_msg.color.r = 0.0;
	        marker_msg.color.g = 1.0;
	        marker_msg.color.b = 0.0;
	        marker_msg.color.a = 1.0;
            for(std::vector<gline>::const_iterator it = lines.begin(); it != lines.end(); it++) {
                geometry_msgs::Point p_start;
		        p_start.x = it->x1;
	            p_start.y = it->y1;
	            p_start.z = 0;
	            marker_msg.points.push_back(p_start);
	            geometry_msgs::Point p_end;
	            p_end.x = it->x2;
	            p_end.y = it->y2;
	            p_end.z = 0;
	            marker_msg.points.push_back(p_end);
            }
            marker_msg.header.frame_id = frame_id;
	        marker_msg.header.stamp = start_scan_time;
            marker_publisher_.publish(marker_msg);
            

		}

        /*{//做imu和odometry数据输入
            odom_info odom;
            odom.x = 0;
            odom.y = 0;
            odom.phi = 0;
            odom.stamp = getTime();
            laser.setSyncOdometry(odom);


        }*/

        rate.sleep();
        ros::spinOnce();

	}
  
    laser.turnOff();
    laser.disconnecting();
    return 0;
}
