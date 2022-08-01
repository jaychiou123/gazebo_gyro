#include <pluginlib/class_list_macros.h>
#include "pub_planner/pub_planner.h"
#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_utils.hpp"  // rapidxml::file
#include "rapidxml/rapidxml_print.hpp"  // rapidxml::print
#include <iostream>
#include <fstream>
#include<string>
#include <stdio.h>
#include <string.h>
#include <sstream> 
#include <string>
#include <tf/tf.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nlohmann/json.hpp>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(pub_planner::PubPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using json = nlohmann::json;
tf2::Quaternion myQuaternion;

// Default Constructor
namespace pub_planner {
    
    PubPlanner::PubPlanner() {}
    
    PubPlanner::PubPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }
    
    void PubPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~" + name);
        map_resolution = 0.05;
        pathname_sub_ = pnh.subscribe("/path_name", 100, &PubPlanner::pathCallback, this);
        Series_path_srv_ = pnh.advertiseService("Series_path", &PubPlanner::Series_path, this);
        PubPlanner_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
        dsrv_ = new dynamic_reconfigure::Server<pub_plannerConfig>(pnh);
        dynamic_reconfigure::Server<pub_plannerConfig>::CallbackType cb = boost::bind(&PubPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        // load parameters
        pnh.param("waypoints_per_meter", waypoints_per_meter_, 20);
        pnh.param("heading_mode", heading_mode_, 1);

        std::string file_path = "";
        if (file_path != "")
        {
            pnh.param("svg_file_path", svg_file_path_, file_path);
            // pnh.param("path_num_p", path_num_p, 10);
            path_num_p=getpathnumber(svg_file_path_);
        }
    }
    
    bool PubPlanner::Series_path(pub_planner::Series_path::Request  &req,
                                 pub_planner::Series_path::Response &res)
    {
        bool no_path_flag = 0;
        std::string string = req.path.c_str();
        json PathJson = json::parse(string);
        std::cout << "seq:" << PathJson["""Header"""]["""seq"""] << std::endl;
        series_path_radius_.clear();
        series_path_turn_mode_.clear();
        station_list_to_get_radius_.clear();

        if (clear_waypoints_)
        {
            waypoints_.clear();
            clear_waypoints_ = false;
        }
        /**
         * 
         *
         * @todo   check the parameter "method" to identify is new path or extend path 
         */
        std::cout << "length:" << PathJson["""path"""].size() << std::endl;

        for (int i = 0; i < PathJson["""path"""].size() ; i++) 
        {
            // std::cout << "num: " << i << std::endl;
            // std::cout << "x: " << PathJson["""path"""][i]["x"] << std::endl;
            // std::cout << "y: " << PathJson["""path"""][i]["y"] << std::endl;
            // std::cout << "theta: " << PathJson["""path"""][i]["theta"] << std::endl;
            double radius, a0x, a0y, a1x, a1y, a2x, a2y, x1, x2, y1, y2, dot, det, angle, length;

            if (PathJson["""path"""][i]["radius"] > 0)
            {
                radius = PathJson["""path"""][i]["radius"];
                a0x = PathJson["""path"""][i]["x"];
                a0y = PathJson["""path"""][i]["y"];
                a1x = station_list_to_get_radius_[i-1].x;
                a1y = station_list_to_get_radius_[i-1].y;
                a2x = PathJson["""path"""][i+1]["x"];
                a2y = PathJson["""path"""][i+1]["y"];

                x1 = a1x-a0x+0.00000001;
                x2 = a2x-a0x+0.00000001;
                y1 = a1y-a0y+0.00000001;
                y2 = a2y-a0y+0.00000001;
                dot = x1*x2 + y1*y2;      // dot product between [x1, y1] and [x2, y2]
                det = x1*y2 - y1*x2;      // determinant
                angle = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)
                length = (radius/1000.0) / tan(abs(angle)/2.0);

                while (hypot(x1, y1) < length || hypot(x2, y2) < length)
                {
                    radius = (radius - 10) > 0 ? radius - 10 : 0;
                    if (radius == 0)
                        break;
                    length = (radius/1000.0) / tan(abs(angle)/2.0);
                }

                PathJson["""path"""][i]["radius"] = radius;
            }

            if (PathJson["""path"""][i]["radius"] > 0)
            {
                double control_1_x = a0x + (length*x1/abs(x1)*abs(cos(atan2(y1,x1))));
                double control_1_y = a0y + (length*y1/abs(y1)*abs(sin(atan2(y1,x1))));
                double control_2_x = a0x + (length*x2/abs(x2)*abs(cos(atan2(y2,x2))));
                double control_2_y = a0y + (length*y2/abs(y2)*abs(sin(atan2(y2,x2))));

                pub_planner::station_point_to_get_radius sp;
                sp.x = control_2_x;
                sp.y = control_2_y;
                station_list_to_get_radius_.push_back(sp);

                // std::cout << "y2 sign: " << y2/abs(y2) << std::endl;
                // std::cout << "control_1_x: " << control_1_x << std::endl;
                // std::cout << "control_1_y: " << control_1_y << std::endl;
                // std::cout << "control_2_x: " << control_2_x << std::endl;
                // std::cout << "control_2_y: " << control_2_y << std::endl;

                double pi = 4 * atan(1.0);
                double circle_center_x;
                double circle_center_y;
                double x3, y3, x3_prime, y3_prime;
                circle_circle_intersection(control_1_x, control_1_y, radius/1000, control_2_x, control_2_y, radius/1000,
                             &x3, &y3, &x3_prime, &y3_prime);

                if (sqrt(pow(a0x-x3, 2) + pow(a0y-y3, 2)) > sqrt(pow(a0x-x3_prime, 2) + pow(a0y-y3_prime, 2)))
                {
                    circle_center_x = x3;
                    circle_center_y = y3;
                }
                else
                {
                    circle_center_x = x3_prime;
                    circle_center_y = y3_prime;                
                }
            
                // std::cout << "x3: " << x3 << std::endl;
                // std::cout << "y3: " << y3 << std::endl;
                // std::cout << "x3_prime: " << x3_prime << std::endl;
                // std::cout << "y3_prime: " << y3_prime << std::endl;

                double angle_start = atan2(control_1_y-circle_center_y, control_1_x-circle_center_x);
                double angle_goal = atan2(control_2_y-circle_center_y, control_2_x-circle_center_x);
                double length_of_this_goal = hypot(control_2_x-control_1_x, control_2_y-control_1_y);
                double yaw_start = 0;
                double yaw_end = 0;
                int point_num_of_this_goal = waypoints_per_meter_ * length_of_this_goal;
                // std::cout << "angle_start: "<< angle_start  << ", angle_goal: "<< angle_goal  << std::endl;

                if(PathJson["""path"""][i]["turn"] == "keep_goal")
                {
                    yaw_start = PathJson["""path"""][i]["theta"];
                    yaw_start = yaw_start * pi/180.0;
                    yaw_end = PathJson["""path"""][i+1]["theta"];
                    yaw_end = yaw_end * pi/180.0;
                }
                else if (PathJson["""path"""][i]["turn"] == "keep_current")
                {
                    yaw_start = PathJson["""path"""][i-1]["theta"];
                    yaw_start = yaw_start * pi/180.0;
                    yaw_end = PathJson["""path"""][i]["theta"];
                    yaw_end = yaw_end * pi/180.0;
                }
                else if (PathJson["""path"""][i]["turn"] == "point_to_goal")
                {
                    double pi = 4 * atan(1.0);
                    double dx_start = a0x-a1x+0.0000001;
                    double dy_start = a0y-a1y+0.0000001;
                    double heading_angle_start = std::atan2(dy_start, dx_start);

                    if (heading_angle_start > pi) 
                        heading_angle_start = heading_angle_start - 2 * pi;
                    else if (heading_angle_start < -pi)
                        heading_angle_start = heading_angle_start + 2 * pi;
                    yaw_start = heading_angle_start;

                    double dx_end = a2x-a0x+0.0000001;
                    double dy_end = a2y-a0y+0.0000001;
                    double heading_angle_end = std::atan2(dy_end, dx_end);

                    if (heading_angle_end > pi) 
                        heading_angle_end = heading_angle_end - 2 * pi;
                    else if (heading_angle_end < -pi) 
                        heading_angle_end = heading_angle_end + 2 * pi;
                    yaw_end = heading_angle_end;
                }
                else if (PathJson["""path"""][i]["turn"]=="keep_last")
                {
                    yaw_start=PathJson["""path"""][i-1]["theta"];
                    yaw_start=yaw_start*pi/180.0;
                    yaw_end=PathJson["""path"""][i+1]["theta"];
                    yaw_end=yaw_end*pi/180.0;
                }

                yaw_start = atan2(sin(yaw_start), cos(yaw_start));
                yaw_end = atan2(sin(yaw_end), cos(yaw_end));

                double yaw_control = (yaw_end-yaw_start) / point_num_of_this_goal;

                if (yaw_end >= 0)
                {
                    std::cout << "yaw_control+: " << yaw_control * point_num_of_this_goal << std::endl;
                    if (abs(yaw_control) > abs((yaw_end-(2*pi)-yaw_start) / point_num_of_this_goal))
                        yaw_control = (yaw_end-(2*pi)-yaw_start) / point_num_of_this_goal;
                }
                else if (yaw_end < 0)
                {
                    std::cout << "yaw_control-: " << yaw_control * point_num_of_this_goal << std::endl;
                    if (abs(yaw_control) > abs((yaw_end+(2*pi)-yaw_start) / point_num_of_this_goal))
                        yaw_control = (yaw_end+(2*pi)-yaw_start) / point_num_of_this_goal;
                }
                std::cout << "yaw_control==: " << yaw_control * point_num_of_this_goal << std::endl;

                for (int i = 0; i < point_num_of_this_goal; i++)
                {
                    int sign = 1;
                    geometry_msgs::PoseStamped p  = geometry_msgs::PoseStamped();
                    p.header.frame_id = "map";
                    if (fabs(angle_goal-angle_start) > pi)
                    {
                        if (angle_goal < 0)
                            angle_goal = angle_goal + 2*pi;
                        if (angle_start < 0)
                            angle_start = angle_start + 2*pi;
                    }

                    p.pose.position.x=(radius/1000.0)*cos(angle_start+(angle_goal-angle_start)*sign/point_num_of_this_goal*i)+circle_center_x;
                    p.pose.position.y=(radius/1000.0)*sin(angle_start+(angle_goal-angle_start)*sign/point_num_of_this_goal*i)+circle_center_y;
                    // std::cout << "angle_cal: "<< angle_start+(angle_goal-angle_start)/point_num_of_this_goal*i << ", sin: "<< sin(angle_start+(angle_goal-angle_start)/point_num_of_this_goal*i) << ", cos: " << cos(angle_start+(angle_goal-angle_start)/point_num_of_this_goal*i) << std::endl;
                    std::cout << "p.pose.position.x: " << p.pose.position.x << ", p.pose.position.y: " << p.pose.position.y << std::endl;
                    waypoints_.push_back(geometry_msgs::PoseStamped());
                    ros::Time now = ros::Time::now();
                    waypoints_.back().header.stamp =  now;
                    waypoints_.back().header.frame_id ="map";
                    waypoints_.back().pose.position.x = p.pose.position.x;
                    waypoints_.back().pose.position.y = p.pose.position.y;
                    waypoints_.back().pose.position.z = 0;
                    waypoints_.back().pose.orientation = tf::createQuaternionMsgFromYaw(yaw_start + yaw_control*i);
                    std::cout << "control_yaw: " << yaw_start + yaw_control*i << std::endl;
                    series_path_turn_mode_.push_back(1);
                }
            }
            else
            {
                pub_planner::station_point_to_get_radius sp;
                sp.x = PathJson["""path"""][i]["x"];
                sp.y = PathJson["""path"""][i]["y"];
                station_list_to_get_radius_.push_back(sp);

                double pi = 4 * atan(1.0);
                waypoints_.push_back(geometry_msgs::PoseStamped());
                ros::Time now = ros::Time::now();
                waypoints_.back().header.stamp =  now;
                waypoints_.back().header.frame_id ="map";
                waypoints_.back().pose.position.x = PathJson["""path"""][i]["x"];
                waypoints_.back().pose.position.y = PathJson["""path"""][i]["y"];
                waypoints_.back().pose.position.z = 0;
                double yaw_ang = PathJson["""path"""][i]["theta"];
                waypoints_.back().pose.orientation = tf::createQuaternionMsgFromYaw(yaw_ang * pi/180.0);
                series_path_radius_.push_back(PathJson["""path"""][i]["radius"]);

                if (PathJson["""path"""][i]["turn"] == "keep_goal")
                    series_path_turn_mode_.push_back(1);
                else if (PathJson["""path"""][i]["turn"] == "keep_current")
                    series_path_turn_mode_.push_back(2);
                else if (PathJson["""path"""][i]["turn"] == "point_to_goal")
                    series_path_turn_mode_.push_back(3);
                else if (PathJson["""path"""][i]["turn"]=="keep_last")
                    series_path_turn_mode_.push_back(4);
            }
        }
        
        no_path_flag=0;
        planing_from_=2;

        if (no_path_flag == 0)
        {
            createAndPublishMarkersFromPath(waypoints_);
            ros::Time now = ros::Time::now();
            path_.header.stamp = now;
            path_.header.frame_id = "map";  // PathJson["""Header"""]["""frame_id"""]
            path_.poses.clear();
            path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
            std::cout << "path length: " << path_.poses.size() << std::endl;
            goal_pub_.publish(waypoints_.back());
            clear_waypoints_ = true;
            ROS_INFO("Published goal pose");
            res.ret = true;
            return res.ret;
        }
        else
        {
            ROS_INFO("No Point get");
            res.ret = false;
            return res.ret;
        }
    }


    void PubPlanner::reconfigureCB(pub_plannerConfig &config, uint32_t level)
    {
    	ROS_INFO("pub_planner reconfigureCB");

        if (strcmp(svg_file_path_.c_str(), config.path_src.c_str()))
        {
            if (strcmp("", config.path_src.c_str()))
            {
                svg_file_path_ = config.path_src;
                ROS_INFO_STREAM("change path file: " << svg_file_path_);
                path_num_p = getpathnumber(config.path_src);
            }
        }
    
        waypoints_per_meter_ = config.waypoints_per_meter;
        ROS_INFO_STREAM("waypoints per meter: " << waypoints_per_meter_);
        radius_ = config.radius;
        heading_mode_ = config.heading_mode;
    }

    bool PubPlanner::makePlan(const geometry_msgs::PoseStamped& star_pont, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        path_.poses.insert(path_.poses.begin(), star_pont);
        // std::cout << "receive goal!!! path size: " << path_.poses.size() << std::endl;

        if (path_.poses.size() < 2)
        {
            if (radius_ > 0)
            {
                double pi = 4 * atan(1.0);
                double circle_center_x;
                double circle_center_y;
                double x3, y3, x3_prime, y3_prime;
                circle_circle_intersection(star_pont.pose.position.x,star_pont.pose.position.y,radius_/1000,goal.pose.position.x,goal.pose.position.y,radius_/1000,
                             &x3, &y3, &x3_prime, &y3_prime);
                // std::cout << "circle: x: "<< x3 << ", y: "<< y3  << ", x1: "<< x3_prime << ", y1: "<< y3_prime  << std::endl;
                if (x3 == 0)
                {
                    std::cout << "circle: fail !!!!" << std::endl;
                    return true;
                }
                else
                {

                    double angle_point = atan2(y3-star_pont.pose.position.y, x3-star_pont.pose.position.x);
                    double angle_point_ = angle_point;
                    if (angle_point < 0)
                        angle_point_ = angle_point_ + 2*pi;

                    double angle_point_prime = atan2(y3_prime-star_pont.pose.position.y, x3_prime-star_pont.pose.position.x);
                    double angle_point_prime_ = angle_point_prime;
                    if (angle_point_prime < 0)
                        {angle_point_prime_ = angle_point_prime_ + 2*pi;}

                    double angle_current = tf::getYaw(star_pont.pose.orientation);
                    double angle_current_ = angle_current;
                    if (angle_current < 0)
                        angle_current_ = angle_current_ + 2*pi;

                    double first_angle = fabs(angle_point-angle_current);
                    if (first_angle>pi)
                        first_angle = 360 - first_angle;
                    double second_angle=fabs(angle_point_prime-angle_current);
                    if (second_angle > pi)
                        second_angle = 360 - second_angle;
                    // std::cout << "--angle: 3: "<< angle_point << " 3_prime: "<< angle_point_prime  << " current: "<< angle_current << std::endl;

                    double length_of_this_goal = hypot(goal.pose.position.x-star_pont.pose.position.x, goal.pose.position.y-star_pont.pose.position.y);
                    int point_num_of_this_goal = waypoints_per_meter_ * length_of_this_goal;
                    std::cout << "angle_current: "<< angle_current << ", angle_point: "<< angle_point << ", angle_point_prime: "<< angle_point_prime << std::endl;

                    if (first_angle > second_angle)
                    {
                        std::cout << "choose first Circle center:   x: " << x3 << " y: " << y3 << std::endl;
                        circle_center_x = x3;
                        circle_center_y = y3;
                    }
                    else
                    {
                        std::cout << "choose second Circle center:   x: " << x3_prime << " y: " << y3_prime << std::endl;
                        circle_center_x = x3_prime;
                        circle_center_y = y3_prime;
                    }

                    double angle_start = atan2(star_pont.pose.position.y-circle_center_y, star_pont.pose.position.x-circle_center_x);
                    double angle_goal = atan2(goal.pose.position.y-circle_center_y, goal.pose.position.x-circle_center_x);
                    std::cout << "angle_start: " << angle_start << ", angle_goal: "<< angle_goal << std::endl;

                    for (int i = 0; i < point_num_of_this_goal; i++)
                    {
                        int sign=1;
                        geometry_msgs::PoseStamped p  = star_pont;
                        p.header.frame_id ="map";
                        if(fabs(angle_goal-angle_start)>pi)
                        {
                            if (angle_goal < 0)
                                angle_goal = angle_goal + 2*pi;
                            if (angle_start < 0)
                                angle_start = angle_start + 2*pi;
                        }
                        p.pose.position.x = (radius_/1000.0) * cos(angle_start+(angle_goal-angle_start)*sign/point_num_of_this_goal*i) + circle_center_x;
                        p.pose.position.y = (radius_/1000.0) * sin(angle_start+(angle_goal-angle_start)*sign/point_num_of_this_goal*i) + circle_center_y;
                        std::cout << "angle_cal: " << angle_start+(angle_goal-angle_start)/point_num_of_this_goal*i << ", sin: "<< sin(angle_start+(angle_goal-angle_start)/point_num_of_this_goal*i) << ", cos: " << cos(angle_start+(angle_goal-angle_start)/point_num_of_this_goal*i) << std::endl;
                        path_.poses.push_back(p);
                    }
                }
            }
            path_.poses.insert(path_.poses.end(), goal);
            // std::cout << "receive goal!!! path size: " << path_.poses.size() << std::endl;
        }
        path_.header.frame_id = "map";
        interpolatePath(path_);
        plan_pub_.publish(path_);
        plan = path_.poses;
        ROS_INFO("Published global plan");
        path_.poses.clear();
        planing_from_ = 0;
        return true;
    }

    void PubPlanner::pathCallback(const std_msgs::String& name)
    {
        planing_from_ = 1;
        int pose = name.data.find_first_of(",", 0);
        std::cout << "re topic: " << pose << std::endl;
        if (pose == -1)
            PathFromGIMPsvg(name.data,0);
        else
            PathFromGIMPsvg(name.data.substr(0, pose), atof(name.data.substr(pose+1, pose).c_str()));
    
        // std::cout << pose << std::endl;
        // std::cout << name.data.substr(0, pose) << std::endl;
        // std::cout << name.data.substr(pose+1, pose) << std::endl;
        // PathFromGIMPsvg(name, yaw.data);
    }

    void PubPlanner::PathFromGIMPsvg( std::string name, float yaw )
    {
        if (clear_waypoints_)
        {
            waypoints_.clear();
            clear_waypoints_ = false;
        }
        // Read xml
        rapidxml::file<> fdoc(svg_file_path_.c_str());//**************user input parameter
        rapidxml::xml_document<> doc;
        doc.parse<0>(fdoc.data());  // parse
        rapidxml::xml_node<> * svg_node=doc.first_node();
        int curve_flag=0;
        int curve_count=0;
        int path_count=0;
        // int path_num_p=6;  // **************user input parameter
	    int i,j,aa;
        aa=0;
        float cp1x,cp1y,cp2x,cp2y,lx,ly,nx,ny;
        // std::cout << svg_node->first_attribute("viewBox")->value() << std::endl;
        std::string size_value=svg_node->first_attribute("viewBox")->value();
        char *s_view;
        char * strs_view = new char[size_value.length() + 1] ; 
	    strcpy(strs_view, size_value.c_str());
        const char * const delim_view = " ";
        float *data_view= new float[4];
        bool no_path_flag=1;
        for(s_view = strtok(strs_view, delim_view); s_view; s_view = strtok(NULL,delim_view))
        {
            std::string A(s_view);
            data_view[aa]=atof(A.c_str());
            aa++;
        }

        for (rapidxml::xml_node<> * node = svg_node->first_node(); node; node = node->next_sibling())
        {
	    	std::string path_value=node->first_attribute("d")->value();
            path_count++;
            char * strs = new char[path_value.length() + 1] ; 
	        strcpy(strs, path_value.c_str());
	    	const char * const delim = "\n ";
	    	char *p;
        
            if (!strcmp(node->first_attribute("id")->value(), name.c_str()))
            {
                no_path_flag=0;
                // std::cout <<node->first_attribute("id")->value()  << std::endl;
                ROS_INFO_STREAM("find path:" <<node->first_attribute("id")->value());
                for (p = strtok(strs, delim); p; p = strtok(NULL,delim))
                {
                    // std::cout << p << std::endl;
                    if (!strcmp(p, "M"))
                    {
                        curve_flag=0; 
                        curve_count=0;
                    }
                    else if(!strcmp(p, "C"))
                    {
                        curve_flag=1;
                        curve_count=0;
                    }

                    if (curve_flag==0 && strcmp(p, "C") && strcmp(p, "M")) 
                    {
                        std::string A(p);
                        int pose = A.find_first_of(",", 0);
                        nx=atof(A.substr(0, pose).c_str());
                        ny=atof(A.substr(pose+1, pose).c_str());
                        // std::cout << "start point:" <<nx<<" , "<< ny<< std::endl;
                        waypoints_.push_back(geometry_msgs::PoseStamped());
                        ros::Time now = ros::Time::now();
                        waypoints_.back().header.stamp =  now;
                        waypoints_.back().header.frame_id ="map";
                        waypoints_.back().pose.position.x = nx*map_resolution;
                        waypoints_.back().pose.position.y = (data_view[3]-ny)*map_resolution;
                        waypoints_.back().pose.position.z = 0;
                        waypoints_.back().pose.orientation.w = 1.0;
                        ROS_INFO_STREAM("start point:" <<nx*map_resolution <<" , "<<(data_view[3]-ny)*map_resolution);
                    }

                    if (curve_count==3 && strcmp(p, "C"))
                    {
                        // std::cout << curve_count <<","<< p << std::endl;
                        lx=nx;
                        ly=ny;
                        std::string A(p);
                        int pose =A.find_first_of(",", 0);
                        nx=atof(A.substr(0, pose).c_str());
                        ny=atof(A.substr(pose+1, pose).c_str());
                        // std::cout <<"cp3x--:"<< nx <<"  cp3y--:"<< ny  << std::endl;
                        // std::cout << path_count << std::endl;

                        if (cp1x==lx &&  cp2x==nx && cp1y==ly && cp2y==ny )
                        {
                            //std::cout <<"get1" << std::endl;
                            waypoints_.push_back(geometry_msgs::PoseStamped());
                            ros::Time now = ros::Time::now();
                            waypoints_.back().header.stamp =  now;
                            waypoints_.back().header.frame_id =  "map";
                            waypoints_.back().pose.position.x = nx*map_resolution;
                            waypoints_.back().pose.position.y = (data_view[3]-ny)*map_resolution;
                            waypoints_.back().pose.position.z = 0;
                            waypoints_.back().pose.orientation.w = 1.0;
                        }
                        else
                        {
                            for(float i = 0 ; i < 1 ; i += 0.025)
                            {
                                ros::Time now = ros::Time::now();
                                float xx= getcurvepoint( lx , cp1x , cp2x  , nx , i );
                                float yy= getcurvepoint( ly , cp1y , cp2y  , ny , i );
                                waypoints_.push_back(geometry_msgs::PoseStamped());
                                waypoints_.back().header.stamp =  now;
                                waypoints_.back().header.frame_id ="map";
                                waypoints_.back().pose.position.x = xx*map_resolution;
                                waypoints_.back().pose.position.y = (data_view[3]-yy)*map_resolution;
                                waypoints_.back().pose.position.z = 0;
                                waypoints_.back().pose.orientation.w = 1.0; 
                            }
                            ros::Time now = ros::Time::now();
                            waypoints_.push_back(geometry_msgs::PoseStamped());
                            waypoints_.back().header.stamp =  now;
                            waypoints_.back().header.frame_id = "map";
                            waypoints_.back().pose.position.x = nx*map_resolution;
                            waypoints_.back().pose.position.y = (data_view[3]-ny)*map_resolution;
                            waypoints_.back().pose.position.z = 0;
                            waypoints_.back().pose.orientation.w = 1.0;
                        }
                    }

                    if (curve_flag == 1)
                    {
                        if (curve_count == 1)
                        {
                            std::string A(p);
                            int pose =A.find_first_of(",", 0);
                            cp1x=atof(A.substr(0, pose).c_str());
                            cp1y=atof(A.substr(pose+1, pose).c_str());
                            // std::cout <<"cp1x:"<< cp1x <<"  cp1y:"<< cp1y  << std::endl;
                        }
                        else if (curve_count==2)
                        {
                            std::string A(p);
                            int pose =A.find_first_of(",", 0);
                            cp2x=atof(A.substr(0, pose).c_str());
                            cp2y=atof(A.substr(pose+1, pose).c_str());
                            // std::cout <<"cp2x:"<< cp2x <<"  cp2y:"<< cp2y  << std::endl;
                        }
                        curve_count++;
                    }            
                    else
                        curve_count=0;

                    if (curve_count >= 4)
                        curve_count = 1;
                    // std::cout << curve_flag<<"," << data[path_count-1][2]<<","<< data[path_count-1][3]<< std::endl;
                }
                // std::cout << "end point:" << data[path_count-1][2]<<","<< data[path_count-1][3]<< std::endl;
            }
        }

        if(no_path_flag == 0)
        {
            createAndPublishMarkersFromPath(waypoints_);
            waypoints_.back().pose.orientation=tf::createQuaternionMsgFromYaw(yaw);
            ros::Time now = ros::Time::now();
            path_.header.stamp =  now;
            path_.header.frame_id ="map";
            path_.poses.clear();
            path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
            std::cout << "path length:" << path_.poses.size()<< std::endl;
            goal_pub_.publish(waypoints_.back());
            clear_waypoints_ = true;
            ROS_INFO("Published goal pose");
        }
    }

    void PubPlanner::interpolatePath(nav_msgs::Path& path)
    {
        std::vector<geometry_msgs::PoseStamped> temp_path;
        std::cout << "path length(interpolatePath): " << static_cast<int>(path.poses.size()) << std::endl;
        geometry_msgs::PoseStamped p;
        for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
        {
            // std::cout << "path (interpolatePath) i: " <<  i<< std::endl;
            // std::cout << "path (interpolatePath) x: " <<  path.poses[i].pose.position.x << std::endl;
            // std::cout << "path (interpolatePath) y: " <<  path.poses[i].pose.position.y << std::endl;
            // std::cout << "path (interpolatePath) nx: " <<  path.poses[i+1].pose.position.x << std::endl;
            // std::cout << "path (interpolatePath) ny: " <<  path.poses[i+1].pose.position.y << std::endl;

            // calculate distance between two consecutive waypoints
            double x1 = path.poses[i].pose.position.x;
            double y1 = path.poses[i].pose.position.y;
            double x2 = path.poses[i+1].pose.position.x+0.00000000001;
            double y2 = path.poses[i+1].pose.position.y+0.00000000001;
            double dist = hypot(x1-x2, y1-y2);
            int num_wpts = dist * waypoints_per_meter_;
            int heading_mode_c_;

            if (i == 0)
                temp_path.push_back(path.poses[i]);

            if (planing_from_ == 2)
            {
                std::cout << "planing_from_: " <<  planing_from_ << std::endl;
                std::cout << "series_path_turn_mode_: " <<  series_path_turn_mode_[i] << std::endl;
                heading_mode_c_ = series_path_turn_mode_[i];
            }
            else
                heading_mode_c_ = heading_mode_;
            
            if (heading_mode_c_ == 1)
                p = path.poses[i+1];
            else if (heading_mode_c_ == 2)
                p = path.poses[i];
            else if (heading_mode_c_ == 3)
            {
                double pi = 4 * atan(1.0);
                double dx_ = x2 - x1;
                double dy_ = y2 - y1;

                if (dx_ == 0 && dy_ == 0)
                    p = path.poses[i+1];
                else
                {
                    double heading_angle_ = std::atan2(dy_, dx_);
                    if (heading_angle_ > pi)
                        heading_angle_ = heading_angle_ - 2 * pi;
                    else if (heading_angle_ < -pi)
                        heading_angle_ = heading_angle_ + 2 * pi;
                    
                    p = path.poses[i+1];
                    p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, heading_angle_);
                }                
            }
            else if(heading_mode_c_==4){
                p = path.poses[i-1];             
            }

            if (num_wpts >= 1)
            {
                for (int j = 1; j <= num_wpts; j++)
                {
                    p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
                    p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
                    if(x2-x1 != 0 && y2-y1 != 0)
                        temp_path.push_back(p);
                }
            }
            else
            {
                p.pose.position.x = x2;
                p.pose.position.y = y2;
            }
        }

        // update sequence of poses
        for (size_t i = 0; i < temp_path.size(); i++)
            temp_path[i].header.seq = static_cast<int>(i);
    
        temp_path.push_back(path.poses.back());
        path.poses = temp_path;
    }

    void PubPlanner::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        // clear previous markers
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;
        marker.header = path[0].header;
        marker.ns = "/move_base/PubPlanner";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.id = 0;
        markers.markers.push_back(marker);
        PubPlanner_marker_pub_.publish(markers);
        marker.action = visualization_msgs::Marker::ADD;
        markers.markers.clear();

        for (size_t i = 0; i < path.size(); i++)
        {
            marker.id = i;
            marker.pose.position = path[i].pose.position;
            markers.markers.push_back(marker);
        }

        PubPlanner_marker_pub_.publish(markers);
    }

    float PubPlanner::getPt( float n1 , float n2 , float perc)
    {
        float diff = n2 - n1;
        return n1 + ( diff * perc );
    }  

    float PubPlanner::getcurvepoint(float x1 ,float x2 ,float x3  ,float x4 , float i)
    {
        // The Green Lines
        float xa = getPt(x1 , x2 , i);
        float xb = getPt(x2 , x3 , i);
        float xc = getPt(x3 , x4 , i);
        // The Blue Line
        float xm = getPt(xa , xb , i);
        float xn = getPt(xb , xc , i);
        // The Black Dot
        return  getPt( xm , xn , i );
    }

    int PubPlanner::getpathnumber( std::string path_src )
    {
        // Read xml
        rapidxml::file<> fdoc(path_src.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(fdoc.data());//parse
        rapidxml::xml_node<> * svg_node = doc.first_node();
        int path_count=0;

        for (rapidxml::xml_node<> * node = svg_node->first_node(); node; node = node->next_sibling())
            path_count++;
        
        std::cout << "there has:[ " << path_count << " ]path in this file" << std::endl;
        ROS_INFO_STREAM("there has:[ " << path_count << " ] path in this file");
        return path_count;
    }

    int PubPlanner::circle_circle_intersection(double x0, double y0, double r0,
                                               double x1, double y1, double r1,
                                               double *xi, double *yi,
                                               double *xi_prime, double *yi_prime)
    {
        double a, dx, dy, d, h, rx, ry;
        double x2, y2;

        /* dx and dy are the vertical and horizontal distances between
        * the circle centers.
        */
        dx = x1 - x0;
        dy = y1 - y0;

        /* Determine the straight-line distance between the centers. */
        // d = sqrt((dy*dy) + (dx*dx));
        d = hypot(dx,dy);  // Suggested by Keith Briggs

        /* Check for solvability. */
        if (d > (r0 + r1))
        {
            /* no solution. circles do not intersect. */
            return 0;
        }
        if (d < fabs(r0 - r1))
        {
            /* no solution. one circle is contained in the other */
            return 0;
        }

        /* 'point 2' is the point where the line through the circle
        * intersection points crosses the line between the circle
        * centers.  
        */

        /* Determine the distance from point 0 to point 2. */
        a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d);

        /* Determine the coordinates of point 2. */
        x2 = x0 + (dx * a/d);
        y2 = y0 + (dy * a/d);

        /* Determine the distance from point 2 to either of the
        * intersection points.
        */
        h = sqrt((r0*r0) - (a*a));

        /* Now determine the offsets of the intersection points from
        * point 2.
        */
        rx = -dy * (h/d);
        ry = dx * (h/d);

        /* Determine the absolute intersection points. */
        *xi = x2 + rx;
        *xi_prime = x2 - rx;
        *yi = y2 + ry;
        *yi_prime = y2 - ry;

        return 1;
    }
}
