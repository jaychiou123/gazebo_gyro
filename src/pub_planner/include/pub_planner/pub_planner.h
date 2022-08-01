/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <pub_planner/pub_plannerConfig.h>
#include <pub_planner/Series_path.h>
#include <cmath>

using std::string;

#ifndef PUB_PLANNER_CPP
#define PUB_PLANNER_CPP

namespace pub_planner {

  struct station_point_to_get_radius
  {
    double x;
    double y;
  };

  class PubPlanner : public nav_core::BaseGlobalPlanner {
    public:
      PubPlanner();
      PubPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      ros::ServiceServer Series_path_srv_;
      /** overridden classes from interface nav_core::BaseGlobalPlanner **/
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& star_pont,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                   );
      void pathCallback(const std_msgs::String& name);
      void interpolatePath(nav_msgs::Path& path);
      void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);
      void PathFromGIMPsvg(std::string name, float yaw);
      float getPt(float n1, float n2, float perc);
      float getcurvepoint(float x1, float x2, float x3, float x4, float i);
      int getpathnumber(std::string path_src);
      int circle_circle_intersection(double x0, double y0, double r0,
                                     double x1, double y1, double r1,
                                     double *xi, double *yi,
                                     double *xi_prime, double *yi_prime);
      bool Series_path(pub_planner::Series_path::Request &req,
                       pub_planner::Series_path::Response &res);

    private:
      void reconfigureCB(pub_plannerConfig &config, uint32_t level);
	    dynamic_reconfigure::Server<pub_plannerConfig> *dsrv_;

      ros::Subscriber pathname_sub_;
      ros::Publisher PubPlanner_marker_pub_;  // publisher of waypoint visualization markers
      ros::Publisher goal_pub_;  // publisher of goal corresponding to the final waypoint
      ros::Publisher plan_pub_;  // publisher of the global plan

      std::vector<geometry_msgs::PoseStamped> waypoints_;
      ros::Time last_time_;
      nav_msgs::Path path_;  // container for the generated interpolated path
      float map_resolution;
      int waypoints_per_meter_;  // number of waypoints per meter of generated path used for interpolation
      double radius_;  // radius of path. in mm
      // bool dir_;  // direction of path.
      int heading_mode_;  // direction of path.
      bool clear_waypoints_;  // flag indicating that the waypoint container must be cleared to start anew
      int planing_from_;  // flag indicating that this make plane is form which source
      std::vector<int> series_path_radius_;
      std::vector<int> series_path_turn_mode_;
      std::vector<station_point_to_get_radius> station_list_to_get_radius_;

      std::string svg_file_path_;
      std::string topic_path_name;
      int path_num_p;
  };
};

#endif
