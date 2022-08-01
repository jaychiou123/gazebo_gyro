#include <manipulator_class/manipulator_class.h>

#define TIMEOUT 0.5
#define linear_tolerance_sq_ 0.001
#define angular_tolerance_sq_ 0.001




Manipulator::Manipulator(std::string name):as_(nh_, name, 
                                                boost::bind(&Manipulator::executeCB, this, _1), false)
{
    ros::NodeHandle private_nh;
    vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    as_.start();
}

Manipulator::~Manipulator()
{
    delete [] tfListener;
}

void Manipulator::executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    ROS_INFO("Receive the goal!!");
    geometry_msgs::TransformStamped base_relative_to_input, map_to_base;
    geometry_msgs::PoseStamped goal_pose;
    ros::Duration timeout = ros::Duration(TIMEOUT);
    ros::Rate r(30);
    
    velocity = 0.0;
    // in order to transform transformstamped into tranformation matrix
    tf2::Stamped<tf2::Transform> base_relative_to_input_tm;
    tf2::Stamped<tf2::Transform> goal_tf2;

    // input frame: reference frame
    std::string input_frame = goal->target_pose.header.frame_id;

    time_initial = ros::Time::now();
    // do the transform and get the difference under base_frame
    try
    {
        goal_pose = goal->target_pose;
        // this is a transformation from base_footprint to input
        base_relative_to_input = tfBuffer.lookupTransform("base_footprint", input_frame, time_initial, timeout);
        // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
        tf2::convert(base_relative_to_input,base_relative_to_input_tm);
        tf2::convert(goal_pose, goal_tf2);
    }
    catch(tf2::TransformException &ex)
    {
        ROS_ERROR_STREAM("Abort, goal invalid. " << ex.what());
        stop_vel();
        as_.setAborted(result_, ex.what());
        return;
    }

    while(ros::ok())
    {
        if(as_.isPreemptRequested())
        {
            ROS_WARN_STREAM("Action cancelled.");
            as_.setPreempted(result_, "Action cancelled.");
            return;
        }

        // continuously do the transform and get the difference under base_frame
        // and check if we achieve within the tolerance.
        try
        {
            time_each = ros::Time::now();

            //show it's position w.r.t. map
            map_to_base = tfBuffer.lookupTransform("map", "base_footprint", time_each, timeout);
            std::cout << "x = "  << map_to_base.transform.translation.x << std::endl;
            std::cout << "y = "  << map_to_base.transform.translation.y << std::endl;
            std::cout << "th = "  << map_to_base.transform.rotation.z << std::endl;

            // update the transformation
            base_relative_to_input = tfBuffer.lookupTransform("base_footprint", input_frame, time_each, timeout);
            tf2::convert(base_relative_to_input, base_relative_to_input_tm);

            // To accommodate the transformation info respect to base_frame
            tf2::Transform diff;
            diff = base_relative_to_input_tm * goal_tf2;
            diff_x = diff.getOrigin().x();
            diff_y = diff.getOrigin().y();
            diff_th = tf2::getYaw(diff.getRotation());
            
            if (diff_x * diff_x + diff_y * diff_y <= linear_tolerance_sq_ && fabs(diff_th) <= angular_tolerance_sq_)
            {
                ROS_INFO_STREAM("diff.getOrigin().x(): " << diff.getOrigin().x() << " diff.getOrigin().y(): " << diff.getOrigin().y() );
                ROS_INFO_STREAM("Success");
                stop_vel();
                as_.setSucceeded(result_, "Success");
                return;
            }
        }
        catch(tf2::TransformException &ex)
        {
            ROS_ERROR_STREAM("Abort. " << ex.what());
            stop_vel();
            as_.setAborted(result_, ex.what());
            return; 
        }

        // fill in feedback data
        feedback_.base_position.header = goal->target_pose.header;
        feedback_.base_position.header.frame_id = "base_footprint";
        feedback_.base_position.pose.position.x = diff_x;
        feedback_.base_position.pose.position.y = diff_y;
        tf2::Quaternion quat;
        quat.setRPY(0,0,diff_th);
        feedback_.base_position.pose.orientation.w = quat.w();
        feedback_.base_position.pose.orientation.x = quat.x();
        feedback_.base_position.pose.orientation.y = quat.y();
        feedback_.base_position.pose.orientation.z = quat.z();
        as_.publishFeedback(feedback_);

        // calculate velocity(geometry_msgs::Twist) and publish it
        x_.max_vel = 0.5; x_.min_vel = -0.5; x_.acceleration = 0.1;
        y_.max_vel = 0.5; y_.min_vel = -0.5; y_.acceleration = 0.1;
        theta_.max_vel = 0.5; theta_.min_vel = -0.5; theta_.acceleration = 0.08;
        geometry_msgs::Twist pub_vel;
        pub_vel.linear.x = cal_vel(diff_x, x_);
        pub_vel.linear.y = cal_vel(diff_y, y_);
        pub_vel.angular.z = cal_vel(diff_th, theta_);
        vel_pub_.publish(pub_vel);


        r.sleep();
    }
}

double Manipulator::cal_vel(double difference, velocity_setting &set)
{
  int sign = 1;
  double move_time_now = (time_each - time_initial).toSec();
  if (difference == 0.0)
  {
    return 0;
  }
  else if (difference < 0.0)
  {
    sign = -1;
  }

  velocity = sign * sqrt(fabs(difference) * 2 * set.acceleration);

  if (velocity > set.max_vel)
  {
    velocity = set.max_vel;
  }
  else if (velocity < set.min_vel)
  {
    velocity = set.min_vel;
  }
  return velocity;
}
void Manipulator::stop_vel()
{
    geometry_msgs::Twist pub_vel;
    vel_pub_.publish(pub_vel);
}
/* Todo 
1. Wait for client sent goal which is the goal.target_frame and goal.target_pose
2. Keep transforming the goal frame to base_footprint and keep track of the 
    distance between pose and poistion
3. Calculate the adequate velocity to pursue the target pose and position
4. Keep publishing feedback and finally publish result.  
*/