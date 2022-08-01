#include <tf_move_relative/tf_move_relative.h>

tf_MoveRelative::tf_MoveRelative(std::string name) : action_server_(nh_, 
                                                                    name,
                                                                    boost::bind(&tf_MoveRelative::execute, this, _1),
                                                                    false)
{
    ros::NodeHandle private_nh;
    
    //goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);

    //####################### why the namespace is turtle2? ###########################333#####333
    vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    //#########################################################################################

    dynamic_reconfigure::Server<tf_move_relative::tf_MoveRelativeConfig>::CallbackType f;

    f = boost::bind(&tf_MoveRelative::reconfig_callback, this, _1, _2);
    reconfigure_server_.setCallback(f);

    // ##################### don't need to delete? ###################################################
    tf_listener_ = new tf2_ros::TransformListener(tf_Buffer_);                
    ///#########################################################################################
    action_server_.start();
}

tf_MoveRelative::~tf_MoveRelative()
{

}

void tf_MoveRelative::reconfig_callback(tf_move_relative::tf_MoveRelativeConfig &config, uint32_t level)
{
    rate_ = config.rate;
    timeout_ = config.timeout;
    base_frame_ = config.base_frame;
    fixed_frame_ = config.fixed_frame;
    x_.max_vel = config.max_vel_x;
    x_.min_vel = config.min_vel_x;
    y_.max_vel = config.max_vel_y;
    y_.min_vel = config.min_vel_y;
    theta_.max_vel = config.max_vel_theta;
    theta_.min_vel = config.min_vel_theta;
    x_.acceleration = config.acceleration_x;
    y_.acceleration = config.acceleration_y;
    theta_.acceleration = config.acceleration_theta;
    linear_tolerance_sq_ = config.linear_tolerance * config.linear_tolerance;
    angular_tolerance_ = config.angular_tolerance;
}

void tf_MoveRelative::execute(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    ROS_INFO("Recieved goal");
    move_base_msgs::MoveBaseResult result;
    move_base_msgs::MoveBaseFeedback feedback;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::TransformStamped move_relative_fixed_transform;
    geometry_msgs::TransformStamped base_odom_initial_transform;

  //##############33 tranform to transformation type which can be used to multiply ########################
    tf2::Stamped<tf2::Transform> move_relative_fixed_tf2;
    tf2::Stamped<tf2::Transform> base_odom_initial_tf2;
    tf2::Stamped<tf2::Transform> goal_tf2;
  //###############################################################

  //################## tf2::Transform is a rotation matrix and translation data type################################33
    tf2::Transform diff_initial;
  //#############################################################################################333333333##
    double move_to_maxvelx;
    double move_to_maxvely;
    double move_to_maxvelth;

    ros::Rate r(rate_);
    ros::Duration timeout = ros::Duration(timeout_);

    std::string move_relative_frame_ = goal->target_pose.header.frame_id;     
    ROS_INFO("Move_relative_frame = %s", move_relative_frame_.c_str());

    //goal_pub_.publish(goal->target_pose);

    time_initial = ros::Time::now();

    try
    {
//###################################################################################333
//  because goal->target_pose.frame_id = move_relative_frame_, therefore the following line only means goal_pose = goal->target_pose
      //tf_Buffer_.transform(goal->target_pose, goal_pose, move_relative_frame_);
//##################################################################################3

      goal_pose = goal->target_pose;

      // lookupTransform (parent, child, ...)
      // fixed_frame_ = world  
      // move_relative_frame_ = input
      // base_frame_ = turtle2
      move_relative_fixed_transform = tf_Buffer_.lookupTransform(fixed_frame_, move_relative_frame_, time_initial, timeout);
      base_odom_initial_transform = tf_Buffer_.lookupTransform(base_frame_, fixed_frame_, time_initial, timeout);

      tf2::convert(move_relative_fixed_transform, move_relative_fixed_tf2); // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
      tf2::convert(base_odom_initial_transform, base_odom_initial_tf2); // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
      tf2::convert(goal_pose, goal_tf2);

//*************************************** 
//*****  here is R(T2, world) * R(world, input) * P(ref to Input frame) = P(ref to turtle2)
//***************************************
      diff_initial = base_odom_initial_tf2 * move_relative_fixed_tf2 * goal_tf2;
      diff_initial_x = diff_initial.getOrigin().x();
      diff_initial_y = diff_initial.getOrigin().y();
      diff_initial_th = tf2::getYaw(diff_initial.getRotation());

//***********************************
// here is some mysterious calculation, remain unknown
//***********************************
      move_to_maxvelx = sqrt(fabs(diff_initial_x) / x_.acceleration);
      move_to_maxvely = sqrt(fabs(diff_initial_y) / y_.acceleration);
      move_to_maxvelth = sqrt(fabs(diff_initial_th) / theta_.acceleration);

      if(x_.acceleration == 0 || y_.acceleration == 0 || theta_.acceleration == 0)
      {
        ROS_ERROR_STREAM("Abort because of set acceleration is zero.");
        return;
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Abort, goal invalid. " << ex.what());
      action_server_.setAborted(result, ex.what());
      return;
    }

    velocity = 0.0;

    while(ros::ok())
    {
      if (action_server_.isPreemptRequested())
      {
        stop_vel();
        ROS_WARN_STREAM("Action canceled.");
        action_server_.setPreempted(result, "Action canceled");
        return;
      }

      geometry_msgs::TransformStamped fixed_base_transform;

      try
      {
        now = ros::Time::now();
        // lookupTransform (parent, child, ...)
        fixed_base_transform = tf_Buffer_.lookupTransform(base_frame_, fixed_frame_, now, timeout);
      }
      catch (tf2::TransformException &ex)
      {
        stop_vel();
        ROS_ERROR_STREAM("Abort. " << ex.what());
        action_server_.setAborted(result, ex.what());
        return;
      }

      tf2::Stamped<tf2::Transform> fixed_base_tf2;

      // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
      tf2::convert(fixed_base_transform, fixed_base_tf2);

      tf2::Transform diff_tf2;

//*****************************************************
// R(turtle2, world) * R(world, input) * P(ref to in) = P(ref to turtle2)
// used for calculating last tolerance
//****************************************************
      diff_tf2 = fixed_base_tf2 * move_relative_fixed_tf2 * goal_tf2;

      double diff_x = diff_tf2.getOrigin().x();
      double diff_y = diff_tf2.getOrigin().y();
      double diff_yaw = tf2::getYaw(diff_tf2.getRotation());
      if (diff_x * diff_x + diff_y * diff_y <= linear_tolerance_sq_ && fabs(diff_yaw) <= angular_tolerance_)
      {
        ROS_INFO_STREAM("diff_tf2.getOrigin().x(): " << diff_tf2.getOrigin().x() << " diff_tf2.getOrigin().y(): " << diff_tf2.getOrigin().y() );
        stop_vel();
        ROS_INFO_STREAM("Success");
        action_server_.setSucceeded(result, "Success");
        return;
      }

      feedback.base_position.header =  goal->target_pose.header;
      feedback.base_position.header.frame_id =  base_frame_;
      feedback.base_position.pose.position.x = diff_x;
      feedback.base_position.pose.position.x = diff_y;
      tf2::Quaternion quat;
      quat.setRPY(0, 0, diff_yaw);
      feedback.base_position.pose.orientation.w = quat.w();
      feedback.base_position.pose.orientation.x = quat.x();
      feedback.base_position.pose.orientation.y = quat.y();
      feedback.base_position.pose.orientation.z = quat.z();

      // tf2::toMsg(fixed_reference_tf2, feedback.base_position); // this works
      // tf2::convert(fixed_reference_tf2, feedback.base_position); // this does not work, why?
      action_server_.publishFeedback(feedback);

      geometry_msgs::Twist pub_vel;
      pub_vel.linear.x = cal_vel(move_to_maxvelx, diff_x, x_);
      pub_vel.linear.y = cal_vel(move_to_maxvely, diff_y, y_);
      pub_vel.angular.z = cal_vel(move_to_maxvelth, diff_yaw, theta_);
      vel_pub_.publish(pub_vel);

      r.sleep();
    }
}

void tf_MoveRelative::stop_vel()
{
    geometry_msgs::Twist pub_vel;
    vel_pub_.publish(pub_vel);
}

double tf_MoveRelative::cal_vel(double move_to_maxvel, double difference, velocity_setting &set)
{
  int sign = 1;
  move_time_now = (now - time_initial).toSec();

  if (difference == 0.0)
  {
    return 0;
  }
  else if (difference < 0.0)
  {
    sign = -1;
  }

  if(move_time_now <= move_to_maxvel)
  {
    velocity = sign * set.acceleration * move_time_now;
  }
  else
  {
    velocity = sign * sqrt(fabs(difference) * 2 * set.acceleration);
  }

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