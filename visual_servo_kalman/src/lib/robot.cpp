#include "robot.h"
robot::robot()
{
    refFrame="base_link";
    childFrame="tool0";
}
bool robot::getData()
{

    if(getTransform()==false) return false;
    if(getPic()==false) return false;

    return true;
}

bool robot::getTransform()
{
    tf::TransformListener _tfListener;
    tf::StampedTransform transform;
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(200),
                                       ros::Duration(0.1),
                                       &errMsg)
            )
    {
        cout<<"is fasle 1 "<<errMsg<<endl;
        ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
        return false;
    }
    else
    {
        try
        {
            _tfListener.lookupTransform( refFrame, childFrame,
                                         ros::Time(0),                  //get latest available
                                         transform);
        }
        catch ( const tf::TransformException& e)
        {
            cout<<"is fasle 2"<<endl;
            ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
            return false;
        }

   }
    Rotation rx=Rotation::Quaternion(transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    Vector vx(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    frame.p = vx;
    frame.M = rx;

    return true;
}

bool robot::getPic()
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImageConstPtr image_sub;

    image_sub = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/ir/image_raw",ros::Duration(3));
    if(image_sub != NULL)
    {
        cv_ptr = cv_bridge::toCvCopy(image_sub);
        image = cv_ptr -> image;
        ROS_INFO("image ok!");
    }
    else
    {
        cout<<"no topic image_sub"<<endl;
        return false;
    }

    return true;
}


void robot::move_robots(moveit::planning_interface::MoveGroupInterface *arm, geometry_msgs::Pose target_pose)
{
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        
        arm->move();
        // 执行运动
        arm->execute(plan);
    }
        else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
}