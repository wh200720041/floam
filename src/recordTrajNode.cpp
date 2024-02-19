#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_listener.h"
#include <tf/tf.h>

#include <iostream>
#include <fstream>

class PathContainer
{
public:
    PathContainer()
    {
        ros::NodeHandle private_nh("~");

        private_nh.param("target_frame_name", p_target_frame_name_, std::string("map"));
        private_nh.param("source_frame_name", p_source_frame_name_, std::string("base_link"));
        private_nh.param("trajectory_update_rate", p_trajectory_update_rate_, 4.0);
        private_nh.param("filename", filename, std::string("traj.txt"));
        fout.open(filename, std::ios::out);
        if (!fout.is_open())
        {
            std::cout << "cannot open " << filename << std::endl;
            return;
        }
        std::cout << "open " << filename << std::endl;
        waitForTf();

        ros::NodeHandle nh;
        last_reset_time_ = ros::Time::now();

        update_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_update_rate_), &PathContainer::trajectoryUpdateTimerCallback, this, false);

        pose_source_.pose.orientation.w = 1.0;
        pose_source_.header.frame_id = p_source_frame_name_;

        trajectory_.header.frame_id = p_target_frame_name_;
    }

    void waitForTf()
    {
        ros::Time start = ros::Time::now();
        ROS_INFO("Waiting for tf transform data between frames %s and %s to become available", p_target_frame_name_.c_str(), p_source_frame_name_.c_str());

        bool transform_successful = false;

        while (!transform_successful)
        {
            transform_successful = tf_.canTransform(p_target_frame_name_, p_source_frame_name_, ros::Time());
            if (transform_successful)
                break;

            ros::Time now = ros::Time::now();

            if ((now - start).toSec() > 20.0)
            {
                ROS_WARN_ONCE("No transform between frames %s and %s available after %f seconds of waiting. This warning only prints once.", p_target_frame_name_.c_str(), p_source_frame_name_.c_str(), (now - start).toSec());
            }

            if (!ros::ok())
                return;
            ros::WallDuration(1.0).sleep();
        }

        ros::Time end = ros::Time::now();
        ROS_INFO("Finished waiting for tf, waited %f seconds", (end - start).toSec());
    }

    void addCurrentTfPoseToTrajectory()
    {
        pose_source_.header.stamp = ros::Time(0);

        geometry_msgs::PoseStamped pose_out;

        tf_.transformPose(p_target_frame_name_, pose_source_, pose_out);

        if (trajectory_.poses.size() != 0)
        {
            // Only add pose to trajectory if it's not already stored
            if (pose_out.header.stamp != trajectory_.poses.back().header.stamp)
            {
                trajectory_.poses.push_back(pose_out);
            }
        }
        else
        {
            trajectory_.poses.push_back(pose_out);
        }

        trajectory_.header.stamp = pose_out.header.stamp;
    }

    void trajectoryUpdateTimerCallback(const ros::TimerEvent &event)
    {

        try
        {
            addCurrentTfPoseToTrajectory();
        }
        catch (tf::TransformException e)
        {
            ROS_WARN("Trajectory Server: Transform from %s to %s failed: %s \n", p_target_frame_name_.c_str(), pose_source_.header.frame_id.c_str(), e.what());
        }
    }

    void saveTraj()
    {
        for (auto msg : trajectory_.poses)
            fout << std::to_string(msg.header.stamp.toSec())
                 << " " << msg.pose.position.x << " " << msg.pose.position.y << " " << msg.pose.position.z
                 << " " << msg.pose.orientation.x << " " << msg.pose.orientation.y << " " << msg.pose.orientation.z
                 << " " << msg.pose.orientation.w << std::endl;
        fout.close();
    }

    // parameters
    std::string p_target_frame_name_;
    std::string p_source_frame_name_;
    std::string filename;
    double p_trajectory_update_rate_;

    geometry_msgs::PoseStamped pose_source_;
    nav_msgs::Path trajectory_;
    ros::Timer update_trajectory_timer_;
    tf::TransformListener tf_;

    ros::Time last_reset_time_;
    ros::Time last_pose_save_time_;

    std::fstream fout;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recordTraj");

    PathContainer pc;
    ros::spin();
    pc.saveTraj();
    return 0;
}