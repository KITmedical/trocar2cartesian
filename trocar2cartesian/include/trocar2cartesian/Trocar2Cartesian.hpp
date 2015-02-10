#ifndef _TROCAR2CARTESIAN_H_
#define _TROCAR2CARTESIAN_H_

// system includes

// library includes
#include <ros/ros.h>
#include <tf/transform_listener.h>

// custom includes
#include <trocar2cartesian_msgs/TrocarPose.h>
#include <trocar2cartesian_msgs/SetTrocar.h>

// forward declarations



class Trocar2Cartesian
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    Trocar2Cartesian(const std::string& baseTfName, const std::string& flangeTfName);

    // overwritten methods

    // methods

    // variables


  protected:
    // methods

    // variables


  private:
    // methods
    void getCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg);
    bool setTrocarCallback(trocar2cartesian_msgs::SetTrocar::Request& request, trocar2cartesian_msgs::SetTrocar::Response& response);
    void setTrocarPoseCallback(const trocar2cartesian_msgs::TrocarPose::ConstPtr& trocarMsg);

    // variables
    ros::NodeHandle m_node;
    ros::Publisher m_setCartesianTopicPub;
    ros::Subscriber m_getCartesianTopicSub;
    ros::ServiceServer m_setTrocarService;
    tf::TransformListener m_tfListener;

    std::string m_baseTfName;
    std::string m_flangeTfName;
    geometry_msgs::Pose m_trocarPose;
    geometry_msgs::Pose m_lastCartesianPose;
    geometry_msgs::Pose m_targetCartesianPose;
    std::string m_instrument_tip_frame;
    tf::StampedTransform m_instrument_tipMVflange; // assume this constant


};

#endif // _TROCAR2CARTESIAN_H_
