#ifndef _TROCAR2CARTESIAN_H_
#define _TROCAR2CARTESIAN_H_

// system includes
#include <thread>
#include <mutex>

// library includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

// custom includes
#include <trocar2cartesian_msgs/TrocarPose.h>
#include <trocar2cartesian_msgs/SetTrocar.h>
#include <GeneralPurposeInterpolator.hpp>

// forward declarations



class Trocar2Cartesian
{
  public:
    // enums

    // typedefs

    // const static member variables
    static const int trocarParams = 3;
    static constexpr double m_velMax = 0.05;
    static constexpr double m_accelMax = 1.0;
    static constexpr double m_trocarPeriod = 0.01;
    static constexpr double m_rAbsoluteMin = 0.01;
    static constexpr double m_rAbsoluteMax = 0.3;
    static constexpr double m_thetaAbsoluteMin = M_PI/2;
    static constexpr double m_thetaAbsoluteMax = M_PI;
    static constexpr double m_phiAbsoluteMin = -M_PI;
    static constexpr double m_phiAbsoluteMax = M_PI;
 
    // static utility functions


    // constructors
    Trocar2Cartesian(const std::string& robotName, const std::string& baseTfName, const std::string& flangeTfName);

    // overwritten methods

    // methods
    trocar2cartesian_msgs::TrocarPose pose2trocarpose(const tf::Pose& pose);
    tf::Pose trocarpose2pose(const trocar2cartesian_msgs::TrocarPose& trocarpose);

    // variables


  protected:
    // methods

    // variables


  private:
    // methods
    void getCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg);
    void publishTrocarTf();
    void publishTrocarTfLoop();
    bool setTrocarCallback(trocar2cartesian_msgs::SetTrocar::Request& request, trocar2cartesian_msgs::SetTrocar::Response& response);
    void setTrocarPoseCallback(const trocar2cartesian_msgs::TrocarPose::ConstPtr& trocarMsg);
    bool moveIntoTrocar(const tf::Pose& target, double velocity_translation, double velocity_rotation);
    void trocarMoveLoop();
    bool withinTrocarLimits(const trocar2cartesian_msgs::TrocarPose& trocarMsg);

    // variables
    ros::NodeHandle m_node;
    ros::Publisher m_setCartesianTopicPub;
    ros::Subscriber m_getCartesianTopicSub;
    ros::ServiceServer m_setTrocarService;
    ros::Publisher m_getTrocarTopicPub;
    ros::Subscriber m_setTrocarTopicSub;
    tf::TransformListener m_tfListener;

    std::string m_robotName;
    std::string m_baseTfName;
    std::string m_flangeTfName;
    tf::Pose m_trocarPose;
    geometry_msgs::Pose m_lastCartesianPose;
    std::mutex m_lastCartesianPoseMutex;
    trocar2cartesian_msgs::TrocarPose m_lastTrocarPose;
    std::mutex m_lastTrocarPoseMutex;
    geometry_msgs::Pose m_targetCartesianPose;
    std::string m_instrument_tip_frame;
    tf::StampedTransform m_instrument_tipMVflange; // assume this constant
    tf::Transform m_instrument_tipMVflangeInverse; // assume this constant
    bool m_publishTrocarTfThreadRunning = false;
    std::thread m_publishTrocarTfThread;
    bool m_inTrocar = false;

    double m_rMin = m_rAbsoluteMin;
    double m_rMax = m_rAbsoluteMax;
    double m_thetaMin = m_thetaAbsoluteMin;
    double m_thetaMax = m_thetaAbsoluteMax;
    double m_phiMin = m_phiAbsoluteMin;
    double m_phiMax = m_phiAbsoluteMax;

    bool m_trocarMoveActive = false;
    std::mutex m_trocarMoveActiveMutex;
    GeneralPurposeInterpolator m_trocarGpi;
    std::vector<double> m_trocarGpiPosCurrentBuffer;
    std::vector<double> m_trocarGpiPosTargetBuffer;
    std::vector<double> m_trocarGpiPosMinBuffer;
    std::vector<double> m_trocarGpiPosMaxBuffer;
    std::vector<double> m_trocarGpiVelCurrentBuffer;
    std::vector<double> m_trocarGpiVelMaxBuffer;
    std::vector<double> m_trocarGpiAccelMaxBuffer;
    std::thread m_trocarMoveThread;
    bool m_trocarMoveThreadRunning = false;


    tf::TransformBroadcaster m_footfBroadcaster; // TODO rm debug only

};

#endif // _TROCAR2CARTESIAN_H_
