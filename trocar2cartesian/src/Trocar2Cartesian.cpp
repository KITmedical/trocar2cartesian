#include "Trocar2Cartesian.hpp"

// system includes

// library includes

// custom includes
#include <ahbros.hpp>


/*---------------------------------- public: -----------------------------{{{-*/
Trocar2Cartesian::Trocar2Cartesian(const std::string& baseTfName, const std::string& flangeTfName)
  :m_baseTfName(baseTfName),
   m_flangeTfName(flangeTfName)
{
  m_getCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>("get_cartesian", 1, &Trocar2Cartesian::getCartesianCallback, this);
  m_setTrocarService = m_node.advertiseService("set_trocar", &Trocar2Cartesian::setTrocarCallback, this);
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
Trocar2Cartesian::getCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  //std::cout << "getCartesianCallback: poseMsg=\n" << *poseMsg << std::endl;
  m_lastCartesianPose = *poseMsg;
}

bool
Trocar2Cartesian::setTrocarCallback(trocar2cartesian_msgs::SetTrocar::Request& request, trocar2cartesian_msgs::SetTrocar::Response& response)
{
  response.success = false;

  tf::StampedTransform trocar_frameCBTbase;
  try {
    m_tfListener.lookupTransform(m_baseTfName, request.trocar_frame, ros::Time(0), trocar_frameCBTbase);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  tf::Pose trocar_pose_trocar_frame;
  tf::poseMsgToTF(request.trocar_pose, trocar_pose_trocar_frame);
  tf::Pose trocar_pose_base = trocar_frameCBTbase * trocar_pose_trocar_frame;
  std::cout << "trocar_pose_trocar_frame: " << ahb::string::toString(trocar_pose_trocar_frame) << std::endl;
  std::cout << "trocar_frameCBTbase: " << ahb::string::toString(trocar_frameCBTbase) << std::endl;
  std::cout << "trocar_pose_base: " << ahb::string::toString(trocar_pose_base) << std::endl;

  // TODO cont (always work in _base)

  m_instrument_tip_frame = request.instrument_tip_frame;
  try {
    m_tfListener.lookupTransform(m_instrument_tip_frame, m_flangeTfName, ros::Time(0), m_instrument_tipMVflange);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  // if (unprojection of) projection of current pose to trocar params is 'close' to current pose
  //   slowly move into trocar-projected pose
  //   start set_trocar sub, get_trocar pub, set_cartesian pub
  // else success = false

  response.success = true;
  return true;
}

void
Trocar2Cartesian::setTrocarPoseCallback(const trocar2cartesian_msgs::TrocarPose::ConstPtr& trocarMsg)
{
  if (trocarMsg->instrument_tip_frame != m_instrument_tip_frame) {
    return;
  }
}
/*------------------------------------------------------------------------}}}-*/
