#include "Trocar2Cartesian.hpp"

// system includes

// library includes

// custom includes


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

  // TODO
  // transform between frame_id (e.g. endoscope_tip) and flange (cartesian topic position)
  tf::StampedTransform trocar_frameMVbase;
  try {
    m_tfListener.lookupTransform(request.trocar_frame, m_baseTfName, ros::Time(0), trocar_frameMVbase);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

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
