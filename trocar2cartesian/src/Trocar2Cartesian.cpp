#include "Trocar2Cartesian.hpp"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
Trocar2Cartesian::Trocar2Cartesian()
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
  // TODO
  // transform between frame_id (e.g. endoscope_tip) and eef (cartesian topic position)
  // check if request.pose is 'close' to current pose
  // slowly move into closest position of trocar projection
  // start set_trocar sub, get_trocar pub, set_cartesian pub

  response.success = true;
  return true;
}
/*------------------------------------------------------------------------}}}-*/
