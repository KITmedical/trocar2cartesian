#include "Trocar2Cartesian.hpp"

// system includes

// library includes

// custom includes
#include <ahbros.hpp>


/*---------------------------------- public: -----------------------------{{{-*/
Trocar2Cartesian::Trocar2Cartesian(const std::string& robotName, const std::string& baseTfName, const std::string& flangeTfName)
  :m_robotName(robotName),
   m_baseTfName(baseTfName),
   m_flangeTfName(flangeTfName)
{
  m_getCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>("get_cartesian", 1, &Trocar2Cartesian::getCartesianCallback, this);
  m_setTrocarService = m_node.advertiseService("set_trocar", &Trocar2Cartesian::setTrocarCallback, this);
}

trocar2cartesian_msgs::TrocarPose
Trocar2Cartesian::pose2trocarpose(const tf::Pose& pose)
{
  trocar2cartesian_msgs::TrocarPose trocarPose;

  Eigen::Vector3d pose_translation(pose.getOrigin()[0], pose.getOrigin()[1], pose.getOrigin()[2]);
  Eigen::Vector3d trocar_translation(m_trocarPose.getOrigin()[0], m_trocarPose.getOrigin()[1], m_trocarPose.getOrigin()[2]);

  Eigen::Vector3d trocar_to_pose = pose_translation - trocar_translation;
  double r = trocar_to_pose.norm();
  double x = trocar_to_pose[0];
  double y = trocar_to_pose[1];
  double z = trocar_to_pose[2];
  double theta = acos(z / r);
  double phi = atan(y / x);

  trocarPose.r = r;
  trocarPose.theta = theta;
  trocarPose.phi = phi;

  return trocarPose;
}

tf::Pose
Trocar2Cartesian::trocarpose2pose(const trocar2cartesian_msgs::TrocarPose& trocarpose)
{
  tf::Pose pose;

  double r = trocarpose.r;
  double theta = trocarpose.theta;
  double phi = trocarpose.phi;

  const double epsilon = 1e-9;
  if (r > -epsilon and r < epsilon) {
    ROS_ERROR("r is (too close to) 0");
    throw std::logic_error("r is (too close to) 0");
  }

  Eigen::Vector3d trocar_to_pose(r * sin(theta) * cos(phi),
                                 r * sin(theta) * sin(phi),
                                 r * cos(theta));

  Eigen::Vector3d vecUp(0, 0, -1);
  Eigen::Vector3d vecZ(trocar_to_pose);
  if (vecZ.norm() == 0) {
    ROS_ERROR("|vecZ| is 0");
    throw std::logic_error("|vecZ| is 0");
  }
  vecZ /= vecZ.norm();
  Eigen::Vector3d vecX = vecUp.cross(vecZ); // right
  vecX /= vecX.norm();
  Eigen::Vector3d vecY = vecZ.cross(vecX); // down
  vecY /= vecY.norm();
  //std::cout << "vecUp: " << vecUp << std::endl;
  //std::cout << "vecZ: " << vecZ << std::endl;
  //std::cout << "vecX: " << vecX << std::endl;
  //std::cout << "vecY: " << vecY << std::endl;

  pose.setOrigin(tf::Vector3(trocar_to_pose[0] + m_trocarPose.getOrigin()[0],
                             trocar_to_pose[1] + m_trocarPose.getOrigin()[1],
                             trocar_to_pose[2] + m_trocarPose.getOrigin()[2]));

  tf::Matrix3x3 rotation;
  for (int i = 0; i < 3; i++) {
    rotation[i][0] = vecX[i];
    rotation[i][1] = vecY[i];
    rotation[i][2] = vecZ[i];
  }
  pose.setBasis(rotation);

  return pose;
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

void
Trocar2Cartesian::publishTrocarTf()
{
  if (!m_publishTrocarTfThreadRunning) {
    m_publishTrocarTfThreadRunning = true;
    m_publishTrocarTfThread = std::thread(&Trocar2Cartesian::publishTrocarTfLoop, this);
  }
}

void
Trocar2Cartesian::publishTrocarTfLoop()
{
  ros::Rate rate(10);
  tf::TransformBroadcaster tfBroadcaster;
  while (ros::ok()) {
    tfBroadcaster.sendTransform(tf::StampedTransform(m_trocarPose, ros::Time::now(), m_baseTfName, m_robotName + "_trocar"));
    rate.sleep();
  }
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
  m_trocarPose = trocar_pose_base;
  publishTrocarTf();

  m_instrument_tip_frame = request.instrument_tip_frame;
  try {
    m_tfListener.lookupTransform(m_flangeTfName, m_instrument_tip_frame, ros::Time(0), m_instrument_tipMVflange);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::Pose flange_base;
  tf::poseMsgToTF(m_lastCartesianPose, flange_base);
  tf::Pose instrument_tip_base = flange_base * m_instrument_tipMVflange;
  trocar2cartesian_msgs::TrocarPose initial_instrument_tip_trocarpose = pose2trocarpose(instrument_tip_base);
  tf::Pose reprojected_instrument_tip_base = trocarpose2pose(initial_instrument_tip_trocarpose);
  std::cout << "flange_base: " << ahb::string::toString(flange_base) << std::endl;
  std::cout << "instrument_tip_base: " << ahb::string::toString(instrument_tip_base) << std::endl;
  std::cout << "initial_instrument_tip_trocarpose:\n" << initial_instrument_tip_trocarpose << std::endl;
  std::cout << "reprojected_instrument_tip_base: " << ahb::string::toString(reprojected_instrument_tip_base) << std::endl;
  m_footfBroadcaster.sendTransform(tf::StampedTransform(reprojected_instrument_tip_base, ros::Time::now(), m_baseTfName, m_robotName + "_trocar_foo"));

  // TODO cont (always work in _base)


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
