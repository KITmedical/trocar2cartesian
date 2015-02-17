#include "Trocar2Cartesian.hpp"

// system includes

// library includes

// custom includes
#include <ahbros.hpp>
#include <cartesian_interpolator/CartesianInterpolator.hpp>

Eigen::Vector3d
tf2eigenVector(const tf::Vector3& tfvec)
{
  return Eigen::Vector3d(tfvec[0], tfvec[1], tfvec[2]);
}

Eigen::Quaterniond
tf2eigenQuaternion(const tf::Quaternion& tfquat)
{
  return Eigen::Quaterniond(tfquat.getW(), tfquat.getX(), tfquat.getY(), tfquat.getZ());
}

Eigen::Vector4d
tf2eigenQuaternionAsVec(const tf::Quaternion& tfquat)
{
  return Eigen::Vector4d(tfquat.getW(), tfquat.getX(), tfquat.getY(), tfquat.getZ());
}

double
distTranslation(const tf::Pose& pose1, const tf::Pose& pose2)
{
  return (tf2eigenVector(pose1.getOrigin()) - tf2eigenVector(pose2.getOrigin())).norm();
}

double
distRotation(const tf::Pose& pose1, const tf::Pose& pose2)
{
  return tf2eigenQuaternion(pose1.getRotation()).angularDistance(tf2eigenQuaternion(pose2.getRotation()));
}

bool
isClose(const tf::Pose& pose1, const tf::Pose& pose2, double delta)
{
  return ((distTranslation(pose1, pose2) + distRotation(pose1, pose2)) < delta);
}

/*---------------------------------- public: -----------------------------{{{-*/
Trocar2Cartesian::Trocar2Cartesian(const std::string& robotName, const std::string& baseTfName, const std::string& flangeTfName)
  :m_robotName(robotName),
   m_baseTfName(baseTfName),
   m_flangeTfName(flangeTfName),
   m_trocarGpi(trocarParams),
   m_trocarGpiPosCurrentBuffer(trocarParams, 0),
   m_trocarGpiPosTargetBuffer(trocarParams, 0),
   m_trocarGpiPosMinBuffer(trocarParams, 0),
   m_trocarGpiPosMaxBuffer(trocarParams, 0),
   m_trocarGpiVelCurrentBuffer(trocarParams, 0),
   m_trocarGpiVelMaxBuffer(trocarParams, m_velMax),
   m_trocarGpiAccelMaxBuffer(trocarParams, m_accelMax)
{
  m_trocarGpiPosMinBuffer[0] = 0.01;
  m_trocarGpiPosMinBuffer[1] = 0;
  m_trocarGpiPosMinBuffer[2] = -M_PI;
  m_trocarGpiPosMaxBuffer[0] = 0.4;
  m_trocarGpiPosMaxBuffer[1] = M_PI;
  m_trocarGpiPosMaxBuffer[2] = M_PI;
  m_trocarGpi.setXTarget(m_trocarGpiPosTargetBuffer);
  m_trocarGpi.setXLast(m_trocarGpiPosCurrentBuffer);
  m_trocarGpi.setVLast(m_trocarGpiVelCurrentBuffer);
  m_trocarGpi.setXMin(m_trocarGpiPosMinBuffer);
  m_trocarGpi.setXMax(m_trocarGpiPosMaxBuffer);
  m_trocarGpi.setVMax(m_trocarGpiVelMaxBuffer);
  m_trocarGpi.setAMax(m_trocarGpiAccelMaxBuffer);
  m_trocarGpi.setDt(m_trocarPeriod);
  m_trocarGpi.setMode(1);

  m_getCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>("get_cartesian", 1, &Trocar2Cartesian::getCartesianCallback, this);
  m_setCartesianTopicPub = m_node.advertise<geometry_msgs::Pose>("set_cartesian", 1);
  m_setTrocarService = m_node.advertiseService("set_trocar", &Trocar2Cartesian::setTrocarCallback, this);
}

trocar2cartesian_msgs::TrocarPose
Trocar2Cartesian::pose2trocarpose(const tf::Pose& pose)
{
  trocar2cartesian_msgs::TrocarPose trocarPose;

  Eigen::Vector3d pose_translation = tf2eigenVector(pose.getOrigin());
  Eigen::Vector3d trocar_translation = tf2eigenVector(m_trocarPose.getOrigin());

  Eigen::Vector3d trocar_to_pose = pose_translation - trocar_translation;
  double r = trocar_to_pose.norm();
  double x = trocar_to_pose[0];
  double y = trocar_to_pose[1];
  double z = trocar_to_pose[2];
  double theta = acos(z / r);
  double phi = atan2(y, x);
  //std::cout << "trocar_to_pose: " << trocar_to_pose << std::endl;
  //std::cout << "r=" << r << " theta=" << theta << " phi=" << phi << std::endl;

  trocarPose.instrument_tip_frame = m_instrument_tip_frame;
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
  Eigen::Vector3d vecZ(trocar_to_pose); // forward
  if (vecZ.norm() == 0) {
    ROS_ERROR("|vecZ| is 0");
    throw std::logic_error("|vecZ| is 0");
  }
  vecZ /= vecZ.norm();
  Eigen::Vector3d vecX = vecUp.cross(vecZ); // right
  vecX /= vecX.norm();
  Eigen::Vector3d vecY = vecZ.cross(vecX); // down
  vecY /= vecY.norm();
  //std::cout << "trocar_to_pose: " << trocar_to_pose << std::endl;
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
  {
    std::lock_guard<std::mutex> guard(m_lastCartesianPoseMutex);
    m_lastCartesianPose = *poseMsg;
  }

  if (m_inTrocar) {
    tf::Pose flange_base;
    tf::poseMsgToTF(m_lastCartesianPose, flange_base);
    tf::Pose instrument_tip_base = flange_base * m_instrument_tipMVflange;
    trocar2cartesian_msgs::TrocarPose trocarPose = pose2trocarpose(instrument_tip_base);
    {
      std::lock_guard<std::mutex> guard(m_lastTrocarPoseMutex);
      m_lastTrocarPose = trocarPose;
    }
    m_getTrocarTopicPub.publish(trocarPose);
    {
      std::lock_guard<std::mutex> guard(m_trocarMoveActiveMutex);
      if (!m_trocarMoveActive) {
        m_trocarGpiPosCurrentBuffer[0] = trocarPose.r;
        m_trocarGpiPosCurrentBuffer[1] = trocarPose.theta;
        m_trocarGpiPosCurrentBuffer[2] = trocarPose.phi;
        m_trocarGpi.setXLast(m_trocarGpiPosCurrentBuffer);
      }
    }
  }
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

  {
    std::lock_guard<std::mutex> guard(m_trocarMoveActiveMutex);
    if (m_trocarMoveActive) {
      ROS_ERROR("Can not change trocar while moving");
      return false;
    }
  }

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
    m_instrument_tipMVflangeInverse = m_instrument_tipMVflange.inverse();
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::Pose flange_base;
  tf::poseMsgToTF(m_lastCartesianPose, flange_base);
  tf::Pose instrument_tip_base = flange_base * m_instrument_tipMVflange;
  trocar2cartesian_msgs::TrocarPose initial_instrument_tip_trocarpose = pose2trocarpose(instrument_tip_base);
  tf::Pose reprojected_instrument_tip_base = trocarpose2pose(initial_instrument_tip_trocarpose);
  tf::Pose reprojected_flange_base = reprojected_instrument_tip_base * m_instrument_tipMVflangeInverse;
  double dist_translation = distTranslation(flange_base, reprojected_flange_base);
  double dist_rotation = distRotation(flange_base, reprojected_flange_base);

  std::cout << "flange_base: " << ahb::string::toString(flange_base) << std::endl;
  std::cout << "instrument_tip_base: " << ahb::string::toString(instrument_tip_base) << std::endl;
  std::cout << "initial_instrument_tip_trocarpose:\n" << initial_instrument_tip_trocarpose << std::endl;
  std::cout << "reprojected_instrument_tip_base: " << ahb::string::toString(reprojected_instrument_tip_base) << std::endl;
  m_footfBroadcaster.sendTransform(tf::StampedTransform(reprojected_instrument_tip_base, ros::Time::now(), m_baseTfName, m_robotName + "_trocar_tip"));
  std::cout << "reprojected_flange_base: " << ahb::string::toString(reprojected_flange_base) << std::endl;
  m_footfBroadcaster.sendTransform(tf::StampedTransform(reprojected_flange_base, ros::Time::now(), m_baseTfName, m_robotName + "_trocar_flange"));
  std::cout << "dist: translation=" << dist_translation << " rotation=" << dist_rotation << std::endl;

  if (dist_translation > 0.2 && dist_rotation > M_PI/6) {
    ROS_ERROR("Robot would move too far in order to be in valid trocar position");
    return false;
  }

  if (!moveIntoTrocar(reprojected_flange_base, 0.1, 0.1)) {
    ROS_ERROR("Robot could not move into trocar");
    return false;
  }
  
  m_setTrocarTopicSub = m_node.subscribe<trocar2cartesian_msgs::TrocarPose>("set_trocar", 1, &Trocar2Cartesian::setTrocarPoseCallback, this);
  m_getTrocarTopicPub = m_node.advertise<trocar2cartesian_msgs::TrocarPose>("get_trocar", 1);
  m_inTrocar = true;
  if (!m_trocarMoveThreadRunning) {
    m_trocarMoveThread = std::thread(&Trocar2Cartesian::trocarMoveLoop, this);
  }

  response.success = true;
  return true;
}

void
Trocar2Cartesian::setTrocarPoseCallback(const trocar2cartesian_msgs::TrocarPose::ConstPtr& trocarMsg)
{
  if (trocarMsg->instrument_tip_frame != m_instrument_tip_frame) {
    ROS_ERROR("instrument_tip_frame in message does not match instrument_tip_frame at setup");
    return;
  }

  m_trocarGpiPosTargetBuffer[0] = trocarMsg->r;
  m_trocarGpiPosTargetBuffer[1] = trocarMsg->theta;
  m_trocarGpiPosTargetBuffer[2] = trocarMsg->phi;
  m_trocarGpi.setXTarget(m_trocarGpiPosTargetBuffer);
  {
    std::lock_guard<std::mutex> guard(m_trocarMoveActiveMutex);
    m_trocarMoveActive = true;
  }
}

bool
Trocar2Cartesian::moveIntoTrocar(const tf::Pose& target, double velocity_translation, double velocity_rotation)
{
  double dt = 0.1;
  CartesianInterpolator poseInterpolator(1, dt); // TODO use velocity_translation, velocity_rotation
  poseInterpolator.posTarget = tf2eigenVector(target.getOrigin());
  poseInterpolator.oriTarget = tf2eigenQuaternionAsVec(target.getRotation());
  ros::Rate rate(1.0/dt);

  tf::Pose lastPose;
  {
    std::lock_guard<std::mutex> guard(m_lastCartesianPoseMutex);
    tf::poseMsgToTF(m_lastCartesianPose, lastPose);
  }
  tf::Pose prevPose = lastPose;
  unsigned notMoving = 0;
  do {
    {
      std::lock_guard<std::mutex> guard(m_lastCartesianPoseMutex);
      tf::poseMsgToTF(m_lastCartesianPose, lastPose);
    }
    if (isClose(lastPose, prevPose, 0.00001)) { // got stuck
      if (notMoving >= 10) {
        return false;
      } else {
        notMoving++;
      }
    } else {
      notMoving = 0;
    }
    Vector3d lastPos = tf2eigenVector(lastPose.getOrigin());
    Vector4d lastOri = tf2eigenQuaternionAsVec(lastPose.getRotation());
    poseInterpolator.posLast = lastPos;
    poseInterpolator.oriLast = lastOri;
    poseInterpolator.interpolate();

    geometry_msgs::Pose nowPoseMsg;
    nowPoseMsg.position.x = poseInterpolator.posNow[0];
    nowPoseMsg.position.y = poseInterpolator.posNow[1];
    nowPoseMsg.position.z = poseInterpolator.posNow[2];
    nowPoseMsg.orientation.w = poseInterpolator.oriNow[0];
    nowPoseMsg.orientation.x = poseInterpolator.oriNow[1];
    nowPoseMsg.orientation.y = poseInterpolator.oriNow[2];
    nowPoseMsg.orientation.z = poseInterpolator.oriNow[3];
    m_setCartesianTopicPub.publish(nowPoseMsg);
    std::cout << nowPoseMsg << std::endl;

    prevPose = lastPose;
    rate.sleep();
  } while (ros::ok() && !isClose(lastPose, target, 0.001));

  return true;
}

void
Trocar2Cartesian::trocarMoveLoop()
{
  m_trocarMoveThreadRunning = true;
  ros::Rate rate(1.0/m_trocarPeriod);
  std::vector<double> lastTrocarGpiPosCurrentBuffer;
  tf::Pose instrumentPose;
  tf::Pose flangePose;
  trocar2cartesian_msgs::TrocarPose trocarPose;
  geometry_msgs::Pose flangePoseMsg;
  unsigned notMoving = 0;
  ROS_INFO("trocarMove thread started");
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> guard(m_trocarMoveActiveMutex);
      if (m_trocarMoveActive) {
        m_trocarGpi.interpolate();
        m_trocarGpi.getXNow(m_trocarGpiPosCurrentBuffer);
        m_trocarGpi.getVNow(m_trocarGpiVelCurrentBuffer);

        trocarPose.r = m_trocarGpiPosCurrentBuffer[0];
        trocarPose.theta = m_trocarGpiPosCurrentBuffer[1];
        trocarPose.phi = m_trocarGpiPosCurrentBuffer[2];
        std::cout << trocarPose << std::endl;
        instrumentPose = trocarpose2pose(trocarPose);
        flangePose = instrumentPose * m_instrument_tipMVflangeInverse;
        tf::poseTFToMsg(flangePose, flangePoseMsg);
        m_setCartesianTopicPub.publish(flangePoseMsg);

        if (std::equal(lastTrocarGpiPosCurrentBuffer.begin(), lastTrocarGpiPosCurrentBuffer.end(), m_trocarGpiPosCurrentBuffer.begin())) {
          notMoving++;
          if (notMoving > 10) {
            notMoving = 0;
            m_trocarMoveActive = false;
          }
        }
        lastTrocarGpiPosCurrentBuffer = m_trocarGpiPosCurrentBuffer;
      }
    }

    rate.sleep();
  }
  ROS_ERROR("trocarMove exited");
}
/*------------------------------------------------------------------------}}}-*/
