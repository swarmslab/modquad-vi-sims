#ifndef INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_
#define INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/MagneticField.h>

#include <memory>

#include "dipole_magnet_container.h"

namespace gazebo {

class DipoleMagnet : public ModelPlugin {
 public:
  DipoleMagnet();

  ~DipoleMagnet();

  // Loads plugin
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  // Callback for when subscribers connect
  void Connect();

  // Callback for when subscribers disconnect
  void Disconnect();

  // Thread to interact with ROS
  void QueueThread();

  // Called by the world update start event
  void OnUpdate(const common::UpdateInfo &);


  // Publish data to ros topics
  void PublishData(
      const ignition::math::Vector3d& force, //force vec for wrench msg
      const ignition::math::Vector3d& torque, // torque vec for wrench msg
      const ignition::math::Vector3d& mfs); //magnetic field data vec

  void GetForceTorque(const ignition::math::Pose3d& p_self, //pose of 1st magnet
			const ignition::math::Vector3d& m_self, //dipole of first magnet
			const ignition::math::Pose3d& p_other, //pose of 2nd magnet
			const ignition::math::Vector3d& m_other, //dipole moment of 2nd moment on which the force is calculated
			ignition::math::Vector3d& force, //calculated force
			ignition::math::Vector3d& torque); //calculated torque

  // Calculate the magnetic field in 6-dof
  void GetMFS(const ignition::math::Pose3d& p_self, //pose of 1st magnet
      const ignition::math::Pose3d& p_other, //pose of 2nd magnet
      const ignition::math::Vector3d& m_other, //dipole moment of 2nd magnet
      ignition::math::Vector3d& mfs); //magnetic field sensed

 private:
  physics::ModelPtr model;
  physics::LinkPtr link;
  physics::WorldPtr world;

  std::shared_ptr<DipoleMagnetContainer::Magnet> mag;

  std::string link_name;
  std::string robot_namespace;
  std::string topic_ns;

  bool should_publish;
  ros::NodeHandle* rosnode;
  ros::Publisher wrench_pub;
  ros::Publisher mfs_pub;

  geometry_msgs::WrenchStamped wrench_msg;
  sensor_msgs::MagneticField mfs_msg;

  boost::mutex lock;
  int connect_count;

  ros::CallbackQueue queue;
  boost::thread callback_queue_thread;

  common::Time last_time;
  double update_rate;
  event::ConnectionPtr update_connection;
};
}
#endif
