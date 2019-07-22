#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <cstdint>

#include "gazebo_magnet/dipole_magnet.h"

namespace gazebo {

DipoleMagnet::DipoleMagnet(): ModelPlugin() {
  this->connect_count = 0;
}

DipoleMagnet::~DipoleMagnet() {
  this->update_connection.reset();
  if (this->should_publish) {
    this->queue.clear();
    this->queue.disable();
    this->rosnode->shutdown();
    this->callback_queue_thread.join();
    delete this->rosnode;
  }
  if (this->mag){
    DipoleMagnetContainer::Get().Remove(this->mag);
  }
}

void DipoleMagnet::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  this->model = _parent;
  this->world = _parent->GetWorld();
  gzdbg << "Loading DipoleMagnet plugin" << std::endl;

  this->mag = std::make_shared<DipoleMagnetContainer::Magnet>();

  // load parameters
  this->robot_namespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if(!_sdf->HasElement("bodyName")) {
    gzerr << "DipoleMagnet plugin missing <bodyName>, cannot proceed" << std::endl;
    return;
  }else {
    this->link_name = _sdf->GetElement("bodyName")->Get<std::string>();
  }

  this->link = this->model->GetLink(this->link_name);
  if(!this->link){
    gzerr << "Error: link named " << this->link_name << " does not exist" << std::endl;
    return;
  }

  this->should_publish = false;
  if (_sdf->HasElement("shouldPublish"))
  {
    this->should_publish = _sdf->GetElement("shouldPublish")->Get<bool>();
  }

  if (!_sdf->HasElement("updateRate"))
  {
    gzmsg << "DipoleMagnet plugin missing <updateRate>, defaults to 0.0"
        " (as fast as possible)" << std::endl;
    this->update_rate = 0;
  }
  else
    this->update_rate = _sdf->GetElement("updateRate")->Get<double>();

  if (_sdf->HasElement("calculate")){
    this->mag->calculate = _sdf->Get<bool>("calculate");
  } else
    this->mag->calculate = true;

  if (_sdf->HasElement("dipole_moment")){
    this->mag->moment = _sdf->Get<ignition::math::Vector3d>("dipole_moment");
  }

  if (_sdf->HasElement("xyzOffset")){
    this->mag->offset.Pos() = _sdf->Get<ignition::math::Vector3d>("xyzOffset");
  }

  if (_sdf->HasElement("rpyOffset")){
    ignition::math::Vector3d rpy_offset = _sdf->Get<ignition::math::Vector3d>("rpyOffset");
    this->mag->offset.Rot() = ignition::math::Quaterniond(rpy_offset);
  }

  if (this->should_publish) {
    if (!_sdf->HasElement("topicNs"))
    {
      gzmsg << "DipoleMagnet plugin missing <topicNs>," 
          "will publish on namespace " << this->link_name << std::endl;
    }

    else {
      this->topic_ns = _sdf->GetElement("topicNs")->Get<std::string>();
    }

    if (!ros::isInitialized())
    {
      gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
        "the gazebo_ros package. If you want to use this plugin without ROS, "
        "set <shouldPublish> to false" << std::endl;
      return;
    }

    this->rosnode = new ros::NodeHandle(this->robot_namespace);
    this->rosnode->setCallbackQueue(&this->queue);

    this->wrench_pub = this->rosnode->advertise<geometry_msgs::WrenchStamped>(
        this->topic_ns + "/wrench", 1,
        boost::bind( &DipoleMagnet::Connect,this),
        boost::bind( &DipoleMagnet::Disconnect,this), ros::VoidPtr(), &this->queue);
    this->mfs_pub = this->rosnode->advertise<sensor_msgs::MagneticField>(
        this->topic_ns + "/mfs", 1,
        boost::bind( &DipoleMagnet::Connect,this),
        boost::bind( &DipoleMagnet::Disconnect,this), ros::VoidPtr(), &this->queue);

    this->callback_queue_thread = boost::thread(boost::bind(&DipoleMagnet::QueueThread,this));
  }

  this->mag->model_id = this->model->GetId();

  gzmsg << "Loaded Gazebo dipole magnet plugin on " << this->model->GetName() << std::endl;

  DipoleMagnetContainer::Get().Add(this->mag);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DipoleMagnet::OnUpdate, this, _1));
}

void DipoleMagnet::Connect() {
  this->connect_count++;
}

void DipoleMagnet::Disconnect() {
  this->connect_count--;
}

void DipoleMagnet::QueueThread() {
  static const double timeout = 0.01;

  while (this->rosnode->ok())
  {
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}

// Called by the world update start event
void DipoleMagnet::OnUpdate(const common::UpdateInfo &) {
  // Calculate the force from all other magnets
  ignition::math::Pose3d p_self = this->link->WorldPose();
  p_self.Pos() += -p_self.Rot().RotateVector(this->mag->offset.Pos());
  p_self.Rot() *= this->mag->offset.Rot().Inverse();

  this->mag->pose = p_self;

  if (!this->mag->calculate)
    return;

  DipoleMagnetContainer& dp = DipoleMagnetContainer::Get();

  ignition::math::Vector3d moment_world = p_self.Rot().RotateVector(this->mag->moment);

  ignition::math::Vector3d force(0, 0, 0);
  ignition::math::Vector3d torque(0, 0, 0);
  ignition::math::Vector3d mfs(0, 0, 0);
  for(DipoleMagnetContainer::MagnetPtrV::iterator it = dp.magnets.begin(); it < dp.magnets.end(); it++){
    std::shared_ptr<DipoleMagnetContainer::Magnet> mag_other = *it;
    if (mag_other->model_id != this->mag->model_id) {
      ignition::math::Pose3d p_other = mag_other->pose;
      ignition::math::Vector3d m_other = p_other.Rot().RotateVector(mag_other->moment);

      ignition::math::Vector3d force_tmp;
      ignition::math::Vector3d torque_tmp;
      GetForceTorque(p_self, moment_world, p_other, m_other, force_tmp, torque_tmp);

      force += force_tmp;
      torque += torque_tmp;

      ignition::math::Vector3d mfs_tmp;
      GetMFS(p_self, p_other, m_other, mfs_tmp);

      mfs += mfs_tmp;

      this->link->AddForce(force_tmp);
      this->link->AddTorque(torque_tmp);
    }
  }

  this->PublishData(force, torque, mfs);
}

void DipoleMagnet::PublishData(
    const ignition::math::Vector3d& force,
    const ignition::math::Vector3d& torque,
    const ignition::math::Vector3d& mfs){
  if(this->should_publish && this->connect_count > 0) {
    // Rate control
    common::Time cur_time = this->world->SimTime();
    if (this->update_rate > 0 &&
        (cur_time-this->last_time).Double() < (1.0/this->update_rate))
      return;

    this->lock.lock();
    this->wrench_msg.header.frame_id = "world";
    this->wrench_msg.header.stamp.sec = cur_time.sec;
    this->wrench_msg.header.stamp.nsec = cur_time.nsec;

    this->wrench_msg.wrench.force.x = force.X();
    this->wrench_msg.wrench.force.y = force.Y();
    this->wrench_msg.wrench.force.z = force.Z();
    this->wrench_msg.wrench.torque.x = torque.X();
    this->wrench_msg.wrench.torque.y = torque.Y();
    this->wrench_msg.wrench.torque.z = torque.Z();

    // now mfs
    this->mfs_msg.header.frame_id = this->link_name;
    this->mfs_msg.header.stamp.sec = cur_time.sec;
    this->mfs_msg.header.stamp.nsec = cur_time.nsec;

    this->mfs_msg.magnetic_field.x = mfs.X();
    this->mfs_msg.magnetic_field.y = mfs.Y();
    this->mfs_msg.magnetic_field.z = mfs.Z();


    this->wrench_pub.publish(this->wrench_msg);
    this->mfs_pub.publish(this->mfs_msg);

    this->lock.unlock();
  }
}

void DipoleMagnet::GetForceTorque(const ignition::math::Pose3d& p_self,
    const ignition::math::Vector3d& m_self,
    const ignition::math::Pose3d& p_other,
    const ignition::math::Vector3d& m_other,
    ignition::math::Vector3d& force,
    ignition::math::Vector3d& torque) {

  bool debug = false;
  ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
  ignition::math::Vector3d p_unit = p/p.Length();

  ignition::math::Vector3d m1 = m_other;
  ignition::math::Vector3d m2 = m_self;
  if (debug)
    std::cout << "p: " << p << " m1: " << m1 << " m2: " << m2 << std::endl;

  double K = 3.0*1e-7/pow(p.Length(), 4);
  force = K * (m2 * (m1.Dot(p_unit)) +  m1 * (m2.Dot(p_unit)) +
      p_unit*(m1.Dot(m2)) - 5*p_unit*(m1.Dot(p_unit))*(m2.Dot(p_unit)));

  double Ktorque = 1e-7/pow(p.Length(), 3);
  ignition::math::Vector3d B1 = Ktorque*(3*(m1.Dot(p_unit))*p_unit - m1);
  torque = m2.Cross(B1);
  if (debug)
    std::cout << "B: " << B1 << " K: " << Ktorque << " t: " << torque << std::endl;
}

void DipoleMagnet::GetMFS(const ignition::math::Pose3d& p_self,
    const ignition::math::Pose3d& p_other,
    const ignition::math::Vector3d& m_other,
    ignition::math::Vector3d& mfs) {

  // sensor location
  ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
  ignition::math::Vector3d p_unit = p/p.Length();

  // Get the field at the sensor location
  double K = 1e-7/pow(p.Length(), 3);
  ignition::math::Vector3d B = K*(3*(m_other.Dot(p_unit))*p_unit - m_other);

  // Rotate the B vector into the capsule/body frame
  ignition::math::Vector3d B_body = p_self.Rot().RotateVectorReverse(B);

  // Assign vector
  mfs.X(B_body[0]);
  mfs.Y(B_body[1]);
  mfs.Z(B_body[2]);
}

GZ_REGISTER_MODEL_PLUGIN(DipoleMagnet)
}
