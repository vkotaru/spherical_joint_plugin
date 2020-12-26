#ifndef SPHERICAL_JOINT_PLUGIN_HH_
#define SPHERICAL_JOINT_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/simbody/SimbodyJoint.hh>
#include <stdio.h>
#include <gazebo/common/CommonTypes.hh>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using IgnitionVector = ignition::math::Vector3d;
using IgnitionPose = ignition::math::Pose3d;
using IgnitionQuaternion = ignition::math::Quaterniond;

namespace gazebo {

inline Eigen::Vector3d vectorIgnition2Eigen(IgnitionVector vec) {
  Eigen::Vector3d out;
  out << vec.X(), vec.Y(), vec.Z();
  return out;
}

inline IgnitionVector vectorEigen2Ignition(Eigen::Vector3d vec) {
  IgnitionVector out(vec(0), vec(1), vec(2));
  return out;
}

inline Eigen::Matrix3d rotationIgnition2Eigen(IgnitionQuaternion quat) {
  Eigen::Quaterniond eig_quat(quat.W(), quat.X(), quat.Y(), quat.Z());
  return eig_quat.toRotationMatrix();
}

inline IgnitionQuaternion rotationEigen2Ignition(Eigen::Matrix3d  rot) {
  Eigen::Quaterniond q(rot);
  IgnitionQuaternion ign_q(q.w(), q.x(), q.y(), q.z());
  return ign_q;
}

inline Eigen::Matrix3d hat(const Eigen::Vector3d _vec) {
  return (Eigen::Matrix3d() << 0.0, -_vec(2), _vec(1), _vec(2),
      0.0, -_vec(0), -_vec(1), _vec(0), 0.0)
      .finished();
}

class SphericalJointPlugin : public ModelPlugin {

 public:
  SphericalJointPlugin();
  ~SphericalJointPlugin() override;

 protected:
  void CustomInit();
  void Reset() override;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const common::UpdateInfo &_info);

 private:
  common::Time lastUpdateTime_, currentUpdateTime_;

  std::string parent_link_name_;
  std::string child_link_name_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  /// \brief Pointer to parent link
  physics::LinkPtr parent_link_;
  /// \brief Pointer to child link
  physics::LinkPtr child_link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  /// \brief Reset the link pose
  IgnitionPose parent_reset_pose_, child_reset_pose_;
};

}

#endif // SPHERICAL_JOINT_PLUGIN_HH_