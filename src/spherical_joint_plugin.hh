#ifndef SPHERICAL_JOINT_PLUGIN_HH_
#define SPHERICAL_JOINT_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/common/CommonTypes.hh>
#include <eigen3/Eigen/Dense>

using IgnitionVector = ignition::math::Vector3d;
using IgnitionPose = ignition::math::Pose3d;
using IgnitionQuaternion = ignition::math::Quaterniond;

namespace gazebo {

inline Eigen::Vector3d vectorIgnition2Vector(IgnitionVector vec) {
  Eigen::Vector3d out;
  out << vec.X(), vec.Y(), vec.Z();
  return out;
}

inline IgnitionVector vectorEigen2Ignition(Eigen::Vector3d vec) {
  IgnitionVector out(vec(0), vec(1), vec(2));
  return out;
}

inline Eigen::Matrix3d rotationIgnition2Vector(IgnitionQuaternion quat) {
  Eigen::Quaterniond eig_quat(quat.W(), quat.X(), quat.Y(), quat.Z());
  return eig_quat.toRotationMatrix();
}

class SphericalJointPlugin : public ModelPlugin {

 public:
  SphericalJointPlugin();
  ~SphericalJointPlugin() override;

 protected:
  void Reset() override;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const common::UpdateInfo &_info);

 private:
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

  /// \brief Spherical joint state on $S^2$ manifold
  struct S2State {
    S2State() : q((Eigen::Vector3d()<< 0, 0, 1).finished()), w(Eigen::Vector3d::Zero()) {}
    /// \brief unit-vector q on (S2)
    Eigen::Vector3d q;
    /// \brief angular velocity w on (TS2)
    Eigen::Vector3d w;
  };
  S2State state_;

};

}

#endif // SPHERICAL_JOINT_PLUGIN_HH_