#ifndef SPHERICAL_JOINT_PLUGIN_HH_
#define SPHERICAL_JOINT_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
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

/// \brief Spherical joint state on $S^2$ manifold
class S2State {
 public:
  /// \brief Default constructor with q=e3 and w=zero(3,1)
  S2State() : q_((Eigen::Vector3d() << 0, 0, 1).finished()), w_(Eigen::Vector3d::Zero()) {}
  /// \brief Constructor with a vector and angular velocity
  /// vector will normalize in constructor
  /// \param: _q Eigen::Vector3d q in R^3
  /// \param: _w Eigen::Vector3d w in R^3
  S2State(Eigen::Vector3d _q, Eigen::Vector3d _w) : q_(_q), w_(_w) {
    q_.normalize();
  }
 public:
  friend std::ostream &operator<<(
      std::ostream &_out, const S2State &_state) {
    _out << "q: " << ignition::math::precision(_state.q()[0], 6)
         << " " << ignition::math::precision(_state.q()[1], 6)
         << " " << ignition::math::precision(_state.q()[2], 6)
         << " w: " << ignition::math::precision(_state.w()[0], 6)
         << " " << ignition::math::precision(_state.w()[1], 6)
         << " " << ignition::math::precision(_state.w()[2], 6);
    return _out;
  }
  /// \brief Public access to the state q_
  Eigen::Vector3d q() const {
    return q_;
  }
  /// \brief Public access to the state w_
  Eigen::Vector3d w() const {
    return w_;
  }
  void update(Eigen::Vector3d _q, Eigen::Vector3d _w) {
    _q.normalize();
    q_ = _q;
    w_ = _w;
  }
 private:
  /// \brief unit-vector q on (S2)
  Eigen::Vector3d q_;
  /// \brief angular velocity w on (TS2)
  Eigen::Vector3d w_;
};

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

  S2State state_;
  double l, m, g = 9.81;
  Eigen::Vector3d e3 = (Eigen::Vector3d() << 0., 0., 1.).finished();
  Eigen::Vector3d angular_accel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos_rel2parent_;

  Eigen::Vector3d compute_angular_accel();

};

}

#endif // SPHERICAL_JOINT_PLUGIN_HH_