#include "spherical_joint_plugin.hh"
namespace gazebo {

SphericalJointPlugin::SphericalJointPlugin() : ModelPlugin() {
  namespace_.clear();
}

SphericalJointPlugin::~SphericalJointPlugin() {
  updateConnection_.reset();
}

void SphericalJointPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  gzmsg << "[SphericalJointPlugin] loading...\n";

  model_ = _model;
  world_ = model_->GetWorld();

  // look for "namespace" in the plugin parameters
  if (_sdf->HasElement("namespace")) {
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
    gzmsg << "[SphericalJointPlugin] namespace: " << namespace_ << std::endl;
  } else
    gzdbg << "[SphericalJointPlugin] No \"namespace\" specified.\n";

  // look for "parent link" in the plugin parameters
  if (_sdf->HasElement("parent_link")) {
    parent_link_name_ = _sdf->GetElement("parent_link")->Get<std::string>();
    gzmsg << "[SphericalJointPlugin] parent_link name: " << parent_link_name_ << std::endl;
  } else
    gzerr << "[SphericalJointPlugin] Parent link not specified\n";
  // find the parent link pointer
  parent_link_ = model_->GetLink(parent_link_name_);
  if (parent_link_ == NULL) gzthrow("[SphericalJointPlugin] Cannot find  \"" << parent_link_name_
                                                                             << "\".");
  // look for "child link" in the plugin parameters
  if (_sdf->HasElement("child_link")) {
    child_link_name_ = _sdf->GetElement("child_link")->Get<std::string>();
    gzmsg << "[SphericalJointPlugin] child_link name: " << child_link_name_ << std::endl;
  } else
    gzerr << "[SphericalJointPlugin] Child link not specified\n";
  // find the "child link" pointer
  child_link_ = model_->GetLink(child_link_name_);

  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SphericalJointPlugin::OnUpdate, this, _1));

  // Initial pose to parent (primary) link
  parent_reset_pose_ = parent_link_->WorldPose();
  child_reset_pose_ = child_link_->WorldPose();
  SphericalJointPlugin::CustomInit();

  gzmsg << "[SphericalJointPlugin] plugin loaded!" << std::endl;
}

void SphericalJointPlugin::CustomInit() {

  gzmsg << "parent_initial_pose_ " << parent_reset_pose_ << std::endl;
  gzmsg << "child_reset_pose_ " << child_reset_pose_ << std::endl;
  pos_rel2parent_ = vectorIgnition2Eigen(child_link_->RelativePose().Pos());

  // access the mass of the child_link
  m = child_link_->GetInertial()->Mass();

  // computing the link-orientation
  IgnitionPose child_COG_pose_ = child_link_->WorldCoGPose();
  Eigen::Vector3d rho = vectorIgnition2Eigen(child_COG_pose_.Pos()) - vectorIgnition2Eigen(child_reset_pose_.Pos());
  l = rho.norm();

  // Initial state
  state_ = S2State(rho, Eigen::Vector3d::Zero());
  gzmsg << "S2State " << state_ << std::endl;

  // Initial angular acceleration
  angular_accel_ = compute_angular_accel();

  // Last time update
  lastUpdateTime_ = world_->SimTime();
}

void SphericalJointPlugin::OnUpdate(const common::UpdateInfo &_info) {

  currentUpdateTime_ = world_->SimTime();
  double h = (currentUpdateTime_ - lastUpdateTime_).Double();

  // Variational Integration on S2
  Eigen::Vector3d dq, qnew, wnew, ang_vel_update;
  Eigen::Matrix3d rotq = (hat(state_.w() * h)).exp();
  dq = state_.w().cross(state_.q());
  qnew = rotq * state_.q();
  wnew = state_.w() + h * angular_accel_; // TODO verify this update against T.Lee et. al paper

  // TODO add T.Lee paper reference

  //  ang_vel_update = h * state_.w() + 0.5 * h * h * l * l * angular_accel_;
  //  qnew = (ang_vel_update).cross(state_.q()) + sqrt(1 - pow(ang_vel_update.norm(), 2)) * state_.q();
  //  wnew = state_.w() + h * l * l * angular_accel_;
  state_.update(qnew, wnew);

  // Angular Acceleration
  angular_accel_ = compute_angular_accel();

  // Set the new Pose & Velocity
  IgnitionPose new_pose;
  IgnitionVector lin_vel, ang_vel;

  Eigen::Vector3d new_child_pos_W = vectorIgnition2Eigen(parent_link_->WorldPose().Pos())
      + rotationIgnition2Eigen(parent_link_->WorldPose().Rot()) * pos_rel2parent_;
  Eigen::Matrix3d new_child_rot_W = rotq * rotationIgnition2Eigen(child_link_->WorldPose().Rot());

  new_pose = IgnitionPose(vectorEigen2Ignition(new_child_pos_W), rotationEigen2Ignition(new_child_rot_W));
  lin_vel = vectorEigen2Ignition(l * dq);
  ang_vel = vectorEigen2Ignition(wnew);
  child_link_->SetWorldPose(new_pose);
  child_link_->SetWorldTwist(lin_vel, ang_vel);

  // Update the time variable
  lastUpdateTime_ = currentUpdateTime_;
}

void SphericalJointPlugin::Reset() {
  // Reset the parent & child link
  parent_link_->SetWorldPose(parent_reset_pose_);
  child_link_->SetWorldPose(child_reset_pose_);
  parent_link_->ResetPhysicsStates();
  child_link_->ResetPhysicsStates();

  SphericalJointPlugin::CustomInit();

  gzdbg << "[SphericalJointPlugin] reset!" << std::endl;
}

Eigen::Vector3d SphericalJointPlugin::compute_angular_accel() {
  return -1 * (g / l) * state_.q().cross(e3);
};

GZ_REGISTER_MODEL_PLUGIN(SphericalJointPlugin);
}