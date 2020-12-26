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
  parent_reset_pose_ = parent_link_->WorldCoGPose();
  child_reset_pose_ = child_link_->WorldCoGPose();

  // Initial state
  //  state_.q = vectorIgnition2Vector(parent_reset_pose_.Pos());
  //  state_.w = vectorIgnition2Vector()

  gzmsg << "[SphericalJointPlugin] plugin loaded!" << std::endl;
}

void SphericalJointPlugin::OnUpdate(const common::UpdateInfo &_info) {
  IgnitionPose newPose;
//  gzmsg << "[Debug] " << newPose << std::endl;
//  child_link_->SetWorldPose(newPose);
}

void SphericalJointPlugin::Reset() {
  // Reset the parent link
  parent_link_->SetWorldPose(parent_reset_pose_);
  parent_link_->ResetPhysicsStates();
  // Reset the parent link
  child_link_->SetWorldPose(child_reset_pose_);
  child_link_->ResetPhysicsStates();

  gzdbg << "[SphericalJointPlugin] reset!" << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(SphericalJointPlugin);
}