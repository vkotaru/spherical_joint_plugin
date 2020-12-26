#ifndef SPHERICAL_JOINT_PLUGIN_HH_
#define SPHERICAL_JOINT_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo {

class SphericalJointPlugin : public ModelPlugin {

 public:
  SphericalJointPlugin();
  ~SphericalJointPlugin() override;

 private:
  void Reset() override;
  void Load() override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);

};

}

#endif // SPHERICAL_JOINT_PLUGIN_HH_