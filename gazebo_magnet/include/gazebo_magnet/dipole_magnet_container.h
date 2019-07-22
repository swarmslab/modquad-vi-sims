#ifndef INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_CONTAINER_H_
#define INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_CONTAINER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <cstdint>

#include <gazebo/common/common.hh>

namespace gazebo {

class DipoleMagnetContainer {
 public:
  DipoleMagnetContainer() {
  }

  static DipoleMagnetContainer& Get() {
    static DipoleMagnetContainer instance;
    return instance;
  }

  struct Magnet {
    bool calculate;
    ignition::math::Vector3d moment;
    ignition::math::Pose3d offset;
    ignition::math::Pose3d pose;
    std::uint32_t model_id;
  };

  typedef std::shared_ptr<Magnet> MagnetPtr ;
  typedef std::vector<MagnetPtr> MagnetPtrV ;

  void Add(MagnetPtr mag) {
    std::cout << "Adding mag id:" << mag->model_id << std::endl;
    this->magnets.push_back(mag);
    std::cout << "Total: " << this->magnets.size() << " magnets" << std::endl;
  }
  void Remove(MagnetPtr mag) {
    std::cout << "Removing mag id:" << mag->model_id << std::endl;
    this->magnets.erase(std::remove(this->magnets.begin(), this->magnets.end(), mag), this->magnets.end());
    std::cout << "Total: " << this->magnets.size() << " magnets" << std::endl;
  }

  MagnetPtrV magnets;
};
}
#endif
