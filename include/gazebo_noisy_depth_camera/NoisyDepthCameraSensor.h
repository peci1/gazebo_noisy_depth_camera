#ifndef GAZEBO_NOISY_DEPTH_CAMERA_NOISYDEPTHCAMERASENSOR_H
#define GAZEBO_NOISY_DEPTH_CAMERA_NOISYDEPTHCAMERASENSOR_H

#include <gazebo/sensors/sensors.hh>

#include <memory>

namespace gazebo
{
namespace sensors
{

struct NoisyDepthCameraSensorPrivate;

class GZ_SENSORS_VISIBLE NoisyDepthCameraSensor : public DepthCameraSensor
{
  public: NoisyDepthCameraSensor();
  public: ~NoisyDepthCameraSensor() override;

  public: void Load(const std::string &_worldName) override;
  public: void Init() override;
  public: virtual void Reset();
  protected: void Fini() override;
  protected: bool UpdateImpl(bool force) override;
  public: std::string Topic() const override;
  public: bool IsActive() const override;

  protected: virtual NoisePtr CreateNoiseModel(sdf::ElementPtr _sdf,
      const std::string &_sensorType);

  private: std::unique_ptr<NoisyDepthCameraSensorPrivate> dataPtr;

  friend struct NoisyDepthCameraSensorPrivate;
};

}
}

#endif //GAZEBO_NOISY_DEPTH_CAMERA_NOISYDEPTHCAMERASENSOR_H
