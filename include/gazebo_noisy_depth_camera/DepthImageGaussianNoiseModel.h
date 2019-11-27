#ifndef GAZEBO_NOISY_DEPTH_CAMERA_DEPTHIMAGEGAUSSIANNOISEMODEL_H
#define GAZEBO_NOISY_DEPTH_CAMERA_DEPTHIMAGEGAUSSIANNOISEMODEL_H

#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo/sensors/Noise.hh>

namespace gazebo
{
namespace sensors
{

class DepthImageGaussianNoiseModel : public ImageGaussianNoiseModel
{
  public: void SetCamera(rendering::CameraPtr _camera) override;
};

}
}

#endif //GAZEBO_NOISY_DEPTH_CAMERA_DEPTHIMAGEGAUSSIANNOISEMODEL_H
