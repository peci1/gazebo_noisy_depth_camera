#ifndef GAZEBO_NOISY_DEPTH_CAMERA_MULTIPLICATIVEGAUSSIANNOISEMODEL_H
#define GAZEBO_NOISY_DEPTH_CAMERA_MULTIPLICATIVEGAUSSIANNOISEMODEL_H

#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo_noisy_depth_camera/PostRenderImageNoise.h>
#include <gazebo/sensors/Noise.hh>

namespace gazebo
{
namespace sensors
{

class MultiplicativeGaussianNoiseModel : public ImageGaussianNoiseModel, public PostRenderImageNoise
{
  public: void SetCamera(rendering::CameraPtr _camera) override;
  public: void ApplyFloat(float* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat) override;
};

}
}

#endif //GAZEBO_NOISY_DEPTH_CAMERA_MULTIPLICATIVEGAUSSIANNOISEMODEL_H
