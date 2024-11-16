// HACK HACK HACK
#include <any>
#include <sstream>
#define private public
#include <gazebo/rendering/DepthCamera.hh>
#undef private
// HACK HACK HACK

#include <gazebo_noisy_depth_camera/NoisyDepthCameraSensor.h>
#include <gazebo_noisy_depth_camera/DepthImageGaussianNoiseModel.h>
#include <gazebo_noisy_depth_camera/MultiplicativeGaussianNoiseModel.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Scene.hh>

#include <functional>

#include "DepthCameraPrivate.hh"

using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;
using ignition::math::Angle;
using gazebo::common::Time;

extern "C"
{
  GZ_REGISTER_STATIC_SENSOR("depth", NoisyDepthCameraSensor)
}

namespace gazebo
{
namespace sensors
{

struct NoisyDepthCameraSensorPrivate
{
  /// \brief Local pointer to the depthCamera.
  rendering::DepthCameraPtr depthCamera;

  event::ConnectionPtr worldResetConnection;
  event::ConnectionPtr timeResetConnection;
  event::ConnectionPtr depthFrameConnection;

  explicit NoisyDepthCameraSensorPrivate() = default;
};

NoisyDepthCameraSensor::NoisyDepthCameraSensor() :
  dataPtr(new NoisyDepthCameraSensorPrivate())
{
}

void NoisyDepthCameraSensor::Load(const std::string &_worldName)
{

  DepthCameraSensor::Load(_worldName);

  gzmsg << "Noisy depth camera loaded" << std::endl;
}

void NoisyDepthCameraSensor::Init()
{
  sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");

  DepthCameraSensor::Init();

  this->dataPtr->depthCamera = boost::dynamic_pointer_cast<rendering::DepthCamera>(this->camera);

  if (cameraSdf->HasElement("noise"))
  {
    this->noises[CAMERA_NOISE] =
        this->CreateNoiseModel(cameraSdf->GetElement("noise"), this->Type());

    this->noises[CAMERA_NOISE]->SetCamera(this->camera);

    auto postRenderImageNoise = std::dynamic_pointer_cast<PostRenderImageNoise>(
        this->noises[CAMERA_NOISE]);
    if (postRenderImageNoise != nullptr)
    {
      const auto nearClip = this->dataPtr->depthCamera->NearClip();
      const auto farClip = this->dataPtr->depthCamera->FarClip();
      const auto depthFrameCb =
        [postRenderImageNoise,nearClip,farClip](const float* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat)
        {
          // HACK: there's no better way to alter the generated depth image than hooking the
          // newDepthFrame callback which is passing a const pointer to the data.
          // Using the hack below, we'll know to be the first hook
          // that gets called, and we also know that we can const_cast the passed data
          // (because the underlying data structure is on the heap, which is always modifiable).
          auto writableBuffer = const_cast<float*>(_buffer);
          postRenderImageNoise->ApplyFloat(writableBuffer, _width, _height, _depth, _pixelFormat);

          for (size_t i = 0; i < _width * _height * _depth; ++i)
          {
            if (writableBuffer[i] < nearClip)
              writableBuffer[i] = -ignition::math::INF_F;
            else if (writableBuffer[i] > farClip)
              writableBuffer[i] = ignition::math::INF_F;
          }
        };

      // We need to be the first callback, so we'll hack into the newDepthFrame event and get the first place
      // Using DepthCameraPrivate is fragile, but we hope that it won't change in the future. This corresponds
      // to its version from Gazebo 11.15.0 (valid for all versions 11.* up to (at least) 11.15).
      gazebo::rendering::DepthCameraPrivate* privData = this->DepthCamera()->dataPtr.get();
      const auto index = privData->newDepthFrame.connections.empty() ?
        0 : privData->newDepthFrame.connections.begin()->first - 1;
      privData->newDepthFrame.connections[index] =
        std::make_unique<decltype(privData->newDepthFrame)::EventConnection>(true, depthFrameCb);
      this->dataPtr->depthFrameConnection.reset(new gazebo::event::Connection(&privData->newDepthFrame, index));
    }
  }

  this->dataPtr->worldResetConnection = event::Events::ConnectWorldReset(
      std::bind(&NoisyDepthCameraSensor::Reset, this));
  this->dataPtr->timeResetConnection = event::Events::ConnectTimeReset(
      std::bind(&NoisyDepthCameraSensor::Reset, this));
}

NoisyDepthCameraSensor::~NoisyDepthCameraSensor() // NOLINT(hicpp-use-equals-default,modernize-use-equals-default)
{
}

void NoisyDepthCameraSensor::Reset()
{
}

NoisePtr NoisyDepthCameraSensor::CreateNoiseModel(sdf::ElementPtr _sdf,
                                                  const std::string &_sensorType)
{
  GZ_ASSERT(_sdf != nullptr, "noise sdf is null");
  GZ_ASSERT(_sdf->GetName() == "noise", "Not a noise SDF element");

  std::string typeString = _sdf->Get<std::string>("type");

  if (typeString == "gaussian" && _sensorType == "depth")
  {
    NoisePtr noise(new DepthImageGaussianNoiseModel());

    GZ_ASSERT(noise->GetNoiseType() == Noise::GAUSSIAN,
              "Noise type should be 'gaussian'");

    noise->Load(_sdf);
    return noise;
  }
  else if (typeString == "gaussian_multiplicative" && _sensorType == "depth")
  {
    NoisePtr noise(new MultiplicativeGaussianNoiseModel());

    GZ_ASSERT(noise->GetNoiseType() == Noise::GAUSSIAN,
              "Noise type should be 'gaussian'");

    noise->Load(_sdf);
    return noise;
  }
  // TODO: implement stereo noise model from:
  // - https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6375037
  // - https://github.com/HannesKeller/sensor_model
  else
  {
    return NoiseFactory::NewNoiseModel(_sdf, _sensorType);
  }
}

}
}