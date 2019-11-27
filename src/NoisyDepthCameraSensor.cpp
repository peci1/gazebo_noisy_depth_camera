#include <gazebo_noisy_depth_camera/NoisyDepthCameraSensor.h>
#include <gazebo_noisy_depth_camera/DepthImageGaussianNoiseModel.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/common/SystemPaths.hh>
#include <OGRE/OgreResourceGroupManager.h>

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
  const NoisyDepthCameraSensor* parentObject;

  event::ConnectionPtr worldResetConnection;
  event::ConnectionPtr timeResetConnection;

  explicit NoisyDepthCameraSensorPrivate(const NoisyDepthCameraSensor* _parentObject)
  {
    this->parentObject = _parentObject;
  }
};

NoisyDepthCameraSensor::NoisyDepthCameraSensor() :
  dataPtr(new NoisyDepthCameraSensorPrivate(this))
{
}

void NoisyDepthCameraSensor::Load(const std::string &_worldName)
{

  DepthCameraSensor::Load(_worldName);

  gzmsg << "Noisy depth camera loaded" << std::endl;
}

std::string NoisyDepthCameraSensor::Topic() const
{
  return DepthCameraSensor::Topic();
}

void NoisyDepthCameraSensor::Init()
{
  sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");

  DepthCameraSensor::Init();

  if (cameraSdf->HasElement("noise"))
  {
    this->noises[CAMERA_NOISE] =
        this->CreateNoiseModel(cameraSdf->GetElement("noise"), this->Type());

    this->noises[CAMERA_NOISE]->SetCamera(this->camera);
    gzwarn << "Found noise" << std::endl;
  }

  this->dataPtr->worldResetConnection = event::Events::ConnectWorldReset(
      std::bind(&NoisyDepthCameraSensor::Reset, this));
  this->dataPtr->timeResetConnection = event::Events::ConnectTimeReset(
      std::bind(&NoisyDepthCameraSensor::Reset, this));
}

void NoisyDepthCameraSensor::Fini()
{
  DepthCameraSensor::Fini();
}

bool NoisyDepthCameraSensor::UpdateImpl(const bool force)
{
  return DepthCameraSensor::UpdateImpl(force);
}

NoisyDepthCameraSensor::~NoisyDepthCameraSensor() // NOLINT(hicpp-use-equals-default,modernize-use-equals-default)
{
}

bool NoisyDepthCameraSensor::IsActive() const
{
  return gazebo::sensors::CameraSensor::IsActive();
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
  else
  {
    return NoiseFactory::NewNoiseModel(_sdf, _sensorType);
  }
}

}
}