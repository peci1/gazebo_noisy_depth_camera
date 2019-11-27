#ifndef GAZEBO_NOISY_DEPTH_CAMERA_POSTRENDERIMAGENOISE_H
#define GAZEBO_NOISY_DEPTH_CAMERA_POSTRENDERIMAGENOISE_H

#include <gazebo/msgs/image.pb.h>
#include <gazebo/common/Image.hh>

namespace gazebo
{
namespace sensors
{

/**
 * A noise model that can be added to a whole image after it is rendered.
 */
class PostRenderImageNoise
{
  public: virtual void ApplyFloat(float* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat)
  {
    throw std::runtime_error("Not implemented");
  }

  public: virtual void ApplyByte(unsigned char* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat)
  {
    throw std::runtime_error("Not implemented");
  }

  /**
   * \brief Apply this noise to the image.
   * \param image The image to add noise to.
   */
  public: virtual void Apply(msgs::Image& image)
  {
    switch (image.pixel_format())
    {
      case common::Image::R_FLOAT32:
        this->ApplyFloat(reinterpret_cast<float*>(image.mutable_data()), image.width(), image.height(), 1, "FLOAT32");
        break;
      case common::Image::RGB_INT8:
        this->ApplyByte(reinterpret_cast<unsigned char*>(image.mutable_data()), image.width(), image.height(), 3, "R8G8B8");
        break;
      case common::Image::BGR_INT8:
        this->ApplyByte(reinterpret_cast<unsigned char*>(image.mutable_data()), image.width(), image.height(), 3, "B8G8R8");
        break;
      case common::Image::L_INT8:
        this->ApplyByte(reinterpret_cast<unsigned char*>(image.mutable_data()), image.width(), image.height(), 1, "L8");
        break;
      case common::Image::BAYER_RGGB8:
        this->ApplyByte(reinterpret_cast<unsigned char*>(image.mutable_data()), image.width(), image.height(), 1, "BAYER_RGGB8");
        break;
      case common::Image::BAYER_GBRG8:
        this->ApplyByte(reinterpret_cast<unsigned char*>(image.mutable_data()), image.width(), image.height(), 1, "BAYER_GBRG8");
        break;
      case common::Image::BAYER_GRBG8:
        this->ApplyByte(reinterpret_cast<unsigned char*>(image.mutable_data()), image.width(), image.height(), 1, "BAYER_GRBG8");
        break;
      default:
        throw std::runtime_error("Unknown pixel format " + std::to_string(image.pixel_format()));
    }
  }
};

}
}

#endif //GAZEBO_NOISY_DEPTH_CAMERA_POSTRENDERIMAGENOISE_H
