#include "roboscan_nsl2206_distance_amplitude_image.h"

namespace ComLib
{

Nsl2206Image::Nsl2206ImageType_e Nsl2206DistanceAmplitudeImage::getType()
{
  return Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_AMPLITUDE;
}

unsigned int Nsl2206DistanceAmplitudeImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::Nsl2206Header::SIZE;

  //Make a pointer to the distance
  uint32_t *distanceAmplitude =  (uint32_t *)(&data.data()[dataOffset]);

  return (distanceAmplitude[index] & 0xFFFF);
}

unsigned int Nsl2206DistanceAmplitudeImage::getAmplitudeOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::Nsl2206Header::SIZE;

  //Make a pointer to the distance
  uint32_t *distanceAmplitude =  (uint32_t *)(&data.data()[dataOffset]);

  return (distanceAmplitude[index] >> 16);
}

}
