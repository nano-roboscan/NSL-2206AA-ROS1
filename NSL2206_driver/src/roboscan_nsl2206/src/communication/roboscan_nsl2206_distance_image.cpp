#include "roboscan_nsl2206_distance_image.h"

namespace ComLib
{

Nsl2206Image::Nsl2206ImageType_e Nsl2206DistanceImage::getType()
{
  return Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE;
}

unsigned int Nsl2206DistanceImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::Nsl2206Header::SIZE;

  //Make a pointer to the distance
  uint16_t *distance =  (uint16_t *)(&data.data()[dataOffset]);

  return (distance[index] & CommunicationConstants::PixelNsl2206::MASK_OUT_CONFIDENCE);
}

}
