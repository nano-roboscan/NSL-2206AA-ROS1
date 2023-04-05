#include "roboscan_nsl2206_distance_grayscale_image.h"

namespace ComLib
{

#pragma pack(push,1)
typedef struct
{
    uint8_t grayscale;
    uint16_t distance;
}distanceGrayscale_t;
#pragma pack(pop)

Nsl2206Image::Nsl2206ImageType_e Nsl2206DistanceGrayscaleImage::getType()
{
  return Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_GRAYSCALE;
}

unsigned int Nsl2206DistanceGrayscaleImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::Nsl2206Header::SIZE;

  distanceGrayscale_t *distanceGrayscale =  (distanceGrayscale_t *)(&data.data()[dataOffset]);

  return distanceGrayscale[index].distance & CommunicationConstants::PixelNsl2206::MASK_OUT_CONFIDENCE;
}

unsigned int Nsl2206DistanceGrayscaleImage::getGrayscaleOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::Nsl2206Header::SIZE;

  distanceGrayscale_t *distanceGrayscale =  (distanceGrayscale_t *)(&data.data()[dataOffset]);

  return distanceGrayscale[index].grayscale;
}

}
