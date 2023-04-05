#ifndef NSL2206_DISTANCE_GRAYSCALE_IMAGE_H
#define NSL2206_DISTANCE_GRAYSCALE_IMAGE_H

#include "roboscan_nsl2206_image.h"

namespace ComLib
{

class Nsl2206DistanceGrayscaleImage: public Nsl2206Image
{
  public:
    using Nsl2206Image::Nsl2206Image;
    Nsl2206ImageType_e getType();
    unsigned int getDistanceOfPixel(const unsigned int index) const;
    unsigned int getGrayscaleOfPixel(const unsigned int index) const;
};

}

#endif // NSL2206_DISTANCE_GRAYSCALE_IMAGE_H
