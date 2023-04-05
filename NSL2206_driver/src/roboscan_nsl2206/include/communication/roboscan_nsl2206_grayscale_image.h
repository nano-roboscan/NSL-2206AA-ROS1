#ifndef NSL2206_GRAYSCALE_IMAGE_H
#define NSL2206_GRAYSCALE_IMAGE_H

#include "roboscan_nsl2206_image.h"

namespace ComLib
{

class Nsl2206GrayscaleImage: public Nsl2206Image
{
  public:
    using Nsl2206Image::Nsl2206Image;
    using Nsl2206Image::getGrayscaleOfPixel;
    Nsl2206ImageType_e getType();    
};

}

#endif // NSL2206_GRAYSCALE_IMAGE_H
