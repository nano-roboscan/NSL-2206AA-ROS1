#include "roboscan_nsl2206_grayscale_image.h"

namespace ComLib
{

Nsl2206Image::Nsl2206ImageType_e Nsl2206GrayscaleImage::getType()
{
  return Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_GRAYSCALE;
}

}
