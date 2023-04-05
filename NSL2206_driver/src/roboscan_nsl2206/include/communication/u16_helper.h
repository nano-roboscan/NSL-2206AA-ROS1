/**
 * nanosystems
 */

#ifndef U16HELPER_H
#define U16HELPER_H

#include <stdint.h>

namespace ComLib
{
//! Helper class for 16Bit values
/*!
 * This class helps, when 16Bit data is being read blocking. It receives the signals and
 * stores the value. Later it can be read using the getter functions. There are functions for the
 * lsb/msb only or for the whole value.
 */
class U16Helper
{
  public:
    U16Helper();
    uint16_t getValue();  
    void onReceivedData(const uint16_t value); //boost slot

  private:
    uint16_t value;
};

}

#endif // U16HELPER_H

/** @} */
