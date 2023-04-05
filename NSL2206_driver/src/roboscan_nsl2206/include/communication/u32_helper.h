/**
 * nanosystems
 */

#ifndef U32HELPER_H
#define U32HELPER_H

#include <stdint.h>

namespace ComLib
{
//! Helper class for 32Bit values
/*!
 * This class helps, when 32Bit data is being read blocking. It receives the signals and
 * stores the value. Later it can be read using the getter functions. There are functions for the
 * lsb/msb only or for the whole value.
 */
class U32Helper
{
  public:
    U32Helper();
    uint32_t getValue();
    uint16_t getValueLsb();
    uint16_t getValueMsb();
    void onReceivedData(const uint32_t value); //boost slot

  private:
    uint32_t value;
};

}

#endif // U32HELPER_H

/** @} */
