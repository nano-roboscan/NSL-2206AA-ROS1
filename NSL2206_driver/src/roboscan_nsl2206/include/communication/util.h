/**
 * nanosystems
 */

#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <vector>

namespace ComLib
{
  //! Utility class
  /*!
   * This class offers some helper functions needed in different other classes. These helper
   * functions are static, so no instance is needed.
   */
  class Util
  {
    public:
      Util();
      static uint32_t getUint16LittleEndian(const std::vector<uint8_t> &array, const unsigned int index);
      static uint32_t getUint32LittleEndian(const std::vector<uint8_t> &array, const unsigned int index);
      static uint32_t getInt16LittleEndian(const std::vector<uint8_t> &array, const unsigned int index);
      static void setUint16LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
      static void setInt16LittleEndian(uint8_t *buffer, const unsigned int index, const int value);
      static void setUint24LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
      static void setUint32LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
  };

}

#endif // UTIL_H

/** @} */
