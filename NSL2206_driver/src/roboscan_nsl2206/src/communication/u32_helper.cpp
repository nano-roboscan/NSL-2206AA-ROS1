#include "u32_helper.h"
#include <iostream>

namespace ComLib
{

U32Helper::U32Helper()
{
  value = 0;
}

uint32_t U32Helper::getValue()
{
  return value;
}

void U32Helper::onReceivedData(const uint32_t value)
{  
  this->value = value;
}

uint16_t U32Helper::getValueLsb()
{
  return (value & 0xFFFF);
}

uint16_t U32Helper::getValueMsb()
{
  return (value >> 16);
}

}
