#include "u16_helper.h"

namespace ComLib
{

U16Helper::U16Helper()
{
  value = 0;
}

uint16_t U16Helper::getValue()
{
  return value;
}

void U16Helper::onReceivedData(const uint16_t value)
{
  this->value = value;
}

}
