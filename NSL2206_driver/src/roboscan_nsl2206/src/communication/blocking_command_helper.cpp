#include "blocking_command_helper.h"

namespace ComLib
{

BlockingCommandHelper::BlockingCommandHelper()
{
  finished = false;
  errorNumber = ErrorNumber_e::ERROR_NUMMBER_NO_ERROR;
}

void BlockingCommandHelper::onCommandFinished()
{
  finished = true;
}

void BlockingCommandHelper::onError(const ErrorNumber_e errorNumber)
{
  this->errorNumber = errorNumber;
  finished = true;
}

bool BlockingCommandHelper::isBusy()
{
  if (finished == false)
  {
    return true;
  }

  return false;
}

ErrorNumber_e BlockingCommandHelper::getErrorNumber()
{
  return errorNumber;
}

}
