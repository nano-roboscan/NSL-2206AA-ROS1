/**
 * nanosystems
 */

#ifndef BLOCKINGCOMMANDHELPER_H
#define BLOCKINGCOMMANDHELPER_H

#include "communication_if.h"

namespace ComLib
{

//! Helper class for blocking commands
/*!
 * When blocking commands are used, this calss receives the signals and stores the values. The blocking caller
 * then can get the values with the corresponding getter functions.
 */
class BlockingCommandHelper
{

  public:
    BlockingCommandHelper();
    bool isBusy();
    ErrorNumber_e getErrorNumber();

  //public slots
    void onCommandFinished();
    void onError(const ErrorNumber_e errorNumber);

  private:
    bool finished;
    ErrorNumber_e errorNumber;
};
}

#endif // BLOCKINGCOMMANDHELPER_H

/** @} */
