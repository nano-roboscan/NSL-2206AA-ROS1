#include "update_controller.h"
#include "communication.h"

using namespace std;

namespace ComLib
{

UpdateController::UpdateController(Communication *commuication)
{
  this->communication = commuication;
  state = UpdateState_e::UPDATE_STATE_IDLE;
}

void UpdateController::startUpdate(const vector<uint8_t> &updateFile)
{
  this->updateFile = updateFile;
  index = 0;
  sizeRemaining = updateFile.size();
  sizeTotal = updateFile.size();

  state = UPDATE_STATE_WAIT_BOOTLOADER;
  communication->sendCommandJumpToBootloader();
}

void UpdateController::cancel()
{
  state = UPDATE_STATE_IDLE;
}

void UpdateController::onReceivedAck()
{
  switch(state)
  {
    case UPDATE_STATE_IDLE:
      //Do nothing
      break;
    case UPDATE_STATE_WAIT_BOOTLOADER:
      state = UPDATE_STATE_WRITE;
      communication->sendCommandFirmwareUpdateStart(sizeTotal);
      break;
    case UPDATE_STATE_WRITE:
    {
      unsigned int bytesToWrite = 4;
      if (sizeRemaining < bytesToWrite)
      {
        bytesToWrite = sizeRemaining;
      }

      uint8_t *data = (uint8_t *)updateFile.data();
      communication->sendCommandFirmwareUpdateWriteData(&data[index], index, bytesToWrite);

      index += bytesToWrite;
      sizeRemaining -= bytesToWrite;

      //Go here only to 99%
      unsigned int progress = (index * 99) / sizeTotal;

      //Update the progress      
      sigUpdateProgress(progress);

      //Check if finished
      if (sizeRemaining == 0)
      {
        state = UPDATE_STATE_FINISH;
      }
      break;
    }
    case UPDATE_STATE_FINISH:
      state = UPDATE_STATE_WAIT_FINISHED;

      //Finally send a command, that the update is finished
      communication->sendCommandFirmwareUpdateFinished();
      break;
    case UPDATE_STATE_WAIT_FINISHED:
      state = UPDATE_STATE_IDLE;
      sigUpdateProgress(100);
      sigUpdateFinished();
      break;
    default:
      break;
  }
}
}
