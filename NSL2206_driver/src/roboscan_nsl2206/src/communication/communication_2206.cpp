
#include <iostream>
#include "communication_2206.h"
#include "communication_constants.h"
#include "util.h"


using namespace std;

namespace ComLib
{

///Fixed baud rate
const unsigned int BAUDRATE = 10000000;

Communication2206::Communication2206()
{
}

/**
 * @brief Process distance amplitude
 *
 * This function processes distance amplitude specific for TofRange/Frame 611
 *
 * @param array Array with received data
 */
void Communication2206::processDistanceAmplitude(const vector<uint8_t> &array)
{
//	printf("processDistanceAmplitude\n");
  std::shared_ptr<Nsl2206Image> header(new Nsl2206DistanceAmplitudeImage(array));
  sigReceivedDistanceAmplitude(header);
}

/**
 * @brief Process distance data
 *
 * This function processes distance specific for epc635
 *
 * @param array Array with received data
 */
void Communication2206::processDistance(const vector<uint8_t> &array)
{
  std::shared_ptr<Nsl2206Image> header(new Nsl2206DistanceImage(array));

  //Forward the data
  sigReceivedDistance(header);  
}

/**
 * @brief Process grayscale  data
 *
 * This function processes grayscale data specific for TofRange/Frame 635
 *
 * @param array Array with received data
 */
void Communication2206::processGrayscale(const vector<uint8_t> &array)
{
  std::shared_ptr<Nsl2206Image> header(new Nsl2206GrayscaleImage(array));

  //Forward the data
  sigReceivedGrayscale(header);
}

void Communication2206::processDistanceGrayscale(const std::vector<uint8_t> &array)
{
  std::shared_ptr<Nsl2206Image> header(new Nsl2206DistanceGrayscaleImage(array));

  //Forward the data
  sigReceivedDistanceGrayscale(header);
}


void Communication2206::processLensCalibrationData(const std::vector<uint8_t> &array)
{
  sigReceivedLensCalibrationData(array);
}


/**
 * @brief Setup device
 *
 * Tasks:
 * - Check, if it is a device we are looking for
 * - Setup default values per device. This is so far:
 *    - The operation mode
 *
 * @retval true It is a correct device: TofRange611 or TofFrame611
 * @retval false No correct/wanted device
 * @return Correct device true/false
 */
bool Communication2206::setupDevice(const Device_e device)
{
  bool deviceIsOk = false;

  switch(device)
  {
    case Device_e::DEVICE_TOFCAM635:
      deviceIsOk = true;
      break;
    case Device_e::DEVICE_UNKNOWN:
      //Device unknown is also ok! This is for a fabric new device and the user must be able to connect to it
      deviceIsOk = true;
      break;
    default:
      break;
  }

  return deviceIsOk;
}

/**
 * @brief Set mode
 *
 * Call this function to set the operation mode
 *
 * @param mode Mod eto set
 */
ErrorNumber_e Communication2206::setMode(const int mode)
{
  switch(mode)
  {
    case Nsl2206Mode_e::MODE_BEAM_A:
      sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeNsl2206::MODE_BEAM_A, false);
      break;
    case Nsl2206Mode_e::MODE_BEAM_B_MANUAL:
      sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeNsl2206::MODE_BEAM_B_MANUAL, false);
      break;
    case Nsl2206Mode_e::MODE_BEAM_AB_AUTO_RESULT:
      sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeNsl2206::MODE_BEAM_AB_AUTO_RESULT, false);
      break;
    case Nsl2206Mode_e::MODE_BEAM_AB_INTERLEAVED_DATA:
      sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeNsl2206::MODE_BEAM_AB_INTERLEAVED_DATA, false);
      break;
    case Nsl2206Mode_e::MODE_BEAM_AB_RESULT:
      sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeNsl2206::MODE_BEAM_AB_RESULT, false);
      break;
    case Nsl2206Mode_e::MODE_BEAM_B_RESULT:
      sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeNsl2206::MODE_BEAM_B_RESULT, false);
      break;
    case Nsl2206Mode_e::MODE_BEAM_B_RESULT_DATA:
      sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeNsl2206::MODE_BEAM_B_RESULT_DATA, false);
      break;
    default:
      return ErrorNumber_e::ERROR_NUMBER_INVALID_PARAMETER;
  }
  return ErrorNumber_e::ERROR_NUMMBER_NO_ERROR;
}

/**
 * @brief Get the baud rate
 *
 * This function returns the baud rate for this device.
 *
 * @return baud rate
 */
unsigned int Communication2206::getBaudRate()
{
  return BAUDRATE;
}

} //end namespace ComLib
