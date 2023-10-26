/**
 * nanosystems
 */

#include "communication.h"
#include "update_controller.h"
#include "communication_constants.h"
#include "u32_helper.h"
#include "u16_helper.h"
#include "chip_information_helper.h"
#include "production_information_helper.h"
#include "blocking_command_helper.h"
#include "util.h"
#include <iostream>
#include <cstring>
#include <ros/ros.h>

#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

#define IDENTIFY_SIZE                 12
#define GET_TEMPERATURE_SIZE          10
#define SET_INTEGRATION_TIME_DIS_SIZE  8
#define GET_INTEGRATION_TIME_DIS_SIZE 10
#define GET_CHIP_INFORMATION_SIZE     12
#define GET_FIRMWARE_VERSION_SIZE     12

#define SET_ROI_SIZE  8

#define GET_DISTANCE_GRAYSCALE_SIZE 28880
#define GET_DISTANCE_AMPLITUDE_SIZE 38480
#define GET_DISTANCE_SIZE           19280
#define GET_GRAYSCALE_SIZE           9680


using namespace std;

namespace ComLib
{

//There are two different timeout times. During connecting, the timeout is short, becuse on each com port a device will be searched. To prevent long searching time, the timeout must be small.
static const unsigned int TIMEOUT_CONNECTING = 100;  ///<Communication timeout in ms during connecting phase
static const unsigned int TIMEOUT_NORMAL = 1000;     ///<Communication timeout in ms during normal operaion

Communication::Communication(): updateController(this)
{  
  serialConnection = new SerialConnection;

  timeout = TIMEOUT_CONNECTING;
  timeoutTimer = new EpcTimer(std::chrono::milliseconds(timeout), boost::bind(&Communication::onTimeout, this));

  serialConnection->sigReceivedData.connect(boost::bind(&Communication::onReceivedData, this, _1, _2));

  //Signals update controller to communication
  updateController.sigUpdateProgress.connect(boost::bind(&Communication::onFirmwareUpdateProgress, this, _1));
  updateController.sigUpdateFinished.connect(boost::bind(&Communication::onFirmwareUpdateFinished, this));

  //Signals from communication to update controller
  sigReceivedAck.connect(boost::bind(&UpdateController::onReceivedAck, &updateController));

  //Member initialization
  state = CommunicationState_e::COMMUNICATION_STATE_UNCONNECTED;
  connectedDevice = Device_e::DEVICE_UNKNOWN;

}

Communication::~Communication()
{
  timeoutTimer->stop();
  delete timeoutTimer;
  delete serialConnection;
}

/**
 * @brief List the available devices
 *
 * This function is used to list the available devices. The index of a device in the list can later be
 * used to select the port to open. The strings can for example be directly put into a comboBox in the GUI and
 * the index is directly given by the comboBox.
 *
 * Only the ports get listed, where a valid device is found
 *
 * @return List of strings containing the names of the available devices
 */
std::list<std::string> Communication::availableDevices()
{
  deviceIdList.clear();
  deviceNameList.clear();
  timeout = TIMEOUT_CONNECTING;

  vector<string> devices = serialConnection->availableDevices();

  for(int i=0; i<devices.size(); i++)
      ROS_INFO_STREAM("available devices: " << devices[i].data());


  //Try all ports
  for (int id = 0; id < devices.size(); id++)
  {
    Device_e device;
    //bool isBootloader;
    //bool openOk = openInternal(id, device, isBootloader);

    //If it could be opened, this means it is a valid device. Important: close it again
    //if(openOk){
      deviceIdList.push_back(id);

      //Combine the port name with the device name, for example COM4 - Tof>range 635
      string deviceName = devices[id] + " - " + createDeviceString(device);
      ROS_INFO_STREAM(deviceName);
      deviceNameList.push_back(deviceName);
      //close();
    //}
  }

  return deviceNameList;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool Communication::openInternal(Device_e &device, bool &isBootloader)
{
  if(!serialConnection->openPort()){
      return false;
  }  

  //If the port is open, try to read the identification. If this works, assume the connection as ok
  ErrorNumber_e errorNumber = getIdentification(device, isBootloader);

  if ((errorNumber != ErrorNumber_e::ERROR_NUMMBER_NO_ERROR) || (setupDevice(device) == false))
  {
      close();
      ROS_ERROR("Identification could not be read. It is probably a valid serial port but with no device");
      return false;
  }

  return true;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool Communication::open()
{

  bool deviceIsInBootloader = false;  

  bool connected = openInternal(connectedDevice, deviceIsInBootloader);

  if(connected){
    state = COMMUNICATION_STATE_NORMAL;
    timeout = TIMEOUT_NORMAL;
  }

  return connected;
}

/**
 * @brief Close the serial port
 *
 */
void Communication::close()
{
  serialConnection->closePort();
  state = COMMUNICATION_STATE_UNCONNECTED;
}

/**
 * @brief Create a device name
 *
 * This function creates a device string depending on the device ID given
 *
 * @param device Found device
 * @return QString containing the device ID
 */
string Communication::createDeviceString(const Device_e device)
{
  string deviceString;

  switch(device)
  {
    case Device_e::DEVICE_TOFFRAME611:
      deviceString.append("Tof>frame 611");
      break;
    case Device_e::DEVICE_TOFRANGE611:
      deviceString.append("Tof>range 611");
      break;
    case Device_e::DEVICE_UNKNOWN:
      deviceString.append("unknown device");
      break;
    default:
      deviceString.append("invalid device");
      break;
  }

  return deviceString;
}

/**
 * @brief Get the device name
 *
 * This function returns the device name. The device name is created depending on the
 * device ID read when connecting.
 *
 * @return QString containing the device ID
 */
string Communication::getDeviceName()
{
  return createDeviceString(connectedDevice);
}

/**
 * @brief Send error signal to connected slots
 *
 * This function emits the error signal. It emits it anyway internally. If the connection is established
 * it emits it also externally. The reason is, that during the connecting phase, some commands are sent to detect
 * if a device is listening. If no device answers, there would be an error.
 *
 * @param errorNumber Error number to send
 */

void Communication::sendErrorSignal(const ErrorNumber_e errorNumber)
{
  //Emit the error internally
  sigErrorInternal(errorNumber); //lsi


  //Emit the error external
  switch(state)
  {
    case CommunicationState_e::COMMUNICATION_STATE_NORMAL:
    //no break
    case CommunicationState_e::COMMUNICATION_STATE_UPDATE:
      ROS_ERROR_STREAM("Error: " << errorNumber);
      sigError(errorNumber);
      break;
    default:
      //Do not emit the error external
      break;
  }
}

/**
 * @brief helper function to send the command
 *
 * All commands are sent over this function. Here also the timeouts
 * are handled.
 *
 * @param data Pointer to the already filled data to send
 * @param blocking set to true to run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommand(uint8_t *data, int size)
{  
  serialConnection->sendData(data);

  int sz = 0;
  int count = 0;
  serialConnection->rxArray.clear();

  for(int n= 0; n < size; n+= sz){
      sz = serialConnection->readRxData(size, true);
      if(sz == -1){		
	  printf("Communication::sendCommand serialConnection->readRxData sz=-1 count = %d ", count);
      //serialConnection->processRemainData(size);
	    return ERROR_NUMBER_SERIAL_PORT_ERROR;
      }else{
         count += sz;
      }
  }
    if( serialConnection->rxArray.size() > 0) serialConnection->processData(serialConnection->rxArray, size);
	else{
	  printf("arr zero ~~\n");
	}
  return ERROR_NUMMBER_NO_ERROR;
}

/**
 * @brief Send a command without data
 *
 * This function is used for commands without any payload.
 *
 * @param command Command to send
 * @param blocking set to true to run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandWithoutData(const uint8_t command, int size)
{
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = command;

  return sendCommand(output, size);
}

/**
 * @brief Send single byte command
 *
 * This function is used for commands with only one byte of payload.
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandSingleByte(const uint8_t command, const uint8_t payload, const bool blocking)
{
  std::ignore = blocking;
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = command;

  //Add the single byte at the first position
  output[CommunicationConstants::Command::INDEX_DATA] = payload;

  return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}

/**
 * @brief Send 16bit / 2byte command
 *
 * This function is used for commands with one 16bit value as payload
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandUint16(const uint8_t command, const uint16_t payload, const bool blocking)
{
  std::ignore = blocking;
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = command;

  //Add the payload
  Util::setUint16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA, payload);

  return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}


/**
 * @brief Send 16bit / 2byte command signed
 *
 * This function is used for commands with one 16bit value as payload
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandInt16(const uint8_t command, const int16_t payload, bool blocking)
{
  std::ignore = blocking;
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = command;

  //Add the payload
  Util::setInt16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA, payload);

  return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}



/**
 * @brief Send 2 x 16bit / 4byte command
 *
 * This function is used for commands with two 16bit value as payload
 *
 * @param command Command to send
 * @param payload0 First payload value to send
 * @param payload1 Second payload value to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommand2xUint16(const uint8_t command, const uint16_t payload0, const uint16_t payload1, const bool blocking)
{
  std::ignore = blocking;
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = command;

  //Add the payload values
  Util::setUint16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA, payload0);
  Util::setUint16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA+2, payload1);

  return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}

/**
 * @brief Timeout callback
 *
 * This function is called, if a command times out. It is used to handle the timeouts
 */
void Communication::onTimeout()
{
  //Prevent the timer of generating further signals
  timeoutTimer->stop();

  switch(state)
  {
    case CommunicationState_e::COMMUNICATION_STATE_UPDATE:
      updateController.cancel();
      onFirmwareUpdateProgress(0);
      state = CommunicationState_e::COMMUNICATION_STATE_NORMAL;
      break;    
    default:
      break;
  }

  sendErrorSignal(ErrorNumber_e::ERROR_NUMBER_TIMEOUT);
}



/**
 * @brief Forward the update progress
 *
 * During the update the update controller is controlling the communication. It generates
 * signals with update progress information. Here forward these signals.
 *
 * @param progress Update progress in percent
 */
void Communication::onFirmwareUpdateProgress(const unsigned int progress)
{
  sigFirmwareUpdateProgress(progress);
}

/**
 * @brief Update finished
 *
 * When the update controller has finished the update, it fires a signal. This is received here.
 */
void Communication::onFirmwareUpdateFinished()
{
  state = CommunicationState_e::COMMUNICATION_STATE_NORMAL;
}

/**
 * @brief Process identification data
 *
 * This function is called when identification data is received.
 *
 * @param array Pointer to the received data
 */
void Communication::processIdentification(const std::vector<uint8_t> &array)
{
  //Here the array is already cut to the payload, so get the data at index 0
  uint32_t identification = Util::getUint32LittleEndian(array, 0);
  sigReceivedIdentification(identification);
}

/**
 * @brief Process chip information data
 *
 * This function is called when chip information data is received.
 *
 * @param array Pointer to the received data
 */
void Communication::processChipInformation(const std::vector<uint8_t> &array)
{
  //Here the array is already cut to the payload, so subtract the header size from the indexes
  uint16_t waferId = Util::getUint16LittleEndian(array, CommunicationConstants::ChipInformation::INDEX_WAFER_ID - CommunicationConstants::Data::SIZE_HEADER);
  uint16_t chipId = Util::getUint16LittleEndian(array, CommunicationConstants::ChipInformation::INDEX_CHIP_ID - CommunicationConstants::Data::SIZE_HEADER);
  sigReceivedChipInformation(chipId, waferId);
  printf("waferId = %d, chipId = %d", waferId, chipId);
}

/**
 * @brief Process firmware release data
 *
 * This function is called when the firmware release data is received.
 *
 * @param array Pointer to the received data
 */
void Communication::processFirmwareRelease(const std::vector<uint8_t> &array)
{
  //Here the array is already cut to the payload, so get the data at index 0
  uint32_t firmwareRelease = Util::getUint32LittleEndian(array, 0);
  sigReceivedFirmwareRelease(firmwareRelease);
  printf("firmwareRelease = %x / %x\n", firmwareRelease>>16, firmwareRelease&0xFF);
}

/**
 * @brief Process integration time data
 *
 * This function is called when integration time data is received
 *
 * @param array Pointer to the received data
 */
void Communication::processIntegrationTime(const std::vector<uint8_t> &array)
{
  //Here the array is already cut to the payload, so get the data at index 0
  uint16_t integrationTime = Util::getUint16LittleEndian(array, 0);
  sigReceivedIntegrationTime(integrationTime);
}

/**
 * @brief Process temperature data
 *
 * This function is called when temperature data is received
 *
 * @param array Pointer to the received data
 */
void Communication::processTemperature(const std::vector<uint8_t> &array)
{
  //Here the array is already cut to the payload, so get the data at index 0
  int16_t temperature = Util::getInt16LittleEndian(array, 0);
  sigReceivedTemperature(temperature);
  ROS_INFO_STREAM("temperature = " << temperature);
}

/**
 * @brief Process production info
 *
 * This function is called when the production info is received
 *
 * @param array Pointer to the received data
 */
void Communication::processProductionInfo(const std::vector<uint8_t> &array)
{
  //Here the array is already cut to the payload, so get the data at index 0
  uint8_t year = array.at(0);
  uint8_t week = array.at(1);
  sigReceivedProductionInfo(year, week);
  ROS_INFO_STREAM("year = " << year << "week = " << week);
}

/**
 * @brief Handle received data
 *
 * This function is called when data from the device has been received.
 * It handles the received data depending on the type.
 *
 * @param array Pointer to the received data
 * @param type Type of the data
 */
void Communication::onReceivedData(const std::vector<uint8_t> &array, const uint8_t type)
{
  //Stop the timeout timer
  //timeoutTimer->stop();

  
  switch(type)
  {
    case CommunicationConstants::Type::DATA_ACK:
      sigReceivedAck();
      //printf("received Ack\n");
      break;
    case CommunicationConstants::Type::DATA_NACK:
      sendErrorSignal(ErrorNumber_e::ERROR_NUMBER_NOT_ACKNOWLEDGE);
      //printf("received Nack\n");
      break;
    case CommunicationConstants::Type::DATA_IDENTIFICATION:
      processIdentification(array);
      //printf("received Identification\n");
      break;
    case CommunicationConstants::Type::DATA_CHIP_INFORMATION:
      processChipInformation(array);
      //printf("received Chip Information\n");
      break;
    case CommunicationConstants::Type::DATA_DISTANCE_AMPLITUDE:
      processDistanceAmplitude(array);
      ROS_DEBUG("received distance amplitude");
      break;
    case CommunicationConstants::Type::DATA_DISTANCE_GRAYSCALE:
      processDistanceGrayscale(array);
      ROS_DEBUG("received distance amplitude");
      break;
    case CommunicationConstants::Type::DATA_TEMPERATURE:
      processTemperature(array);
      ROS_DEBUG("received temperature");
      break;
    case CommunicationConstants::Type::DATA_DISTANCE:
      processDistance(array);
      ROS_DEBUG("received distance");
      break;
    case CommunicationConstants::Type::DATA_GRAYSCALE:
      processGrayscale(array);
      ROS_DEBUG("receivedGrayscale");
      break;      
    case CommunicationConstants::Type::DATA_LENS_CALIBRATION_DATA:
      processLensCalibrationData(array);
      break;
    case CommunicationConstants::Type::DATA_FIRMWARE_RELEASE:
      processFirmwareRelease(array);
      break;
    case CommunicationConstants::Type::DATA_INTEGRATION_TIME:
      processIntegrationTime(array);
      break;
    case CommunicationConstants::Type::DATA_PRODUCTION_INFO:
      processProductionInfo(array);
      break;
	case CommunicationConstants::Type::DATA_TRACE:
	  break;
    default:
      printf("received unknown type= %x\n", type);
      break;
  }

  sigReceivedAnswer();
}

/**
 * @brief Handle serial port error
 *
 * This function is called when the serial port has an error.
 *
 * @param errorMessage Error message, not used here
 */
/*//TODO...
void Communication::onError(QSerialPort::SerialPortError errorMessage __attribute__((unused)))
{
  //Stop the timeout timer, because the connection is broken anyway
  timeoutTimer->stop();
  sendErrorSignal(ErrorNumber_e::ERROR_NUMBER_SERIAL_PORT_ERROR);
}
*/

/***************************************************************************
 * Internal update commands. These commands are public, but not in the
 * interface, because they are not used by the GUI, but just by the update
 * controller.
 ***************************************************************************/
/**
 * @brief Update procedure start
 *
 * This command has to be sent at the beginning of the update procedure. The bootloader will not accept any data without
 * sending this command at the beginning.
 *
 * @param fileSize Size of the update file
 */
void Communication::sendCommandFirmwareUpdateStart(const unsigned int fileSize)
{
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Write the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = CommunicationConstants::CommandList::COMMAND_UPDATE_FIRMWARE;

  //Write the update password
  Util::setUint24LittleEndian(output, CommunicationConstants::Update::INDEX_INDEX, CommunicationConstants::Update::PASSWORD_DELETE);

  //Write the control bytes
  output[CommunicationConstants::Update::INDEX_CONTROL] = CommunicationConstants::Update::CONTROL_START;

  //Write the file size
  Util::setUint32LittleEndian(output, CommunicationConstants::Update::INDEX_DATA, fileSize);

  sendCommand(output, false);
}

/**
 * @brief Update procedure write data
 *
 * This command has to be sent during the update procedure as long as there is data to send. It writes a maximum of 4 bytes
 * of data to the device.
 *
 * @param dataToWrite Pointer to the data to write
 * @param index Index of the actual data in relation to the whole data
 * @param bytesToWrite Number of bytes to write. This is maximal 4 and usual 4
 */
void Communication::sendCommandFirmwareUpdateWriteData(const uint8_t *dataToWrite, const uint32_t index, const unsigned int bytesToWrite)
{
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Write the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = CommunicationConstants::CommandList::COMMAND_UPDATE_FIRMWARE;

  //Write the control byte
  output[CommunicationConstants::Update::INDEX_CONTROL] = CommunicationConstants::Update::CONTROL_WRITE_DATA;

  //Write the index
  Util::setUint24LittleEndian(output, CommunicationConstants::Update::INDEX_INDEX, index);

  //Clear and copy the payload
  memset(&output[CommunicationConstants::Update::INDEX_DATA], 0, CommunicationConstants::Command::SIZE_PAYLOAD);
  memcpy(&output[CommunicationConstants::Update::INDEX_DATA], dataToWrite, bytesToWrite);

  sendCommand(output, false);
}

/**
 * @brief Update procedure finished
 *
 * This command has to be sent at the end of the update procedure. On receiving this command, the bootloader
 * starts the application, if it is valid.
 */
void Communication::sendCommandFirmwareUpdateFinished()
{
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Write the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = CommunicationConstants::CommandList::COMMAND_UPDATE_FIRMWARE;

  //Write the control byte
  output[CommunicationConstants::Update::INDEX_CONTROL] = CommunicationConstants::Update::CONTROL_COMPLETE;

  sendCommand(output, false);
}

/**
 * @brief Jump to bootloader
 *
 * This command makes the device jump to the bootloader. The bootloader then sends an acknowledge.
 * This is used for the firmware upate procedure.
 */
void Communication::sendCommandJumpToBootloader()
{
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_JUMP_TO_BOOTLOADER, false);
}


/***************************************************************************
 * General commands blocking and non blocking
 ***************************************************************************/
/**
 * @brief Enable or disable power
 *
 * @param enabled True to enable the power, else false
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setPower(const bool enabled)
{
  uint8_t controlByte = 0;

  //Set the control byte --> true != 1
  if (enabled)  controlByte = 1;

  return sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_POWER, controlByte, true);
}


/***************************************************************************
 * Information commands --> blocking and non blocking
 ***************************************************************************/
/**
 * @brief Request the identification
 *
 * This identification helps to detect, which device is connected to the PC and if the bootloader
 * is active or the application. When the bootloader is active, the flag "isBootloader" is set.
 * In addition the version of the device is read. This is the coast version and has nothing to do with
 * the firmware version.
 *
 * @param device Reference where the device is written to
 * @param isBootloader Reference to the bootloader flag
 * @param version Reference where the version is written to
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getIdentification(Device_e &device, bool &isBootloader, unsigned int &version)
{
  U32Helper identificationHelper;

  //Temporary connect the signal to the helper  
  boost::signals2::connection cn = sigReceivedIdentification.connect(boost::bind(&U32Helper::onReceivedData,  &identificationHelper, _1));

  //Send this command blocking
  ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_IDENTIFY, IDENTIFY_SIZE);

  //The helper has the value
  uint32_t identificationValue = identificationHelper.getValue();

  //Disconnect the signal from the helper
  cn.disconnect();

  //Check, if the Bootloader Flag is set
  isBootloader = false;
  if (identificationValue & CommunicationConstants::Identification::VALUE_BOOTLOADER)
      isBootloader = true;

  //Mask out the combination of chip type and devic
  unsigned int chipTypeDevice = (identificationValue & CommunicationConstants::Identification::MASK_CHIP_TYPE_DEVICE) >> CommunicationConstants::Identification::SHIFT_CHIP_TYPE_DEVICE;
  switch(chipTypeDevice)
  {
    case CommunicationConstants::Identification::DEVICE_TOFFRAME611:
      device = Device_e::DEVICE_TOFFRAME611;
      break;
    case CommunicationConstants::Identification::DEVICE_TOFRANGE611:
      device = Device_e::DEVICE_TOFRANGE611;
      break;
    case CommunicationConstants::Identification::DEVICE_TOFCAM635:
      device = Device_e::DEVICE_TOFCAM635;
      break;
    default:
      device = Device_e::DEVICE_UNKNOWN;
      break;
  }


  //Mask out the version
  version = (identificationValue & CommunicationConstants::Identification::MASK_VERSION) >> CommunicationConstants::Identification::SHIFT_VERSION;

  return status;
}



/**
 * @brief Request the identification
 *
 * This identification helps to detect, which device is connected to the PC and if the bootloader
 * is active or the application. When the bootloader is active, the flage "isBootloader" is set.
 *
 * @param device Reference where the device is written to
 * @param isBootloader Reference to the bootloader flag
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getIdentification(Device_e &device, bool &isBootloader)
{
  unsigned int versionNotUsed;

  return getIdentification(device, isBootloader, versionNotUsed);
}

/**
 * @brief Request the chip information
 *
 * @param chipId Reference to the variable to write the chip id
 * @param waferId Reference to the variable to write the wafer id
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getChipInformation(uint16_t &chipId, uint16_t &waferId)
{
  ChipInformationHelper chipInformationHelper;

  //Temporary connect the signal to the helper  
  boost::signals2::connection cn = sigReceivedChipInformation.connect(boost::bind(&ChipInformationHelper::onReceivedChipInformation, &chipInformationHelper, _1, _2));

  //Send this command blocking
  ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_CHIP_INFORMATION, GET_CHIP_INFORMATION_SIZE);

  //The helper has the value
  chipId = chipInformationHelper.getChipId();
  waferId = chipInformationHelper.getWaferId();

  //Disconnect the signal from the helper
  cn.disconnect();

  return status;
}

/**
 * @brief Request the firmware rlease
 *
 * @param major Reference to write the major number
 * @param minor Reference to write the minor number
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getFirmwareRelease(unsigned int &major, unsigned int &minor)
{
  U32Helper releaseHelper;

  //Temporary connect the signal to the helper  
  boost::signals2::connection cn = sigReceivedFirmwareRelease.connect(boost::bind(&U32Helper::onReceivedData, &releaseHelper, _1));

  //Send this command blocking
  ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_FIRMWARE_RELEASE, GET_FIRMWARE_VERSION_SIZE);

  //Disconnect the signal from the helper
  cn.disconnect();

  major = releaseHelper.getValueMsb();
  minor = releaseHelper.getValueLsb();

  return status;
}

/**
 * @brief Request the temperature
 *
 * This function will be answered by the signal "receivedTemperature"
 */
void Communication::getTemperature()
{
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_TEMPERATURE, GET_TEMPERATURE_SIZE);
}

/**
 * @brief Get the production information
 *
 * This function returns the production information
 *
 * @param year Reference to the year
 * @param week Reference to the week
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getProductionInfo(unsigned int &year, unsigned int &week)
{
  ProductionInformationHelper productionInformationHelper;

  //Temporary connect the signal to the helper  
  boost::signals2::connection cn = sigReceivedProductionInfo.connect(boost::bind(&ProductionInformationHelper::onReceivedProductionInformation, &productionInformationHelper, _1, _2));

  //Send this command blocking
  ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_PRODUCTION_INFO, true);

  //The helper has the value
  year = productionInformationHelper.getYear();
  week = productionInformationHelper.getWeek();

  //Disconnect the signal from the helper
  cn.disconnect();

  return status;
}


/***************************************************************************
 * Acquisition commands --> blocking and non blocking
 ***************************************************************************/

void Communication::getDistanceGrayscale()
{
    int dataSize = 3 * (xMax_ - xMin_ + 1) * (yMax_ - yMin_ + 1) + 80 + CommunicationConstants::Command::SIZE_PAYLOAD; //16 bit distance + 8 bit grayscale
    sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_DISTANCE_GRAYSCALE, dataSize);
}

/**
 * @brief Request distance and amplitude
 *
 * This function will be answered by the signal "receivedDistanceAmplitude"
 */
void Communication::getDistanceAmplitude()
{
  int dataSize = 4 * (xMax_ - xMin_ + 1) * (yMax_ - yMin_ + 1) + 80  + CommunicationConstants::Command::SIZE_PAYLOAD; //16 bit distance + 16 bit amplitude
  
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_DISTANCE_AMPLITUDE, dataSize);
}

/**
 * @brief Request distance
 *
 * This function will be answered by the signal "receivedDistance"
 */
void Communication::getDistance()
{
  int dataSize = 2 * (xMax_ - xMin_ + 1) * (yMax_ - yMin_ + 1) + 80 + CommunicationConstants::Command::SIZE_PAYLOAD; //16 bit distance
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_DISTANCE, dataSize);
}

/**
 * @brief Request dcs distance amplitude
 *
 * This function will be answered by the signal "receivedDcsDistanceAmplitude"
 */
void Communication::getDcsDistanceAmplitude()
{
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_DCS_DISTANCE_AMPLITUDE, false);
}

/**
 * @brief Request dcs
 *
 * This function will be answered by the signal "receivedDcs"
 */
void Communication::getDcs()
{
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_DCS, false);
}

/**
 * @brief Request grayscale
 *
 * This function will be answered by the signal "receivedGrayscale"
 */
void Communication::getGrayscale()
{
  int dataSize = (xMax_ - xMin_ + 1) * (yMax_ - yMin_ + 1) + 80;
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_GRAYSCALE, dataSize);
}

void Communication::getLensCalibrationData()
{
  sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_LENS_CALIBRATION_DATA, 136); //(8+9)*8
}


/**
 * @brief Request the integration time
 *
 * Useful when automatic mode is enabled
 *
 * @param integrationTime Reference to the integration time
 */
ErrorNumber_e Communication::getIntegrationTime3d(unsigned int &integrationTime)
{
  U16Helper integrationTimeHelper;

  //Temporary connect the signal to the helper  
  boost::signals2::connection cn = sigReceivedIntegrationTime.connect(boost::bind(&U16Helper::onReceivedData, &integrationTimeHelper,  _1));

  //Send this command blocking
  ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_INTEGRATION_TIME_3D, GET_INTEGRATION_TIME_DIS_SIZE);

  //The helper has the value
  integrationTime = integrationTimeHelper.getValue();

  //Disconnect the signal from the helper
  cn.disconnect();

  return status;
}


/***************************************************************************
 * Setup commands --> blocking
 ***************************************************************************/
/**
 * @brief Set integration time 3D
 *
 * @param index Index of the integration time
 * @param integrationTime Integration time in us
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime)
{
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = CommunicationConstants::CommandList::COMMAND_SET_INTEGRATION_TIME_3D;

  //Add the index
  output[CommunicationConstants::IntegrationTime::INDEX_INDEX_3D] = index;

  //Add the time
  Util::setUint16LittleEndian(output, CommunicationConstants::IntegrationTime::INDEX_INTEGRATION_TIME_3D, integrationTime);

  //Send blocking
  return sendCommand(output, SET_INTEGRATION_TIME_DIS_SIZE);
}

/**
 * @brief Set integration time grayscale
 *
 * @param integrationTime Integration time in us
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setIntegrationTimeGrayscale(const unsigned int integrationTime)
{
  return sendCommandUint16(CommunicationConstants::CommandList::COMMAND_SET_INTEGRATION_TIME_GRAYSCALE, integrationTime, true);
}

/**
 * @brief Set modulation frequency
 *
 * @param modulationFrequency Selected modulation frequency
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setModulationFrequency(const ModulationFrequency_e modulationFrequency)
{
  ErrorNumber_e status = ErrorNumber_e::ERROR_NUMBER_INVALID_PARAMETER;

  switch(modulationFrequency)
  {
    case ModulationFrequency_e::MODULATION_FREQUENCY_10MHZ:
      status = sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODULATION_FREQUENCY, CommunicationConstants::ModulationFrequency::VALUE_10MHZ, false);
      break;
    case ModulationFrequency_e::MODULATION_FREQUENCY_20MHz:
      status = sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODULATION_FREQUENCY, CommunicationConstants::ModulationFrequency::VALUE_20MHZ, false);
      break;
    default:
      sendErrorSignal(status);
      break;
  }

  return status;
}

/**
 * @brief Set the filter settings
 *
 * Factor example:
 * 300 gives 300 x actualValue + 700 x lastValue
 *
 * @param threshold Threshold where the filter is cleared
 * @param factor Factor for the actual value
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setFilter(const unsigned int threshold, const unsigned int factor)
{
  return sendCommand2xUint16(CommunicationConstants::CommandList::COMMAND_SET_FILTER, threshold, factor, false);
}

ErrorNumber_e Communication::setDcsFilter(const bool enabled)
{
  uint8_t value = 0;

  //bool to 0/1
  if (enabled)
  {
    value = 1;
  }

  return sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_DCS_FILTER, value, false);
}

/**
 * @brief Enable/disable the gaussian filter
 *
 * @param enabled gaussian filter enabled or not
 */
ErrorNumber_e Communication::setGaussianFilter(const bool enabled)
{
  std::ignore = enabled;
  //uint8_t value = 0;

  //bool to 0/1
  //if (enabled)
  //{
  //  value = 1;
  //}

  //sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_GAUSSIAN_FILTER, value, CommunicationConstants::Type::DATA_ACK, false);
  return ErrorNumber_e::ERROR_NUMMBER_NO_ERROR;
}


/**
 * @brief Set the calibration mode
 *
 * In calibration mode the device disables the compensation and sends raw distance information. This is used during the calibration
 * procedure.
 *
 * calibration mode enabled --> compensation disabled
 * calibration mode disabled --> compensation enabled
 *
 * @param enabled Enable or disable calibration mode
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setCalibrationMode(const bool enabled)
{
  uint8_t value = 0;

  if (enabled)
  {
    value = 1;
  }

  return sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_CALIBRATE_DRNU, value, true);
}

/***************************************************************************
 * Update commands --> non blocking
 ***************************************************************************/
/**
 * @brief Send firmware update
 *
 * Call this function to do a firmware update on the target.
 *
 * @param updateFile QByteArray containing the update file data
 */
void Communication::updateFirmware(const std::vector<uint8_t> &updateFile)
{
  state = CommunicationState_e::COMMUNICATION_STATE_UPDATE;
  updateController.startUpdate(updateFile);
}



/**
 * @brief Set the ROI
 *
 * Set the ROI (region of interest) of the image
 *
 * @param xMin X coordinate top left
 * @param yMin Y coordinate top left
 * @param xMax X coordinate bottom right
 * @param yMax Y coordinate bottom right
 */
ErrorNumber_e Communication::setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax)
{

  xMin_ = xMin;
  yMin_ = yMin;
  xMax_ = xMax;
  yMax_ = yMax;

  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL + 4 * sizeof(uint16_t)];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = CommunicationConstants::CommandList::COMMAND_SET_ROI;

  //xMin
  Util::setUint16LittleEndian(output, CommunicationConstants::ROI::INDEX_ROI_X_MIN, xMin);

  //yMin
  Util::setUint16LittleEndian(output, CommunicationConstants::ROI::INDEX_ROI_Y_MIN, yMin);

  //xMax
  Util::setUint16LittleEndian(output, CommunicationConstants::ROI::INDEX_ROI_X_MAX, xMax);

  //yMax
  Util::setUint16LittleEndian(output, CommunicationConstants::ROI::INDEX_ROI_Y_MAX, yMax);

  return sendCommand(output, SET_ROI_SIZE);
}

ErrorNumber_e Communication::setOffset(const int offset){
  return sendCommandInt16(CommunicationConstants::CommandList::COMMAND_SET_OFFSET, offset, false);
}

ErrorNumber_e Communication::setMinimalAmplitude(const unsigned index, const unsigned int amplitude){
  uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

  memset(output, 0, sizeof(output));

  //Add the command
  output[CommunicationConstants::Command::INDEX_COMMAND] = CommunicationConstants::CommandList::COMMAND_SET_MINIMAL_AMPLITUDE;

  //Add the index
  output[CommunicationConstants::Amplitude::INDEX_INDEX] = index;

  //Add the amplitude
  Util::setUint16LittleEndian(output, CommunicationConstants::Amplitude::INDEX_AMPLITUDE, amplitude);

  return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}

ErrorNumber_e Communication::setBinning(const int binning){
  return sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_BINNING, binning, false);
}

ErrorNumber_e Communication::setFrameRate(const unsigned int FrameTime){
  return sendCommandUint16(CommunicationConstants::CommandList::COMMAND_SET_FRAME_RATE, FrameTime, false);
}




}//end namespace

/** @} */
