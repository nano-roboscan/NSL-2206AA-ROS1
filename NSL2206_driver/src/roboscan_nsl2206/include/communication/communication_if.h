/**
 * nanosystems
 */

#ifndef COMMUNICATION_IF_H
#define COMMUNICATION_IF_H

#include <list>
#include <string>
#include <vector>

#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace ComLib
{

///Enum to define the modes
enum Nsl2206Mode_e
{
  MODE_BEAM_A = 0,                                        ///<Normal operation with illumination beam A
  MODE_BEAM_B_MANUAL = 1,                                 ///<Normal operation with illumination beam B (all settings by user, same as)
  MODE_BEAM_B_RESULT = 2,                                 ///<Beam B with calibrated ROI, only one distance as result
  MODE_BEAM_B_RESULT_DATA = 3,                            ///<Beam B with calibrated ROI, one distance and the pixels as result
  MODE_BEAM_AB_RESULT = 4,                                ///<Beam A and B operating with calibrated ROI and only one distance as result
  MODE_BEAM_AB_AUTO_RESULT = 5,                           ///<Beam A and B with automatic selection
  MODE_BEAM_AB_INTERLEAVED_DATA = 6                       ///<Beam A and B interleaved output
};

///Enum to define the modulation frequencies
enum ModulationFrequency_e
{
  MODULATION_FREQUENCY_10MHZ = 0,
  MODULATION_FREQUENCY_20MHz = 1
};

///Enum to define the modes
enum Mode_e
{
  MODE_TIM = 0,
  MODE_ULN = 1,
  MODE_UFS = 2
};

//These error numbers are given in the signal "error"
enum ErrorNumber_e
{
  ERROR_NUMMBER_NO_ERROR = 0,
  ERROR_NUMBER_TIMEOUT = 32768,
  ERROR_NUMBER_NOT_ACKNOWLEDGE = 32769,
  ERROR_NUMBER_INVALID_PARAMETER = 32770,
  ERROR_NUMBER_SERIAL_PORT_ERROR = 32771,
  ERROR_NUMBER_INVALID_DATA = 32772
};

//Device list. The device is read with the command "getIdentification"
enum Device_e
{
  DEVICE_UNKNOWN = 0,
  DEVICE_TOFRANGE611 = 1,
  DEVICE_TOFFRAME611 = 2,
  DEVICE_TOFCAM635 = 3
};

//! Communication Interface
/*!
 * This abstract class is the interface for the communication. All access must be done using this
 * interface.
 */
/* --Remarks--
 *
 * Bit width
 * Where the number of bits is important, the stdint types are used. If the number of bits is not important, the
 * standard types are used, so they could be 32Bit or 64Bit without affect. In any case they are as big as in the
 * sensor module or bigger. Like this the compiler on the PC does not need to do unneeded masking (for example
 * for a uint32_t on a 64Bit machine).
 *
 * Blocking/NonBlocking
 * Settings commands are done blocking. This makes it easier to send a list of commands.
 * Acquisition commands are done non blocking
 */
class Communication_IF
{

  public:                                                                                                           //Command type
    Communication_IF(){};    
    virtual ~Communication_IF(){};

    virtual std::list<std::string> availableDevices() = 0;
    virtual bool open() = 0;
    virtual void close() = 0;
	
	//General commands
    virtual ErrorNumber_e setPower(const bool enabled) = 0;                                                         //blocking command
	
	//Information commands
    virtual ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader) = 0;                              //blocking command
    virtual ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader, unsigned int &version) = 0;       //blocking command
    virtual ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId) = 0;                              //blocking command
    virtual ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor) = 0;                         //blocking command
    virtual std::string getDeviceName() = 0;                                                                        //blocking command
    virtual ErrorNumber_e getProductionInfo(unsigned int &year, unsigned int &week) = 0;                            //blocking command
    	
	//Setup commands
    virtual ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime) = 0;   //blocking command
    virtual ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime) = 0;                      //blocking command
    virtual ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency) = 0;              //blocking command
    virtual ErrorNumber_e setMode(const int mode) = 0;                                                              //blocking command
    virtual ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor) = 0;                   //blocking command
    virtual ErrorNumber_e setDcsFilter(const bool enabled) = 0;                                    // --> no signal in case of success
    virtual ErrorNumber_e setGaussianFilter(const bool enabled) = 0;                               // --> no signal in case of success

    virtual ErrorNumber_e setCalibrationMode(const bool enabled) = 0;                                               //blocking command

    virtual ErrorNumber_e setOffset(const int offset) = 0;                                                                                 // --> no signal in case of success
    virtual ErrorNumber_e setMinimalAmplitude(const unsigned int index, const unsigned int amplitude) = 0;                                 // --> no signal in case of success
    virtual ErrorNumber_e setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax) = 0;  // --> no signal in case of success                                                                                      // --> no signal in case of success
    virtual ErrorNumber_e setBinning(const int binning) = 0;                                                                               // --> no signal in case of success
    virtual ErrorNumber_e setFrameRate(const unsigned int FrameTime) = 0;                                                                  // --> no signal in case of success
	
  //Update commands
    virtual void updateFirmware(const std::vector<uint8_t> &updateFile) = 0;                                                  // --> firmwareUpdateProgress

  //public slots
    //Information commands
    virtual void getTemperature() = 0;                                                                              // --> receivedTemperature

    //Acquisition commands
    virtual void getGrayscale() = 0;
    virtual void getDistanceGrayscale() = 0;                                                                        // --> receivedDistanceGrayscale
    virtual void getDistanceAmplitude() = 0;                                                                        // --> receivedDistanceAmplitude
    virtual void getDistance() = 0;                                                                                 // --> receivedDistance
    virtual void getDcsDistanceAmplitude() = 0;                                                                     // --> receivedDcsDistanceAmplitude
    virtual void getDcs() = 0;                                                                                      // --> receivedDcs
    virtual void getLensCalibrationData() = 0;                                                                      // --> receivedLensCalibrationData
    virtual ErrorNumber_e getIntegrationTime3d(unsigned int &integrationTime) = 0;                                  //blocking command

  //signals
    //boost::signals2::signal<void (const uint32_t *distance, const unsigned int numPixel)> sigReceivedDistance;


    //boost::signals2::signal<void (const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel)> sigReceivedDistanceAmplitude;
    boost::signals2::signal<void (const uint16_t *dcs, const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel)> sigReceivedDcsDistanceAmplitude16; //DCS with 16Bits
    boost::signals2::signal<void (const uint32_t *dcs, const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel)> sigReceivedDcsDistanceAmplitude32; //DCS with 32Bits
    boost::signals2::signal<void (const uint16_t *dcs, const unsigned int numPixel)> sigReceivedDcs16;   //DCS with 16Bits
    boost::signals2::signal<void (const uint32_t *dcs, const unsigned int numPixel)> sigReceivedDcs32;  //DCS with 32Bits
    //boost::signals2::signal<void (const uint16_t *grayscale, const unsigned int numPixel)> sigReceivedGrayscale;
    boost::signals2::signal<void((const int16_t temperature))> sigReceivedTemperature;
    boost::signals2::signal<void (const unsigned int)> sigFirmwareUpdateProgress;    
    boost::signals2::signal<void (const ErrorNumber_e errorNumber)> sigError;
};
} //end namespace ComLib

#endif // COMMUNICATION_IF_H

/** @} */
