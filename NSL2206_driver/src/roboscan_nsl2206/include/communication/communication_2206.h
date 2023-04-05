/**
 * nanosystems
 */

#ifndef COMMUNICATION_2206_H
#define COMMUNICATION_2206_H

#include "communication.h"
#include "roboscan_nsl2206_image.h"
#include "roboscan_nsl2206_distance_image.h"
#include "roboscan_nsl2206_distance_amplitude_image.h"
#include "roboscan_nsl2206_distance_grayscale_image.h"
#include "roboscan_nsl2206_grayscale_image.h"


namespace ComLib
{
//! Communication Implementation for NSL2206
/*!
 * This class implements the specific functionality for NSL2206 device.
 */
class Communication2206: public Communication
{
public:
    Communication2206();
    ErrorNumber_e setMode(const int mode);

    //signals
    boost::signals2::signal<void (std::shared_ptr<Nsl2206Image>)> sigReceivedDistance;
    boost::signals2::signal<void (std::shared_ptr<Nsl2206Image>)> sigReceivedGrayscale;
    boost::signals2::signal<void (std::shared_ptr<Nsl2206Image>)> sigReceivedDistanceGrayscale;
    boost::signals2::signal<void (std::shared_ptr<Nsl2206Image>)> sigReceivedDistanceAmplitude;
    boost::signals2::signal<void (std::vector<uint8_t>)> sigReceivedLensCalibrationData;

private:

    unsigned int getBaudRate();    
    bool setupDevice(const Device_e device);
    void processDistanceAmplitude(const std::vector<uint8_t> &array);
    void processDistance(const std::vector<uint8_t> &array);
    void processGrayscale(const std::vector<uint8_t> &array);
    void processDistanceGrayscale(const std::vector<uint8_t> &array);
    void processLensCalibrationData(const std::vector<uint8_t> &array);
    //Mode_e actualMode;   ///<Stores the actual selected mode
};
}

#endif // COMMUNICATION_2206_H

/** @} */
