#ifndef NSL2206HEADER_H
#define NSL2206HEADER_H

#include "communication_constants.h"
#include <vector>
#include <string>


namespace ComLib
{

class HeaderTemporalFilterSettings
{
  public:    
    void readValues(const std::vector<uint8_t> &data, const unsigned int index);
    unsigned int getFactor() const;
    unsigned int getThreshold() const;

  private:
    unsigned int factor;
    unsigned int threshold;
};

class HeaderModulationSettings
{
  public:
    void readValues(const std::vector<uint8_t> &data);
    unsigned int getFrequencyMhz() const;
    std::string getFrequencyString() const;
    unsigned int getChannel() const;

  private:
    unsigned int frequency;
    unsigned int channel;
};

class HeaderFlags
{
  public:
    uint16_t getFlags() const;
    void readValues(const std::vector<uint8_t> &data);
    bool getAutoModulationEnabled() const;
    bool getAutoIntegrationTimeEnabled() const;
    bool getDcsFilterEnabled() const;
    bool getGaussianFilterEnabled() const;
    bool getCompensated() const;

  private:
    bool readFlag(const uint16_t value, const uint16_t mask);

    uint32_t flags;
    bool autoModulationEnabled;
    bool autoIntegrationTimeEnabled;
    bool averageFilterEnabled;
    bool gaussianFilterEnabled;
    bool compensated;
};





class Nsl2206Image
{
  public:
    enum Nsl2206ImageType_e
    {
      NSL2206_IMAGE_DISTANCE, NSL2206_IMAGE_DISTANCE_AMPLITUDE, NSL2206_IMAGE_GRAYSCALE, NSL2206_IMAGE_DISTANCE_GRAYSCALE
    };

    enum Nsl2206BeamType_e
    {
      NSL2206_BEAM_TYPE_NONE, NSL2206_BEAM_TYPE_A, NSL2206_BEAM_TYPE_B
    };    

    Nsl2206Image(const std::vector<uint8_t> &data);
    virtual Nsl2206ImageType_e getType() = 0;

    void printInfo(std::vector<std::string> &stream);
    unsigned int getNumPixel() const;

    unsigned int getHeaderVersion() const;
    unsigned int getFrameCounter() const;
    unsigned int getTimestamp() const;
    unsigned int getHardwareVersion() const;
    unsigned int getChipID() const;
    unsigned int getWidth() const;
    unsigned int getHeight() const;
    unsigned int getOriginX() const;
    unsigned int getOriginY() const;        
    unsigned int getCurrentIntegrationTime3DWF() const;
    unsigned int getCurrentIntegrationTime3DNF() const;
    unsigned int getCurrentIntegrationTimeGrayscale() const;
    unsigned int getIntegrationTimeGrayscale() const;
    unsigned int getIntegrationTime3d(const unsigned int index) const;
    unsigned int getAmplitudeLimit(int index) const;
    int getOffset() const;
    unsigned int getBinningType() const;
    HeaderModulationSettings getModulation() const;
    HeaderTemporalFilterSettings getTemporalFilterDistance() const;
    HeaderTemporalFilterSettings getTemporalFilterSingleValue() const;
    HeaderFlags getHeaderFlags() const;
    int getTemperature() const;
    std::string getTemperatureC() const;
    unsigned int getSingleValueDistance() const;
    std::string getSingleValueDistanceMm() const;
    unsigned int getSingleValueAmplitude() const;
    unsigned int getSingleValue(int index) const;

    virtual unsigned int getGrayscaleOfPixel(const unsigned int index) const;
    virtual unsigned int getDistanceOfPixel(const unsigned int index) const;    
    virtual std::string getDistanceOfPixelMm(const unsigned int index) const;
    virtual unsigned int getAmplitudeOfPixel(const unsigned int index) const;
    virtual bool getPixelIsSaturated(const unsigned int index) const;
    virtual bool getPixelIsAdcOverflow(const unsigned int index) const;
    virtual bool getPixelIsLowAmplitude(const unsigned int index) const;
    std::string convertToMm(const unsigned int distance) const;
    std::string convertToMmNoErrorCode(const unsigned int distance) const;
    Nsl2206BeamType_e getBeamType() const;
    double getFirmwareRelease() const;
    uint16_t getFirmwareVersion() const;
    static bool getPixelIsInvalid(const unsigned int value);

  protected:
    std::vector<uint8_t> data;

  private:
    void extractData(const std::vector<uint8_t> &data);
    Nsl2206BeamType_e extractBeamType(const unsigned int value);

    unsigned int headerVersion;
    unsigned int frameCounter;
    unsigned int timestamp;

    unsigned int firmwareVersion;
    unsigned int hardwareVersion;
    unsigned int chipId;

    unsigned int width;
    unsigned int height;

    unsigned int originX;
    unsigned int originY;

    unsigned int integrationTime3d[CommunicationConstants::Nsl2206Header::NUM_INTEGRATION_TIME_3D];
    unsigned int currentIntegrationTime3dNf;
    unsigned int currentIntegrationTime3dWf;
    unsigned int currentIntegrationTimeGrayscale;
    unsigned int integrationTimeGrayscale;

    unsigned int amplitudeLimit0;
    unsigned int amplitudeLimit1;
    unsigned int amplitudeLimit2;
    unsigned int amplitudeLimit3;
    unsigned int amplitudeLimit4;

    int offset;
    unsigned int binning;

    HeaderTemporalFilterSettings temporalFilterDistance;
    HeaderTemporalFilterSettings temporalFilterSingleValue;

    HeaderModulationSettings modulation;

    HeaderFlags flags;

    int temperature;
    unsigned int singleValueDistance;
    unsigned int singleValueAmplitude;

    unsigned int value1;
    unsigned int value2;


    Nsl2206BeamType_e beamType;
};

}

#endif // NSL2206HEADER_H
