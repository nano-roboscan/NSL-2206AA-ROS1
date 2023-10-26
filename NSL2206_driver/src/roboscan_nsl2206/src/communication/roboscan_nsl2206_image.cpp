#include "roboscan_nsl2206_image.h"
#include "util.h"

using namespace std;

namespace ComLib
{

using namespace CommunicationConstants::Nsl2206Header;


void HeaderTemporalFilterSettings::readValues(const std::vector<uint8_t> &data, const unsigned int index)
{
  factor = Util::getUint16LittleEndian(data, index);
  threshold = Util::getUint16LittleEndian(data, index+2);
}

unsigned int HeaderTemporalFilterSettings::getFactor() const
{
  return factor;
}

unsigned int HeaderTemporalFilterSettings::getThreshold() const
{
  return threshold;
}



void HeaderModulationSettings::readValues(const std::vector<uint8_t> &data)
{
  frequency = data[INDEX_MODULATION_FREQUENCY];
  channel = data[INDEX_MODULATION_CHANNEL];
}

unsigned int HeaderModulationSettings::getFrequencyMhz() const
{
  switch(frequency)
  {
    case CommunicationConstants::ModulationFrequency::VALUE_10MHZ:
      return 10;
      break;
    case CommunicationConstants::ModulationFrequency::VALUE_20MHZ:
      return 20;
      break;
    default:
      break;
  }

  return 0;
}

string HeaderModulationSettings::getFrequencyString() const
{
  return std::to_string(getFrequencyMhz()) + "MHz";
}

unsigned int HeaderModulationSettings::getChannel() const
{
  return channel;
}

bool HeaderFlags::readFlag(const uint16_t value, const uint16_t mask)
{
  if (value & mask)
  {
    return true;
  }

  return false;
}

uint16_t HeaderFlags::getFlags() const{
  return (uint16_t)flags;
}

void HeaderFlags::readValues(const vector<uint8_t> &data)
{  
  flags = Util::getUint16LittleEndian(data, INDEX_FLAGS);
  compensated = readFlag(flags, CommunicationConstants::Flags::MASK_COMPENSATED);
  gaussianFilterEnabled = readFlag(flags, CommunicationConstants::Flags::MASK_GAUSSIAN_FILTER);
  averageFilterEnabled = readFlag(flags, CommunicationConstants::Flags::MASK_DCS_FILTER);
  autoIntegrationTimeEnabled = readFlag(flags, CommunicationConstants::Flags::MASK_AUTO_INTEGRATION_TIME);
  autoModulationEnabled = readFlag(flags, CommunicationConstants::Flags::MASK_AUTO_MODULATION);
}

bool HeaderFlags::getAutoModulationEnabled() const
{
  return autoModulationEnabled;
}

bool HeaderFlags::getAutoIntegrationTimeEnabled() const
{
  return autoIntegrationTimeEnabled;
}

bool HeaderFlags::getDcsFilterEnabled() const
{
  return averageFilterEnabled;
}

bool HeaderFlags::getGaussianFilterEnabled() const
{
  return gaussianFilterEnabled;
}

bool HeaderFlags::getCompensated() const
{
  return compensated;
}

Nsl2206Image::Nsl2206Image(const vector<uint8_t> &data)
{
  this->data = data;

  extractData(this->data);
}

Nsl2206Image::Nsl2206BeamType_e Nsl2206Image::extractBeamType(const unsigned int value)
{
  switch(value)
  {
    case Beam::BEAM_A:
      return Nsl2206BeamType_e::NSL2206_BEAM_TYPE_A;
      break;
    case Beam::BEAM_B:
      return Nsl2206BeamType_e::NSL2206_BEAM_TYPE_B;
      break;
    case Beam::BEAM_NONE:
      return Nsl2206BeamType_e::NSL2206_BEAM_TYPE_NONE;
      break;
    default:
      return Nsl2206BeamType_e::NSL2206_BEAM_TYPE_NONE;
      break;
  }

  return Nsl2206BeamType_e::NSL2206_BEAM_TYPE_NONE;
}

void Nsl2206Image::extractData(const vector<uint8_t> &data)
{
  this->frameCounter = data[INDEX_VERSION];
  this->frameCounter = Util::getInt16LittleEndian(data, INDEX_FRAME_COUNTER);
  this->timestamp = Util::getUint16LittleEndian(data, INDEX_TIMESTAMP);

  this->firmwareVersion = Util::getUint32LittleEndian(data, INDEX_FIRMWARE_VERSION);
  this->hardwareVersion = data[INDEX_HARDWARE_VERSION];
  this->chipId = Util::getUint16LittleEndian(data, INDEX_CHIP_ID);

  this->width = Util::getUint16LittleEndian(data, INDEX_WIDTH);
  this->height = Util::getUint16LittleEndian(data, INDEX_HEIGHT);
  this->originX = Util::getUint16LittleEndian(data, INDEX_ORIGIN_X);
  this->originY = Util::getUint16LittleEndian(data, INDEX_ORIGIN_Y);

  currentIntegrationTime3dWf = Util::getUint16LittleEndian(data, INDEX_CURRENT_INTEGRATION_TIME_3D_WF);
  currentIntegrationTime3dNf = Util::getUint16LittleEndian(data, INDEX_CURRENT_INTEGRATION_TIME_3D_NF);
  currentIntegrationTimeGrayscale = Util::getUint16LittleEndian(data, INDEX_CURRENT_INTEGRATION_TIME_GRAYSCALE);

  //The integration times can be extracted in a loop, because they come one after each other
  for (unsigned int i = 0; i < NUM_INTEGRATION_TIME_3D; i++)
  {
    integrationTime3d[i] = Util::getUint16LittleEndian(data, INDEX_INTEGRATION_TIME_0 + (i * sizeof(uint16_t)));
  }

  this->amplitudeLimit0 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_0);
  this->amplitudeLimit1 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_1);
  this->amplitudeLimit2 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_2);
  this->amplitudeLimit3 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_3);
  this->amplitudeLimit4 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_4);

  this->offset = Util::getInt16LittleEndian(data, INDEX_OFFSET);
  this->binning = data[INDEX_BINNING];

  this->temporalFilterDistance.readValues(data, INDEX_DISTANCE_TEMPORAL_FILTER_FACTOR);
  this->temporalFilterSingleValue.readValues(data, INDEX_SINGLE_VALUE_TEMPORAL_FILTER_FACTOR);

  this->modulation.readValues(data);

  this->flags.readValues(data);

  this->temperature = Util::getInt16LittleEndian(data, INDEX_TEMPERATURE);
  this->beamType = extractBeamType(data[INDEX_ILLUMINATION_BEAM]);

  this->singleValueDistance = Util::getUint16LittleEndian(data, INDEX_BEAM_B_DISTANCE);
  this->singleValueAmplitude = Util::getUint16LittleEndian(data, INDEX_BEAM_B_AMPLITUDE);

  this->value1 = Util::getUint16LittleEndian(data, INDEX_BEAM_B_AMPLITUDE+2);
  this->value2 = Util::getUint16LittleEndian(data, INDEX_BEAM_B_AMPLITUDE+4);

}

void Nsl2206Image::printInfo(std::vector<string> &stream)
{    
  /*stream <<"Frame Counter: " << frameCounter << "\r\n";
  stream << "Timestamp: " << timestamp << "\r\n";

  stream << "FW: " << getFirmwareRelease() << "\r\n";
  stream << "HW: " << hardwareVersion << "\r\n";
  stream << "ChipId: " << chipId << "\r\n";

  stream << "Width: " << width << "\r\n";
  stream << "Height: " << height << "\r\n";  
  stream << "OriginX: " << originX << "\r\n";
  stream << "OriginY: " << originY << "\r\n";

  stream << "currentIntegrationTime3d: " << currentIntegrationTime3d << "\r\n";
  stream << "currentIntegrationTimerGrayscale: " << currentIntegrationTimeGrayscale << "\r\n";

  for (unsigned int i = 0; i < NUM_INTEGRATION_TIME_3D; i++)
  {
    stream << "Integration time [" << i <<"] = " << integrationTime3d[i] << "\r\n";
  }

  stream << "AmplitudeLimit: " << amplitudeLimit << "\r\n";
  stream << "Offset: " << offset << "\r\n";
  stream << "Binning: " << binning << "\r\n";

  stream << "TemporalFilterDistance: " << "factor: " << temporalFilterDistance.getFactor() << ", threshold: " << temporalFilterDistance.getThreshold() << "\r\n";
  stream << "TemporalFilterSingleValue: " << "factor: " << temporalFilterSingleValue.getFactor() << ", threshold: " << temporalFilterSingleValue.getThreshold() << "\r\n";

  stream << "Modulation: " << "frequency : " << modulation.getFrequencyString() << ", channel: " << modulation.getChannel() << "\r\n";

  stream << "Compensated: " << flags.getCompensated() << "\r\n";
  stream << "Gaussian Filter: " << flags.getGaussianFilterEnabled() << "\r\n";
  stream << "DCS Filter: " << flags.getDcsFilterEnabled() << "\r\n";
  stream << "AutoIntegrationTime: " << flags.getAutoIntegrationTimeEnabled() << "\r\n";
  stream << "AutoModulationChannel: " << flags.getAutoModulationEnabled() << "\r\n";

  stream << "Temperature: " << getTemperatureC() << "\r\n";
  stream << "Beam: " << beamType << "\r\n";

  stream << "singleValueDistance: " << getSingleValueDistanceMm() << "\r\n";
  stream << "singleValueAmplitude: " << singleValueAmplitude << "\r\n";*/
}

unsigned int Nsl2206Image::getHeaderVersion() const
{
  return headerVersion;
}

unsigned int Nsl2206Image::getFrameCounter() const
{
  return frameCounter;
}

unsigned int Nsl2206Image::getTimestamp() const
{
  return timestamp;
}


unsigned int Nsl2206Image::getHardwareVersion() const
{
  return hardwareVersion;
}

unsigned int Nsl2206Image::getChipID() const
{
  return chipId;
}

unsigned int Nsl2206Image::getNumPixel() const
{
  return width * height;
}

unsigned int Nsl2206Image::getWidth() const
{
  return width;
}

unsigned int Nsl2206Image::getHeight() const
{
  return height;
}

unsigned int Nsl2206Image::getOriginX() const
{
  return originX;

}

unsigned int Nsl2206Image::getOriginY() const
{
  return originY;
}

unsigned int Nsl2206Image::getCurrentIntegrationTime3DWF() const
{
  return currentIntegrationTime3dWf;
}

unsigned int Nsl2206Image::getCurrentIntegrationTime3DNF() const
{
  return currentIntegrationTime3dNf;
}

unsigned int Nsl2206Image::getCurrentIntegrationTimeGrayscale() const
{
  return currentIntegrationTimeGrayscale;
}

unsigned int Nsl2206Image::getIntegrationTimeGrayscale() const
{
  return integrationTimeGrayscale;
}

unsigned int Nsl2206Image::getIntegrationTime3d(const unsigned int index) const
{
  if (index >= NUM_INTEGRATION_TIME_3D)
  {
    return 0;
  }

  return integrationTime3d[index];
}

unsigned int Nsl2206Image::getAmplitudeLimit(int index) const
{
  switch(index){
    case 0: return amplitudeLimit0;
    case 1: return amplitudeLimit1;
    case 2: return amplitudeLimit2;
    case 3: return amplitudeLimit3;
    case 4: return amplitudeLimit4;
  }
  return amplitudeLimit0;
}

int Nsl2206Image::getOffset() const
{
  return offset;
}

unsigned int Nsl2206Image::getBinningType() const
{
  return binning;
}

HeaderModulationSettings Nsl2206Image::getModulation() const
{
  return modulation;
}


HeaderTemporalFilterSettings Nsl2206Image::getTemporalFilterDistance() const
{
  return temporalFilterDistance;
}

HeaderTemporalFilterSettings Nsl2206Image::getTemporalFilterSingleValue() const
{
  return temporalFilterSingleValue;
}

HeaderFlags Nsl2206Image::getHeaderFlags() const
{
  return flags;
}


int Nsl2206Image::getTemperature() const
{
  return temperature;
}

string Nsl2206Image::getTemperatureC() const
{
  return std::to_string(static_cast<double>(getTemperature()) / 100.0) + "°C";
}

unsigned int Nsl2206Image::getSingleValueDistance() const
{
  return singleValueDistance;
}

string Nsl2206Image::getSingleValueDistanceMm() const
{
  return convertToMmNoErrorCode(singleValueDistance);
}

unsigned int Nsl2206Image::getSingleValueAmplitude() const
{
  return singleValueAmplitude;
}

unsigned int Nsl2206Image::getSingleValue(int index) const{

  if(index == 0) return value1;
  else return value2;
}

bool Nsl2206Image::getPixelIsSaturated(const unsigned int index) const
{
  if (getDistanceOfPixel(index) == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
  {
    return true;
  }

  return false;
}

bool Nsl2206Image::getPixelIsAdcOverflow(const unsigned int index) const
{
  if (getDistanceOfPixel(index) == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
  {
    return true;
  }

  return false;
}

bool Nsl2206Image::getPixelIsLowAmplitude(const unsigned int index) const
{
  if (getDistanceOfPixel(index) == CommunicationConstants::PixelNsl2206::VALUE_LOW_AMPLITUDE)
  {
    return true;
  }

 return false;
}

unsigned int Nsl2206Image::getAmplitudeOfPixel(const unsigned int index __attribute__((unused))) const
{
  return 0;
}

unsigned int Nsl2206Image::getDistanceOfPixel(const unsigned int index __attribute__((unused))) const
{
  return 0;
}

string Nsl2206Image::getDistanceOfPixelMm(const unsigned int index) const
{
  return convertToMm(getDistanceOfPixel(index));
}

string Nsl2206Image::convertToMm(const unsigned int distance) const
{
  if (distance == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
  {
    return "AdcOverflow";
  }
  if (distance == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
  {
    return "Saturation";
  }
  if (distance == CommunicationConstants::PixelNsl2206::VALUE_LOW_AMPLITUDE)
  {
    return "LowAmplitude";
  }

  string str;
  return str;     //QString::number(distance) + "mm";
}

string Nsl2206Image::convertToMmNoErrorCode(const unsigned int distance) const
{
  if (distance == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
  {
    return "";
  }
  if (distance == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
  {
    return "";
  }
  if (distance == CommunicationConstants::PixelNsl2206::VALUE_LOW_AMPLITUDE)
  {
    return "";
  }

  //Cut off distance lower than 1m
  if (distance < 1000)
  {
    return string("");
  }
  else
  {    
    return std::to_string(distance) + "mm";
  }
}

unsigned int Nsl2206Image::getGrayscaleOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::Nsl2206Header::SIZE;

  //Make a pointer to the distance
  uint8_t *grayscale =  (uint8_t *)(&data.data()[dataOffset]);

  return grayscale[index];
}

Nsl2206Image::Nsl2206BeamType_e Nsl2206Image::getBeamType() const
{
  return beamType;
}

uint16_t Nsl2206Image::getFirmwareVersion() const
{
  return firmwareVersion;
}

double Nsl2206Image::getFirmwareRelease() const
{
  double release = static_cast<double>((firmwareVersion >> 16)) + (static_cast<double>(firmwareVersion & 0xFFFF)) / 100.0;

  return release;
}

bool Nsl2206Image::getPixelIsInvalid(const unsigned int value)
{
  if (value < CommunicationConstants::PixelNsl2206::LIMIT_VALID_PIXEL)
  {
    return false;
  }

  return true;
}

}
