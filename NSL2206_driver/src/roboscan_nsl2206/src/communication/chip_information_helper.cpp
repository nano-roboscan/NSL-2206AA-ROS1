#include "chip_information_helper.h"

namespace ComLib
{

ChipInformationHelper::ChipInformationHelper()
{
    waferId = 0;
    chipId = 0;
}

uint16_t ChipInformationHelper::getWaferId()
{
    return waferId;
}

uint16_t ChipInformationHelper::getChipId()
{
    return chipId;
}

void ChipInformationHelper::onReceivedChipInformation(const uint16_t chipId, const uint16_t waferId)
{
    this->waferId = waferId;
    this->chipId = chipId;
}

}
