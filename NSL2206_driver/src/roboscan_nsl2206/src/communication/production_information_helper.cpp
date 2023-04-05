/**
 * nanosystems
 */

#include "production_information_helper.h"

namespace ComLib
{

ProductionInformationHelper::ProductionInformationHelper()
{
    year = 0;
    week = 0;
}

unsigned int ProductionInformationHelper::getYear()
{
    return year;
}

unsigned int ProductionInformationHelper::getWeek()
{
    return week;
}

void ProductionInformationHelper::onReceivedProductionInformation(const uint8_t year, const uint8_t week)
{
    this->year = year;
    this->week = week;
}

}
