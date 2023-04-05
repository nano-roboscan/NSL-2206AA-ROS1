#ifndef PRODUCTIONINFOHELPER_H
#define PRODUCTIONINFOHELPER_H

#include <stdint.h>


namespace ComLib
{

//! Helper class for Production Information
/*!
 * This class helps, when the Production Information is being read blocking. It receives the signals and
 * stores the values. Later they can be read using the getter functions.
 */
class ProductionInformationHelper
{

public:
    ProductionInformationHelper();
    unsigned int getYear();
    unsigned int getWeek();

    //public slots
    void onReceivedProductionInformation(const uint8_t year, const uint8_t week);

private:
    unsigned int year;
    unsigned int week;
};

}

#endif // PRODUCTIONINFOHELPER_H

/** @} */
