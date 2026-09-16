#include "aja.h"

#include <common/log.h>

#include <ajantv2/includes/ntv2card.h>
#include <ajantv2/includes/ntv2devicescanner.h>
#include "consumer/aja_consumer.h"

namespace caspar { namespace aja {

std::wstring get_version()
{
    return L"NTV2 SDK 18.0.0";
}

void init(const core::module_dependencies& dependencies)
{
    CASPAR_LOG(info) << L"AJA module initializing";

    dependencies.consumer_registry->register_consumer_factory(
        L"AJA Consumer",
        create_consumer);

    dependencies.consumer_registry->register_preconfigured_consumer_factory(
        L"aja",
        create_preconfigured_consumer);

    try {
        CNTV2DeviceScanner scanner(true);
        const size_t numDevices = CNTV2DeviceScanner::GetNumDevices();

        if (numDevices == 0) {
            CASPAR_LOG(info) << L"No AJA NTV2 devices found";
            return;
        }

        CASPAR_LOG(info) << L"AJA NTV2 devices found: " << numDevices;

        for (UWord deviceIndex = 0; deviceIndex < numDevices; ++deviceIndex) {
            CNTV2Card device;

            if (!CNTV2DeviceScanner::GetDeviceAtIndex(deviceIndex, device)) {
                CASPAR_LOG(warning)
                    << L"Unable to open AJA device index "
                    << deviceIndex;
                continue;
            }

            const std::string displayName = device.GetDisplayName();

            std::string serialNumber;
            if (!device.GetSerialNumberString(serialNumber))
                serialNumber = "unknown";

            CASPAR_LOG(info)
                << L" - AJA device [" << deviceIndex << L"]: "
                << std::wstring(displayName.begin(), displayName.end())
                << L", serial "
                << std::wstring(serialNumber.begin(), serialNumber.end());
        }
    }
    catch (const std::exception& e) {
        const std::string what = e.what();

        CASPAR_LOG(error)
            << L"AJA module initialization failed: "
            << std::wstring(what.begin(), what.end());
    }
    catch (...) {
        CASPAR_LOG(error)
            << L"AJA module initialization failed with unknown exception";
    }
}

}} // namespace caspar::aja
