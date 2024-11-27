#include "Settings.h"

bool validate_settings(const Settings_union& settings) {
    // clang-format off
    if (settings.data.step_current > SettingLimits::MAX_CURRENT || 
        settings.data.sleep_current > SettingLimits::MAX_CURRENT || 
        settings.data.microstep_res > SettingLimits::MAX_MICROSTEP_RES) {
        return false;
    }
    // clang-format on
    return true;
}