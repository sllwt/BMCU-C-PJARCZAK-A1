#pragma once
#include <stdint.h>

#ifndef BMCU_AMS2_PRO_IDENTITY
#define BMCU_AMS2_PRO_IDENTITY 1
#endif

#ifndef BMCU_AMS2_FULL_IDENTITY
#define BMCU_AMS2_FULL_IDENTITY 0
#endif

enum class bambubus_package_type
{
    error = -1,
    none = 0,
    filament_motion_short,
    filament_motion_long,
    online_detect,
    REQx6,
    NFC_detect,
    set_filament_info,
    MC_online,
    read_filament_info,
    set_filament_info_type2,
    version,
    serial_number,
    authentication_cert,
    authentication_challenge,
    heartbeat,
    ETC,

    __BambuBus_package_packge_type_size
};

void bambubus_init(void);
void bambubus_heartbeat_seen_fast(void);
extern bambubus_package_type bambubus_run();
