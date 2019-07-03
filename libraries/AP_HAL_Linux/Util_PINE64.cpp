#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_URUS || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI

#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#include "Util_PINE64.h"
#include "Util.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

UtilPINE64::UtilPINE64()
{
    _check_pine64_version();
}

int UtilPINE64::_check_pine64_version()
{
    _pine64_version = 1;
    return _pine64_version;
}

int UtilPINE64::get_pine64_version() const
{
    return _pine64_version;
}

/*
*author: ZengZhiwei
*date:2017-3-19
*function:return safety state,the _safety_state
*value update on LinuxRCInput_Raspilot
*/
enum AP_HAL::Util::safety_state UtilPINE64::safety_switch_state(void)
{
    return _safety_state;
}

#endif
