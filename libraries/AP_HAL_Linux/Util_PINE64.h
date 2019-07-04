#pragma once

#include "Util.h"

namespace Linux {

class UtilPINE64 : public Util {
public:
    UtilPINE64();

    static UtilPINE64 *from(AP_HAL::Util *util) {
        return static_cast<UtilPINE64*>(util);
    }

    /* return the PINE64 version */
    int get_pine64_version() const;

    enum AP_HAL::Util::safety_state safety_switch_state(void);	//add author ZengZhiwei

    AP_HAL::Util::safety_state _safety_state;		//add author ZengZhiwei

protected:
    // Called in the constructor once
    int _check_pine64_version();

private:
    int _pine64_version = 0;
};

}

