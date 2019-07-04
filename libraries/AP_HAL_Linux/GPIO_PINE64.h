#ifndef    __AP_HAL_LINUX_GPIO_PINE64_H
#define    __AP_HAL_LINUX_GPIO_PINE64_H

#include "AP_HAL_Linux.h"
#include <stdint.h>

#ifndef LOW
    #define LOW                 0
#endif
#ifndef HIGH
    #define HIGH                1
#endif

#define PINE64_BLOCK_SIZE (4*1024)
#define PINE64_MAP_SIZE        (4096*2)
#define PINE64_MAP_MASK        (PINE64_MAP_SIZE - 1)


#define SUNXI_GPIO_BASE     0x01C20000
#define SUNXI_GPIO_OFFSET   0x800
//register offset
#define SUNXI_BASE_OFFSET   0x24
#define SUNXI_CFG0_OFFSET   0x00
#define SUNXI_CFG1_OFFSET   0x04
#define SUNXI_CFG2_OFFSET   0x08
#define SUNXI_CFG3_OFFSET   0x0C
#define SUNXI_DAT_OFFSET    0x10

/*
#define PINE_A64_GPIO_PB2       102      //start_led        //复用UART2_RTS 

#define PINE_A64_GPIO_PC4       204     //A64_L3GD20H_CS
#define PINE_A64_GPIO_PC5       205     //A64_MS5611_1_CS
#define PINE_A64_GPIO_PC6       206     //A64_MS5611_2_CS
#define PINE_A64_GPIO_PC7       207     //A64_LSM303D_CS
#define PINE_A64_GPIO_PC8       208     //A64_ADXRS453_X_CS
#define PINE_A64_GPIO_PC9       209     //A64_ADXRS453_Y_CS
#define PINE_A64_GPIO_PC10      210     //A64_ADXRS453_Z_CS
#define PINE_A64_GPIO_PC11      211     //A64_MPU9250_CS
#define PINE_A64_GPIO_PC12      212     //A64_ADXL357_CS
#define PINE_A64_GPIO_PC13      213     //L3GD20H_DRDY
#define PINE_A64_GPIO_PC14      214     //LSM303D_INT1  ACCEL_DRDY
#define PINE_A64_GPIO_PC15      215     //LSM303D_INT2  MAG_DRDY
#define PINE_A64_GPIO_PC16      216     //MPU9250_DRDY

#define PINE_A64_GPIO_PD6       306     //F407_SPI_CS       //复用LCD-D10
#define PINE_A64_GPIO_PD7       307                         //复用LCD-D11

#define PINE_A64_GPIO_PH5       705     //LED_!RESET          //复用UART3-RX
#define PINE_A64_GPIO_PH6       706     //IO-RPI-SPI_NSS    //复用UART3-RTS
#define PINE_A64_GPIO_PH9       709     //ADXL357_DRDY
*/

#define PINE_A64_GPIO_PB2       100      //start_led        //复用UART2_RTS 

#define PINE_A64_GPIO_PC4       101     //A64_L3GD20H_CS
#define PINE_A64_GPIO_PC5       102     //A64_MS5611_1_CS
#define PINE_A64_GPIO_PC6       103     //A64_MS5611_2_CS
#define PINE_A64_GPIO_PC7       104     //A64_LSM303D_CS
#define PINE_A64_GPIO_PC8       105     //A64_ADXRS453_X_CS
#define PINE_A64_GPIO_PC9       106     //A64_ADXRS453_Y_CS
#define PINE_A64_GPIO_PC10      107     //A64_ADXRS453_Z_CS
#define PINE_A64_GPIO_PC11      108     //A64_MPU9250_CS
#define PINE_A64_GPIO_PC12      109     //A64_ADXL357_CS
#define PINE_A64_GPIO_PC13      110     //L3GD20H_DRDY
#define PINE_A64_GPIO_PC14      111     //LSM303D_INT1  ACCEL_DRDY
#define PINE_A64_GPIO_PC15      112     //LSM303D_INT2  MAG_DRDY
#define PINE_A64_GPIO_PC16      113     //MPU9250_DRDY

#define PINE_A64_GPIO_PD6       114     //F407_SPI_CS       //复用LCD-D10
#define PINE_A64_GPIO_PD7       115                         //复用LCD-D11

#define PINE_A64_GPIO_PH5       116     //LED_!RESET          //复用UART3-RX
#define PINE_A64_GPIO_PH6       117     //IO-RPI-SPI_NSS    //复用UART3-RTS
#define PINE_A64_GPIO_PH9       118     //ADXL357_DRDY

#define GPIO_INDEX_HEAD 100
static const uint32_t gpio_index[] = {102, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 306, 307, 705, 706, 709};


//计算CFGn
#define CFG_INDEX(X)  ((X)>15)?(((X)>23)?3:2):(((X)>7)?1:0)


namespace Linux {

class GPIO_PINE64 : public AP_HAL::GPIO {

public:
    GPIO_PINE64();

    void    init();
    void    pinMode(uint8_t pin, uint8_t mode);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);


private:
    void set_gpio_direction(uint8_t pin, uint8_t direct);
    void set_gpio_value(uint8_t pin, uint8_t value);
    uint8_t get_gpio_value(uint8_t pin);
};

}

#endif // __AP_HAL_LINUX_GPIO_PINE64_H
