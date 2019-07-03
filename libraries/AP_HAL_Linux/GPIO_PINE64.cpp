/**
 * @author  ZengZhiwei
 * @version 2018/01/10
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include "GPIO.h"   //包含了GPIO_PINE64.h

#include <linux/version.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

uint32_t SUNXI_CFG[] = {SUNXI_CFG0_OFFSET, SUNXI_CFG1_OFFSET, SUNXI_CFG2_OFFSET, SUNXI_CFG3_OFFSET};

volatile uint32_t *pio_map;


GPIO_PINE64::GPIO_PINE64() 
{}

/**
 *参数：void
 *返回值：void
 *功能：映射PINE64的GPIO寄存器地址到内存
 */
void GPIO_PINE64::init() {
    int mem_fd;
    uint32_t * gpio_map;

    // mmap the GPIO memory registers
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
        AP_HAL::panic("open /dev/mem error!\n");

    gpio_map = (uint32_t *)mmap(0, PINE64_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, SUNXI_GPIO_BASE & ~PINE64_MAP_MASK);

    if (gpio_map < 0) {
        AP_HAL::panic("PINE64 mmap gpio register address error!\n");
    }
    
    pio_map = (uint32_t *)(gpio_map + (SUNXI_GPIO_OFFSET >> 2));

    close(mem_fd);

    //设置CS引脚为输出，初始化为高电平
    pinMode(PINE_A64_GPIO_PC4, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC4, HIGH);
    pinMode(PINE_A64_GPIO_PC5, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC5, HIGH);
    pinMode(PINE_A64_GPIO_PC6, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC6, HIGH);
    pinMode(PINE_A64_GPIO_PC7, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC7, HIGH);
    pinMode(PINE_A64_GPIO_PC8, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC8, HIGH);
    pinMode(PINE_A64_GPIO_PC9, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC9, HIGH);
    pinMode(PINE_A64_GPIO_PC10, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC10, HIGH);
    pinMode(PINE_A64_GPIO_PC11, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC11, HIGH);
    pinMode(PINE_A64_GPIO_PC12, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PC12, HIGH);
    pinMode(PINE_A64_GPIO_PH6, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PH6, HIGH);

    //LED-RESET输出高电平
    pinMode(PINE_A64_GPIO_PH5, HAL_GPIO_OUTPUT);
    write(PINE_A64_GPIO_PH5, HIGH);
    
}

/**
 *参数1：pin GPIO_PINE64.h中宏定义的引脚
 *参数2：mode 设置模式
 *返回值：void
 *功能：设置引脚的模式
 */
void GPIO_PINE64::pinMode(uint8_t pin, uint8_t mode){
    if (mode == HAL_GPIO_INPUT) {
        set_gpio_direction(pin, 1);
    } else {
        set_gpio_direction(pin, 0);
    }
}

int8_t GPIO_PINE64::analogPinToDigitalPin(uint8_t pin){
    return -1;
}


/**
 *参数1：pin GPIO_PINE64.h中宏定义的引脚
 *返回值：高定平返回1
 *功能：读取引脚的状态
 */
uint8_t GPIO_PINE64::read(uint8_t pin){
    uint8_t value = get_gpio_value(pin);
    return value ? 1: 0;
}

/**
 *参数1：pin GPIO_PINE64.h中宏定义的引脚
 *参数2：value 1为高电平
 *返回值：void
 *功能：设置引脚的状态
 */
void GPIO_PINE64::write(uint8_t pin, uint8_t value){
    if (value == LOW) {
        set_gpio_value(pin,0);
    } else {
        set_gpio_value(pin,1);
    }
}

/**
 *参数1：pin GPIO_PINE64.h中宏定义的引脚
 *返回值：void
 *功能：切换引脚的状态
 */
void GPIO_PINE64::toggle(uint8_t pin){
    write(pin, !read(pin));
}


AP_HAL::DigitalSource* GPIO_PINE64::channel(uint16_t n){
    return new DigitalSource(n);
}

bool  GPIO_PINE64::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode){
    return true;
}


bool GPIO_PINE64::usb_connected(void){
    return false;
}

uint8_t GPIO_PINE64::get_gpio_value(uint8_t pin)
{
    uint32_t offset;
    uint32_t data;
    uint8_t state;
    uint32_t gpio_num;

    gpio_num = gpio_index[pin - GPIO_INDEX_HEAD];

    offset = (gpio_num/100) * SUNXI_BASE_OFFSET + SUNXI_DAT_OFFSET;
    data = *(pio_map + (offset >> 2));
    state = (data >> (gpio_num % 100)) & 0x00000001;
    if (state) {
        return 1;
    }
    return 0;
}


void  GPIO_PINE64::set_gpio_value(uint8_t pin, uint8_t value) 
{
    uint32_t offset;
    uint8_t bit_offset;
    uint32_t data;
    uint32_t gpio_num;

    gpio_num = gpio_index[pin - GPIO_INDEX_HEAD];


    offset = (gpio_num/100) * SUNXI_BASE_OFFSET + SUNXI_DAT_OFFSET;
    bit_offset = gpio_num % 100;
    data = *(pio_map + (offset >> 2));
    if (value) {
        data |= (1 << bit_offset);
    }else {
        data &= ~(1 << bit_offset);
    }
    *(pio_map + (offset >> 2)) = data;
}


void  GPIO_PINE64::set_gpio_direction(uint8_t pin, uint8_t direct)
{
    uint32_t offset;
    uint8_t bit_offset;
    uint32_t data;
    uint32_t gpio_num;

    gpio_num = gpio_index[pin - GPIO_INDEX_HEAD];


    offset = (gpio_num/100) * SUNXI_BASE_OFFSET + SUNXI_CFG[CFG_INDEX(gpio_num % 100)];
    bit_offset = 4 * ((gpio_num % 100) - 8 * (CFG_INDEX(gpio_num % 100)));
    data = *(pio_map + (offset >> 2));
    if (direct) {
        data &= ~(0x7 << bit_offset); //set input mode
    }else {
        data &= ~(0x7 << bit_offset); //clean bit
        data |= (0x1 << bit_offset); //set output mode
    }
    *(pio_map + (offset >> 2)) = data;
}


#endif 
