/*
  independent watchdog support
 */

#include "hal.h"
#include "watchdog.h"

#ifndef IWDG_BASE
#if defined(STM32H7)
#define IWDG_BASE             0x58004800
#elif defined(STM32F7) || defined(STM32F4)
#define IWDG_BASE             0x40003000
#elif defined(STM32F1)
#define IWDG_BASE             0x40003000
#else
#error "Unsupported IWDG MCU config"
#endif
#endif

#ifndef RCC_BASE
#error "Unsupported IWDG RCC MCU config"
#endif

/*
  defines for working out if the reset was from the watchdog
 */
#if defined(STM32H7)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + ))
#define WDG_RESET_CLEAR (1U<<16)
#define WDG_RESET_IS_IWDG (1U<<26)
#elif defined(STM32F7) || defined(STM32F4)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0x74))
#define WDG_RESET_CLEAR (1U<<24)
#define WDG_RESET_IS_IWDG (1U<<29)
#elif defined(STM32F1)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0x24))
#define WDG_RESET_CLEAR (1U<<24)
#define WDG_RESET_IS_IWDG (1U<<29)
#else
#error "Unsupported IWDG MCU config"
#endif

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_Regs;

#define IWDGD (*(IWDG_Regs *)(IWDG_BASE))

/*
  setup the watchdog
 */
void stm32_watchdog_init(void)
{
    // setup for 1s reset
    IWDGD.KR = 0x5555;
    IWDGD.PR = 8;
    IWDGD.RLR = 0xFFF;
    IWDGD.KR = 0xCCCC;
}

/*
  pat the dog, to prevent a reset. If not called for 1s
  after stm32_watchdog_init() then MCU will reset
 */
void stm32_watchdog_pat(void)
{
    IWDGD.KR = 0xAAAA;
}

static bool was_watchdog_reset;

/*
  save reason code for reset
 */
void stm32_watchdog_save_reason(void)
{
    if (WDG_RESET_STATUS & WDG_RESET_IS_IWDG) {
        was_watchdog_reset = true;
    }
    WDG_RESET_STATUS = WDG_RESET_CLEAR;
}

/*
  return true if reboot was from a watchdog reset
 */
bool stm32_was_watchdog_reset(void)
{
    return was_watchdog_reset;
}
