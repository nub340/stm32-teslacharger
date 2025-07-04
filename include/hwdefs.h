#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


#define RCC_CLOCK_SETUP() rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ])

//Address of parameter block in flash
#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1   //last block of 1k
#define CAN1_BLKNUM   2
#define CAN2_BLKNUM   4

#endif // HWDEFS_H_INCLUDED
