/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#ifdef TEST_COMMON_H
#include "../test/timer_mock.h"
#else
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#endif
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/crc.h>
#include "stm32_can.h"
#include "cansdo.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "picontroller.h"
#include "chargercan.h"
#include "charger.h"
#include "terminalcommands.h"

#define PRINT_JSON 0

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
PiController dcCurController;
uint32_t startTime;

//sample 100ms task
static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   //The boot loader enables the watchdog, we have to reset it
   //at least every 2s or otherwise the controller is hard reset.
   iwdg_reset();
   //Calculate CPU load. Don't be surprised if it is zero.
   float cpuLoad = scheduler->GetCpuLoad() / 10.0f;
   //This sets a fixed point value WITHOUT calling the parm_Change() function
   Param::SetFloat(Param::cpuload, cpuLoad);
   //Set timestamp of error message
   ErrorMessage::SetTime(rtc_get_counter_val());
   Param::SetInt(Param::uptime, rtc_get_counter_val());
   Param::SetFloat(Param::uaux, AnaIn::uaux.Get() / 223.418f);

   ResetValuesInOffMode();
   CalcTotals();
   CalcEnable();
   CalcAcCurrentLimit();
   ChargerStateMachine();

   EvseRead();

   canMap->SendAll();
}

static void MapChargerMessages()
{
   uint32_t dummyId;
   uint8_t dummyOfs;
   int8_t dummyAdd, dummyLen;
   float dummyGain;
   bool dummyrx;

   //check sample value, if it is mapped assume valid CAN map
   if (canMap->FindMap(Param::hwaclim, dummyId, dummyOfs, dummyLen, dummyGain, dummyAdd, dummyrx)) return;

   //canMap->Clear();

   ChargerCAN::MapMessages(canMap);

   canMap->Save();
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   s32fp spnt;

   switch (paramNum)
   {
      case Param::idckp:
      case Param::idcki:
         dcCurController.SetGains(Param::GetInt(Param::idckp), Param::GetInt(Param::idcki));
         break;
      case Param::idclim:
      case Param::idcspnt:
         spnt = MIN(Param::Get(Param::idcspnt), Param::Get(Param::idclim));
         dcCurController.SetRef(spnt);
         break;
      default:
         //Handle general parameter changes here. Add paramNum labels for handling specific parameters
         break;
   }
}

static void HandleClear()
{
   MapChargerMessages();
}

static bool CanCallback(uint32_t, uint32_t*, uint8_t)
{
   return false;
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   extern const TERM_CMD termCmds[];

   clock_setup(); //Must always come first
   rtc_setup();
   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);
   AnaIn::Start(); //Starts background ADC conversion via DMA
   write_bootloader_pininit(); //Instructs boot loader to initialize certain pins
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB);
   tim_setup(); //Use timer3 for sampling pilot PWM
   nvic_setup(); //Set up some interrupts
   parm_load(); //Load stored parameters
   Param::Change(Param::idckp); //Call callback once for parameter propagation
   Param::Change(Param::idclim); //Call callback once for parameter propagation

   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;
   //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
   Stm32Can c(CAN1, CanHardware::Baud500, true);
   FunctionPointerCallback cb(CanCallback, HandleClear);
   c.AddCallback(&cb);
   can = &c;

   //store a pointer for easier access
   CanMap cm(&c);
   canMap = &cm;
   CanSdo sdo(&c, &cm);
   sdo.SetNodeId(2);

   TerminalCommands::SetCanMap(&cm);

   Terminal t3(USART3, termCmds);
   Terminal t1(USART1, termCmds);

   MapChargerMessages();
   dcCurController.SetCallingFrequency(10);

   //Up to four tasks can be added to each timer scheduler
   //AddTask takes a function pointer and a calling interval in milliseconds.
   //The longest interval is 655ms due to hardware restrictions
   //You have to enable the interrupt (int this case for TIM2) in nvic_setup()
   //There you can also configure the priority of the scheduler over other interrupts
   s.AddTask(Ms100Task, 100);

   //backward compatibility, version 4 was the first to support the "stream" command
   Param::SetInt(Param::version, 4);
   Param::SetFlag(Param::test_time, Param::FLAG_HIDDEN);
   Param::SetFlag(Param::test_timer_flag, Param::FLAG_HIDDEN);
   Param::SetFlag(Param::test_timer_icvalue, Param::FLAG_HIDDEN);

   //In version 1.11 this changed from mV to V
   if (Param::GetInt(Param::udcspnt) > 420)
   {
      Param::SetFloat(Param::udcspnt, Param::GetFloat(Param::udcspnt) / 1000);
   }

   //Now all our main() does is running the terminal
   //All other processing takes place in the scheduler or other interrupt service routines
   //The terminal has lowest priority, so even loading it down heavily will not disturb
   //our more important processing routines.
   while(1)
   {
      t1.Run();
      t3.Run();

      if (sdo.GetPrintRequest() == PRINT_JSON)
      {
         char c = 0;
         TerminalCommands::PrintParamsJson(&sdo, &c);
      }
   }


   return 0;
}

