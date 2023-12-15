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

/* This file contains a standard set of commands that are used by the
 * esp8266 web interface.
 * You can add your own commands if needed
 */
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include "hwdefs.h"
#include "terminal.h"
#include "params.h"
#include "my_string.h"
#include "my_fp.h"
#include "printf.h"
#include "param_save.h"
#include "errormessage.h"
#include "stm32_can.h"
#include "terminalcommands.h"
#include "chargercan.h"

static void LoadDefaults(Terminal* term, char *arg);
static void PrintSerial(Terminal* term, char *arg);
static void MapCan(Terminal* term, char *arg);
static void PrintErrors(Terminal* term, char *arg);

static Terminal* curTerm = NULL;

extern "C" const TERM_CMD termCmds[] =
{
  { "set", TerminalCommands::ParamSet },
  { "get", TerminalCommands::ParamGet },
  { "flag", TerminalCommands::ParamFlag },
  { "stream", TerminalCommands::ParamStream },
  { "defaults", LoadDefaults },
  { "save", TerminalCommands::SaveParameters },
  { "load", TerminalCommands::LoadParameters },
  { "json", TerminalCommands::PrintParamsJson },
  { "can", MapCan },
  { "serial", PrintSerial },
  { "errors", PrintErrors },
  { "reset", TerminalCommands::Reset },
  { NULL, NULL }
};

void PrintCanMap(Param::PARAM_NUM param, uint32_t canid, uint8_t offsetBits, uint8_t length, float gain, int8_t offset, bool rx)
{
   const char* name = Param::GetAttrib(param)->name;
   fprintf(curTerm, "can ");

   if (rx)
      fprintf(curTerm, "rx ");
   else
      fprintf(curTerm, "tx ");
   fprintf(curTerm, "%s %d %d %d %f %d\r\n", name, canid, offsetBits, length, FP_FROMFLT(gain), offset);
}

//cantx param id offset len gain
static void MapCan(Terminal* term, char *arg)
{
   arg = my_trim(arg);

   if (arg[0] == 'd')
   {
      fprintf(term, "CAN map can not be deleted selectively, use clear command\r\n");
   }

   if (arg[0] == 'c')
   {
      TerminalCommands::MapCan(term, arg);
      //ChargerCAN::MapMessages(Can::GetInterface(0));
      fprintf(term, "Default CAN mapping restored\r\n");
      return;
   }

   TerminalCommands::MapCan(term, arg);
}

static void LoadDefaults(Terminal* term, char *arg)
{
   arg = arg;
   Param::LoadDefaults();
   fprintf(term, "Defaults loaded\r\n");
}

static void PrintErrors(Terminal* term, char *arg)
{
   term = term;
   arg = arg;
   ErrorMessage::PrintAllErrors();
}

static void PrintSerial(Terminal* term, char *arg)
{
   arg = arg;
   fprintf(term, "%X:%X:%X\r\n", DESIG_UNIQUE_ID2, DESIG_UNIQUE_ID1, DESIG_UNIQUE_ID0);
}
