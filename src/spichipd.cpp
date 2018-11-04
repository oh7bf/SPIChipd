/**************************************************************************
 * 
 * Read chips with Serial Peripheral Interface (SPI). 
 *       
 * Copyright (C) 2018 Jaakko Koivuniemi.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************************
 *
 * Sat  3 Nov 20:21:27 CDT 2018
 * Edit: Sun  4 Nov 14:31:27 CST 2018
 *
 * Jaakko Koivuniemi
 **/

#include "spichipd.hpp"
#include <systemd/sd-daemon.h>
#include "signal.h"
#include <unistd.h>
#include <iostream>

using namespace std;

bool cont = true;

void shutdown(int sig)
{
  fprintf(stderr, SD_WARNING "SIGTERM received, shut down\n");
  cont = false;
}

void reload(int sig)
{
  fprintf(stderr, SD_WARNING "SIGHUP received (to implement: reload configuration)\n");
}


/// spichipd program to read SPI chips at regular intervals 

/// Run the program with `spichipd 2> /dev/null` on shell. Printing
/// to standard error stream is for _systemd_ logging with _systemd-journald_
/// and includes different log levels defined in `sd-daemon.h`.
int main()
{
  string name = "T1";
  string device = "/dev/spidev0.0";
  uint32_t clock = 500000; 
  double Rref = 400.0;
  int readinterval = 10;

  signal(SIGTERM, &shutdown);
  signal(SIGHUP, &reload);

  Max31865 chip = Max31865(name, device, clock, Rref);

  chip.SetHighFault(40000);
  cout << chip.GetName() << " high fault = " << chip.GetHighFault(); 
  chip.SetLowFault(400);
  cout << " low fault = " << chip.GetLowFault() << " \n";

  chip.BiasOn();
  usleep( 500000 );

  while( cont )
  {
    chip.OneShot();
    usleep( 100000 );

    cout << chip.GetName() << " = " << chip.GetTemperature();
    cout << " C  fault = " << (int)chip.GetFaultStatusByte() << "\n";
 
    sleep( readinterval );
  }

  return 0;
};

