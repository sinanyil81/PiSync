/*
 * Copyright (c) 2013, EGE University, Izmir, Turkey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @author: K. Sinan YILDIRIM (sinanyil81@gmail.com)
 */

#include "TimeSyncMsg.h"

configuration TimeSyncMicroC
{
  uses interface Boot;
  provides interface Init;
  provides interface StdControl;
  provides interface GlobalTime<TMicro>;

  //interfaces for extra fcionality: need not to be wired
  provides interface TimeSyncInfo;
  provides interface TimeSyncMode;
  provides interface TimeSyncNotify;
}

implementation
{
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
;
#else
#error "LPL timesync is not available for your platform"
#endif

  components new TimeSyncP(TMicro) as TimeSyncP;
  //components new TimeSyncPulseP(TMicro) as TimeSyncP;

  GlobalTime      =   TimeSyncP;
  StdControl      =   TimeSyncP;
  Init            =   TimeSyncP;
  Boot            =   TimeSyncP;
  TimeSyncInfo    =   TimeSyncP;
  TimeSyncMode    =   TimeSyncP;
  TimeSyncNotify  =   TimeSyncP;

  components TimeSyncMessageC as ActiveMessageC;
  TimeSyncP.RadioControl    ->  ActiveMessageC;
  TimeSyncP.Send            ->  ActiveMessageC.TimeSyncAMSendRadio[TIMESYNC_AM_FTSP];
  TimeSyncP.Receive         ->  ActiveMessageC.Receive[TIMESYNC_AM_FTSP];
  TimeSyncP.TimeSyncPacket  ->  ActiveMessageC;

  components LocalTimeMicroC;
  TimeSyncP.LocalTime     -> LocalTimeMicroC;

  components new TimerMilliC() as TimerC;
  TimeSyncP.Timer ->  TimerC;

  components RandomC;
  TimeSyncP.Random -> RandomC;
    
#if defined(TIMESYNC_LEDS)
  components LedsC;
#else
  components NoLedsC as LedsC;
#endif
  TimeSyncP.Leds  ->  LedsC;

#ifdef LOW_POWER_LISTENING
  TimeSyncP.LowPowerListening -> ActiveMessageC;
#endif
}
