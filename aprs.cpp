/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "config.h"
#include "ax25.h"
#include "gps.h"
#include "aprs.h"
#include "sensors.h"
#include "hw.h"
#include "afsk_avr.h"
//#include "sensors_pic32.h"
#include <stdio.h>
#include <stdlib.h>
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif

static uint32_t telemetry_packet_num = 0;

// Module functions
float meters_to_feet(float m)
{
  // 10000 ft = 3048 m
  return m / 0.3048;
}

int aprs_format (float value, float multiplier, float offset)
{
  float x;
  x = value - offset;
  x = x / multiplier;
  // range limit
  if (x > 255) x = 255.0;
  if (x < 0) x = 0.0;
  return (int)x;
}

void aprs_begin()
{
  // send APRS header
  const struct s_address addresses[] = { 
  {D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
  {S_CALLSIGN, S_CALLSIGN_ID},  // Source callsign (-11 = balloon, -9 = car)
#ifdef DIGI_PATH1
    {DIGI_PATH1, DIGI_PATH1_TTL}, // Digi1 (first digi in the chain)
#endif
#ifdef DIGI_PATH2
    {DIGI_PATH2, DIGI_PATH2_TTL}, // Digi2 (second digi in the chain)
#endif
  };
  ax25_send_header(addresses, sizeof(addresses)/sizeof(s_address));
}

// special call sign formatting required for telemetry setup
int aprs_make_call(int len, char* str)
{
  if (len < 10) return -1; // need minimum of 9 characters + string end 00
  if (S_CALLSIGN_ID > 0) {
    snprintf(str, len, "%s-%d", S_CALLSIGN, S_CALLSIGN_ID);
  }
  else {
    snprintf(str, len, "%s   ", S_CALLSIGN);  // with ID of 0 we need three spaces after the call sign
  }
  return 0;
}

void aprs_end()
{
  ax25_send_footer();
  ax25_flush_frame();                 // Tell the modem to go
}

// Exported functions
void aprs_beacon_send()
{
  char temp[12];                   // Temperature (int/ext)
  aprs_begin();
  /*
  // ax25_send_string("021709z");     // 021709z = 2nd day of the month, 17:09 zulu (UTC/GMT)
  ax25_send_string(gps_time);         // 170915 = 17h:09m:15s zulu (not allowed in Status Reports)
  ax25_send_byte('h');
  ax25_send_string(gps_aprs_lat);     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
  ax25_send_byte('/');                // Symbol table
  ax25_send_string(gps_aprs_lon);     // Lon: 000deg and 25.80 min
  ax25_send_byte('O');                // Symbol: O=balloon, -=QTH
  snprintf(temp, 4, "%03d", (int)(gps_course + 0.5)); 
  ax25_send_string(temp);             // Course (degrees)
  ax25_send_byte('/');                // and
  snprintf(temp, 4, "%03d", (int)(gps_speed + 0.5));
  ax25_send_string(temp);             // speed (knots)
  ax25_send_string("/A=");            // Altitude (feet). Goes anywhere in the comment area
  snprintf(temp, 7, "%06ld", (long)(meters_to_feet(gps_altitude) + 0.5));
  ax25_send_string(temp);
  ax25_send_string("/Ti=");
  snprintf(temp, 6, "%d", sensors_int_lm60());
  ax25_send_string(temp);
  ax25_send_string("/Te=");
  snprintf(temp, 6, "%d", sensors_ext_lm60());
  ax25_send_string(temp);
  ax25_send_string("/V=");
  snprintf(temp, 6, "%d", sensors_vin());
  ax25_send_string(temp);
  ax25_send_byte(' ');
  */
 //ax25_send_byte('/');                // Position Report w/ timestamp, no APRS messaging. $ = NMEA raw data
  ax25_send_byte('!');    // Position Report without timestamp
  ax25_send_string(APRS_POS_LAT);   // Latitude
  ax25_send_byte(APRS_SYM_TABLE);
  ax25_send_string(APRS_POS_LONG);  // Longitude
  ax25_send_byte(APRS_SYMBOL);
  ax25_send_string(APRS_COMMENT);     // Comment
  aprs_end();
 }

// send telemetry data
void aprs_tele_send()
{
  // T#261,087,077,189,020,005,00000000live data
  char str[12];              
  aprs_begin();
  snprintf(str, 6, "T#%03d", telemetry_packet_num);    // TX Temperature
  ax25_send_string(str);   
  snprintf(str, 5, ",%03d", aprs_format(sensors_tx_temp() ,0.3333, -10.0));    // TX Temperature
  ax25_send_string(str);
  snprintf(str, 5, ",%03d", aprs_format(sensors_env_temp() ,0.3333, -10.0));    // Env Temperature
  ax25_send_string(str);
  snprintf(str, 5, ",%03d", aprs_format(sensors_voltage() ,0.03333, 8.0));           // Voltage
  ax25_send_string(str);
  snprintf(str, 5, ",%03d", aprs_format(sensors_current() ,0.1, 0.0));         //Current
  ax25_send_string(str);
  snprintf(str, 5, ",%03d", aprs_format(sensors_power() ,1, 0.0));           // Power
  ax25_send_string(str);
  
  ax25_send_string(",00000000");     
  ax25_send_string("Loguino");                        // Comment
  aprs_end();
  telemetry_packet_num++;
}

// send telemetry parameter setup
void aprs_setup()
{
  char call_str[12]; 
  aprs_make_call(12, &call_str[0]);   

  aprs_begin();
  ax25_send_byte(':');
  ax25_send_string(call_str);
  ax25_send_string(":PARM.");
  ax25_send_string("TX Temp,Env Tmp,Voltage,Current,Power");
  aprs_end();
  while (afsk_flush()) {
    power_save();
  }
  psDelay(500);
  
  aprs_begin();
  ax25_send_byte(':');      
  ax25_send_string(call_str);
  ax25_send_string(":UNIT.");
  ax25_send_string("degC,degC,V,A,Wh");
  aprs_end();
  while (afsk_flush()) {
    power_save();
  }
  psDelay(500);
  
  aprs_begin();
  ax25_send_byte(':');     
  ax25_send_string(call_str);
  ax25_send_string(":EQNS.");
  ax25_send_string("0,0.3333,-10.0"); // TX temp degC, 1/3 deg step, -10deg offset (range -10 to +75 degC)
  ax25_send_string(",0,0.3333,-10.0");// Env temp degC, 1/3 deg step, -10deg offset (range -10 to +75 degC)
  ax25_send_string(",0,0.03333,8.0");// Voltage V, 1/30V steps, 8V offset (range 8-16.5V)
  ax25_send_string(",0,0.1,0");// Current A, 0 offset, 0.1A steps (range 0-25.5A)
  ax25_send_string(",0,1,0");// Power Wh, 0 offset, 0.1A steps (range 0-255Wh) [max 12V * 20A = 240Wh]
  aprs_end();
  while (afsk_flush()) {
    power_save();
  }
  psDelay(500);
  
  aprs_begin();
  ax25_send_byte(':');     
  ax25_send_string(call_str);
  ax25_send_string(":BITS.");
  ax25_send_string("11111111");
  ax25_send_string(",Repeater");
  aprs_end();
  while (afsk_flush()) {
    power_save();
  }
  psDelay(500);
}

