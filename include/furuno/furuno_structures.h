/*!
 * \file furuno/furuno_structures.h
 * \author RENAME
 * Portions based on previous code by William Travis and Scott Martin
 * \version 1.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 RENAME
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides structure definitions for messages output by a Novatel GPS.
 *
 */


#ifndef FURUNOSTRUCTURES_H
#define FURUNOSTRUCTURES_H

#include "furuno_enums.h"

#define MAX_NOUT_SIZE      (8192)   // Maximum size of a NovAtel log buffer (ALMANACA logs are big!)
#define EPH_CHAN 33
#define NUMSAT 14
#define MAX_CHAN	28  // Maximum number of signal channels
#define MAX_NUM_SAT 28	// Maximum number of satellites with information in the RTKDATA log
#define HEADER_SIZE 28 // Binary header size for OEM 4, V, and 6 receivers
#define SHORT_HEADER_SIZE 12 // short binary header size
#define POS_MSG_ID 0x50
#define NAV_MSG_ID 0x62
#define POS_MSG_LEN 266
#define NAV_MSG_LEN 35

 #ifndef  GPS_CLOCK_CORRECTION_RELATIVISTIC_CONSTANT_F
 #define  GPS_CLOCK_CORRECTION_RELATIVISTIC_CONSTANT_F  (-4.442807633e-10)  //!< combined constant defined in ICD-GPS-200C p. 88     [s]/[sqrt(m)]
 #endif
 #ifndef  GPS_UNIVERSAL_GRAVITY_CONSTANT
 #define  GPS_UNIVERSAL_GRAVITY_CONSTANT                (3.986005e14)       //!< gravity constant defined on ICD-GPS-200C p. 98      [m^3/s^2]
 #endif
 #ifndef  GPS_RATIO_OF_SQUARED_FREQUENCIES_L1_OVER_L2   
 #define  GPS_RATIO_OF_SQUARED_FREQUENCIES_L1_OVER_L2   (1.6469444444444444444444444444444) //!< (f_L1/f_L2)^2 = (1575.42/1227.6)^2 = (77/60)^2
 #endif
 #ifndef  GPS_WGS84_EARTH_ROTATION_RATE
 #define  GPS_WGS84_EARTH_ROTATION_RATE                 (7.2921151467e-05)  //!< constant defined on ICD-GPS-200C p. 98            [rad/s]
 #endif
 
 #define TWO_TO_THE_POWER_OF_55  (36028797018963968.0)
 #define TWO_TO_THE_POWER_OF_43  (8796093022208.0)
 #define TWO_TO_THE_POWER_OF_33  (8589934592.0)
 #define TWO_TO_THE_POWER_OF_31  (2147483648.0)
 #define TWO_TO_THE_POWER_OF_29  (536870912.0)
 #define TWO_TO_THE_POWER_OF_19  (524288.0)

#define PI (3.1415926535897932384626433832795)


// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#include <stdint.h>  // use fixed size integer types, rather than standard c++ types


//*******************************************************************************
// HEADER STRUCTURES
//*******************************************************************************

PACK(
struct channel
{
	unsigned char prn;					//!< satellite number
	unsigned char tracking_mode;		//!< trackin mode
	unsigned char carrier_to_noise;		//!< signal to noise ration (C/No) in (db/Hz)
	unsigned char num_cycle_slip;		//!< number of cycle slip
	long carrier_phase;					//!< accumulated delta range ADR (Google it)
	long doppler;						//!< doppler frequency
	long pseudorange;
});

PACK(
struct channelOutput
{
	unsigned char prn;					//!< satellite number
	unsigned char tracking_mode;		//!< trackin mode
	unsigned char carrier_to_noise;		//!< signal to noise ration (C/No) in (db/Hz)
	unsigned char num_cycle_slip;		//!< number of cycle slip
	double carrier_phase;				//!< accumulated delta range ADR (Google it)
	double doppler;						//!< doppler frequency
	double pseudorange;
});
//*******************************************************************************
// GENERIC GPS STRUCTURES
//*******************************************************************************

/*!
 * Position Message Structure
 * This log contains the a position received from the receiver
 * in latitude and longitude as well as status information such as
 * the solution type and the number of satellites used.
 */

PACK(
struct Position
{
	unsigned char sync_byte;			//!< Message header
	unsigned char message_id;			//!< Message ID
	unsigned long gps_time_sec;			//!< GPS time in seconds
	unsigned short gps_time_msec;		//!< GPS time in milliseconds
	unsigned char num_of_sats;			//!< number of satellites used for fix
	unsigned char gps_pos_status;		//!< gps postion status
	long position_x;					//!< ecef x position in meters
	long position_y;					//!< ecef y position in meters
	long position_z;					//!< ecef z position in meters
	long velocity_x;					//!< ecef x velocity (m/s)
	long velocity_y;					//!< ecef y velocity (m/s)
	long velocity_z;					//!< ecef z velocity (m/s)
	unsigned char hdop;					//!< hdop (Google it)
	unsigned char vdop;					//!< vdop
	unsigned long sample_offset;		//!< sample offset
	
	//instances of channel structure for each tracking channel
	channel channel[14];

	unsigned short check_sum;					//!< check sum

});

PACK(
struct PositionOutput
{
	unsigned long gps_week;
	unsigned long gps_time_sec;			//!< GPS time in seconds
	unsigned short gps_time_msec;		//!< GPS time in milliseconds
	unsigned char num_of_sats;			//!< number of satellites used for fix
	unsigned char gps_pos_status;		//!< gps postion status
	double position_x;					//!< ecef x position in meters
	double position_y;					//!< ecef y position in meters
	double position_z;					//!< ecef z position in meters
	double velocity_x;					//!< ecef x velocity (m/s)
	double velocity_y;					//!< ecef y velocity (m/s)
	double velocity_z;					//!< ecef z velocity (m/s)
	double hdop;						//!< hdop (Google it)
	double vdop;						//!< vdop
	double sample_offset;				//!< sample offset
	
	//instances of channel structure for each tracking channel
	channelOutput channelOutput[14];

	unsigned short check_sum;					//!< check sum

});
//*******************************************************************************
// SATELLITE INFORMATION GPS STRUCTURES
//*******************************************************************************


/*!
 * GPSEPHEM Message Structure
 * This log contains a single set of
 * GPS ephemeris parametres.
 */


PACK(
struct NavMessage
{
	unsigned char sync_byte;
	unsigned char message_id;
	unsigned char sat_num;
	char nav_word[10][3];
	unsigned short check_sum;
});


struct GpsEphemeris
{ 
   unsigned short  prn;                //!< GPS PRN number (helps with debugging)
   unsigned        tow;                //!< time of week in subframe1, the time of the leading bit edge of subframe 2     [s]
   unsigned short  iodc;               //!< 10 bit issue of data (clock), 8 LSB bits will match the iode                  []    
   unsigned char   iode;               //!< 8 bit  issue of data (ephemeris)                                              []
   unsigned        toe;                //!< reference time ephemeris (0-604800)                                           [s]
   unsigned        toc;                //!< reference time (clock)   (0-604800)                                           [s]      
   unsigned short  week;               //!< 10 bit gps week 0-1023 (user must account for week rollover )                 [week]    
   unsigned char   health;             //!< 6 bit health parameter, 0 if healthy, unhealth othersize                      [0=healthy]    
   unsigned char   alert_flag;         //!< 1 = URA may be worse than indicated                                           [0,1]
   unsigned char   anti_spoof;         //!< anti-spoof flag from 0=off, 1=on                                              [0,1]    
   unsigned char   code_on_L2;         //!< 0=reserved, 1=P code on L2, 2=C/A on L2                                       [0,1,2]
   unsigned char   ura;                //!< User Range Accuracy lookup code, 0 is excellent, 15 is use at own risk        [0-15], see p. 83 GPSICD200C
   unsigned char   L2_P_data_flag;     //!< flag indicating if P is on L2 1=true                                          [0,1]
   unsigned char   fit_interval_flag;  //!< fit interval flag (four hour interval or longer) 0=4 fours, 1=greater         [0,1]
   unsigned short  age_of_data_offset; //!< age of data offset                                                            [s]
   double  tgd;              //!< group delay                                                                   [s]
   double  af2;                //!< polynomial clock correction coefficient (rate of clock drift)                 [s/s^2]
   double  af1;              //!< polynomial clock correction coefficient (clock drift)                         [s/s]
   double  af0;              //!< polynomial clock correction coefficient (clock bias)                          [s]    
   double  m0;               //!< mean anomaly at reference time                                                [rad]
   double  delta_n;          //!< mean motion difference from computed value                                    [rad/s]
   double  ecc;              //!< eccentricity                                                                  []
   double  sqrta;            //!< square root of the semi-major axis                                            [m^(1/2)]
   double  omega0;           //!< longitude of ascending node of orbit plane at weekly epoch                    [rad]
   double  i0;               //!< inclination angle at reference time                                           [rad]
   double  w;                //!< argument of perigee                                                           [rad]
   double  omegadot;         //!< rate of right ascension                                                       [rad/s]
   double  idot;             //!< rate of inclination angle                                                     [rad/s]
   double  cuc;              //!< amplitude of the cosine harmonic correction term to the argument of latitude  [rad]
   double  cus;              //!< amplitude of the sine harmonic correction term to the argument of latitude    [rad]
   double  crc;              //!< amplitude of the cosine harmonic correction term to the orbit radius          [m]
   double  crs;              //!< amplitude of the sine harmonic correction term to the orbit radius            [m]
   double  cic;              //!< amplitude of the cosine harmonic correction term to the angle of inclination  [rad]
   double  cis;                //!< amplitude of the sine harmonic correction term to the angle of inclination    [rad]

   unsigned char subframe_id;    // subrame id
};


/*struct BigEphemeris
{
	GpsEphemeris GpsEphemeris[32];
};*/

/*PACK(
struct GpsEphemeris
{
   // Oem4BinaryHeader header;		//!< Message header
    uint32_t prn;                   //!< PRN number
    double time_of_week;            //!< time stamp of subframe 0 (s)
    uint32_t health;                //!< health status, defined in ICD-GPS-200
    uint32_t issue_of_ephemeris_1;  //!< issue of ephemeris data 1
    uint32_t issue_of_ephemeris_2;  //!< issue of ephemeris data 2
    uint32_t gps_week;              //!< GPS week number
    uint32_t z_count_week;          //!< z count week number
    double time_of_ephemeris;       //!< reference time for ephemeris (s)
    double semi_major_axis;         //!< semi major axis (m)
    double mean_motion_difference;  //!< Mean motion difference (rad/s)
    double anomoly_reference_time;  //!< mean anomoly reference time (rad)
    double eccentricity;            //!< eccentricity
    double omega;                   //!< arguement of perigee (rad)
    double latitude_cosine;         //!< arugument of latitude - cos (rad)
    double latitude_sine;           //!< argument of latitude - sine (rad)
    double orbit_radius_cosine;     //!< orbit radius - cos (rad)
    double orbit_radius_sine;       //!< orbit radius - sine (rad)
    double inclination_cosine;      //!< inclination - cos (rad)
    double inclination_sine;        //!< inclination - sine (rad)
    double inclination_angle;       //!< inclination angle (rad)
    double inclination_angle_rate;  //!< rate of inclination angle (rad/s)
    double right_ascension;         //!< right ascension (rad)
    double right_ascension_rate;    //!< rate of right ascension (rad/s)
    uint32_t issue_of_data_clock;   //!< issue of data clock
    double sv_clock_correction;     //!< SV clock correction term (s)
    double group_delay_difference;  //!< estimated group delay difference
    double clock_aligning_param_0;  //!< clock aiging parameter 0
    double clock_aligning_param_1;  //!< clock aiging parameter 1
    double clock_aligning_param_2;  //!< clock aiging parameter 2
    yes_no anti_spoofing;           //!< anti spoofing on
    double corrected_mean_motion;   //!< corrected mean motion
    double range_accuracy_variance; //!< user range accuracy variance
    uint8_t crc[4];                 //!< 32-bit cyclic redundancy check (CRC)
});*/

#endif
