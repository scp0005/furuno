#include <string>
#include <iostream>
#include <sstream>
#include <iomanip> // for setprecision()
#include "furuno\furuno.h"
#include <MOOSApp.h>

using namespace furuno;
using namespace std;

ofstream OutputFile;

void onBestPositionCallback(PositionOutput best_position, double time_stamp){
	if (!OutputFile)
	{
		cout << "\nERROR: File could not be opened.";
	}
	std::cout << setprecision(16); // show 16 digits
	OutputFile << setprecision(16);
	std::cout << "BESTPOS: \nGPS Week: " << best_position.gps_week <<
		" GPS seconds :" << best_position.gps_time_sec << std::endl <<
		" GPS milliseconds :" << best_position.gps_time_msec << std::endl <<
		"  ECEF X Pos: " << best_position.position_x << std::endl <<
		"  ECEF Y Pos: " << best_position.position_y << std::endl <<
		"  ECEF Z Pos: " << best_position.position_z << std::endl <<
		"  ECEF X Vel: " << best_position.velocity_x << std::endl <<
		"  ECEF Y Vel: " << best_position.velocity_y << std::endl <<
		"  ECEF Z Vel: " << best_position.velocity_z << std::endl <<
		" HDOP: " << hex << (int)best_position.hdop << std::endl <<
		" VDOP: " << hex << (int)best_position.vdop << std::endl <<
		" Number of Satellites: " << hex << (int)best_position.num_of_sats << std::endl <<
		" GPS Status: " << (int)best_position.gps_pos_status << dec << std::endl << std::endl;
		
		OutputFile << 5000 << 
		", " << best_position.gps_time_sec << 
		", " << "NaN" << 
		", " << "NaN" << 
		", " << hex << (int)best_position.hdop << dec << 
		", " << hex << (int)best_position.vdop << dec <<
		", " << "NaN" <<
		", " << hex << (int)best_position.num_of_sats << dec << std::endl;

		OutputFile << 5002 << 
		", " << best_position.gps_time_sec << 
		", " << best_position.position_x << 
		", " << best_position.position_y << 
		", " << best_position.position_z << 
		", " << "NaN" << 
		", " << "NaN" <<
		", " << "NaN" << std::endl;

		OutputFile << 5102 << 
		", " << best_position.gps_time_sec << 
		", " << best_position.velocity_x << 
		", " << best_position.velocity_y << 
		", " << best_position.velocity_z << 
		", " << "NaN" << 
		", " << "NaN" <<
		", " << "NaN" << std::endl;
	
	for (int i=0;i<14;i++)
	{
		if (best_position.channelOutput[i].prn <= 0.0){
		//std:: cout << std::endl << "CH #: " << i+1 <<std::endl;
		}
		else if(best_position.channelOutput[i].tracking_mode == 8) {
			std:: cout << std::endl << "CH #: " << i+1 <<std::endl <<
			"	Satellite #      :" << hex <<(float)best_position.channelOutput[i].prn			<< std::endl <<
			"	Tracking Mode    :" << (float)best_position.channelOutput[i].tracking_mode		<< std::endl <<
			"	Signal level     :" << (float)best_position.channelOutput[i].carrier_to_noise	<< std::endl <<
			"	# of cycle slips :" << (float)best_position.channelOutput[i].num_cycle_slip << dec << std::endl <<
			"	ADR              :" << best_position.channelOutput[i].carrier_phase	<< std::endl <<
			"	Doppler Frequency:" << best_position.channelOutput[i].doppler		<< std::endl <<
			"	Pseudorange      :" << best_position.channelOutput[i].pseudorange	<<std::endl;

		
			OutputFile << hex << 1000 + (float)best_position.channelOutput[i].prn << 
			", " << dec << best_position.gps_time_sec << 
			", " << best_position.channelOutput[i].pseudorange << 
			", " << "NaN" << 
			", " << best_position.channelOutput[i].carrier_phase << 
			", " << "NaN" << 
			", " << best_position.channelOutput[i].doppler <<
			", " << hex << (float)best_position.channelOutput[i].carrier_to_noise << dec << std::endl;
		
		}
	}
	OutputFile << "9999, NaN, NaN, NaN, NaN, NaN, NaN, NaN" << std::endl; 
}

void onNavMessageCallback(NavMessage nav_msg, GpsEphemeris furuno_ephemeris, double time_stamp){
	std:: cout << "Nav Message Callback" << std::endl <<
		"Satellite #      : " << hex << (float)nav_msg.sat_num << dec << std::endl <<
		"GPS Week         : "<< furuno_ephemeris.week << std::endl <<
		"GPS Time Of Week : "<< furuno_ephemeris.tow << std::endl;

	switch (furuno_ephemeris.subframe_id) {
		case 1:
		std:: cout << std::endl << "Subframe ID Check: "  << hex << (int)furuno_ephemeris.subframe_id << dec <<std::endl << 
		"	tow        : " << furuno_ephemeris.tow			<< std::endl <<
		"	alert_flag : " << hex << (int)furuno_ephemeris.alert_flag	<< dec << std::endl <<
		"	anti_spoof : " << hex << (int)furuno_ephemeris.anti_spoof	<< dec << std::endl <<
		"	week       : " << furuno_ephemeris.week		<< std::endl <<
		"	code_on_L2 : " << hex << (int)furuno_ephemeris.code_on_L2	<< dec <<std::endl <<
		"	ura        : " << hex << (float)furuno_ephemeris.ura << dec << std::endl <<
		"	health     : " << hex << (int)furuno_ephemeris.health << dec << std::endl <<
		"	iodc       : " << furuno_ephemeris.iodc 		<< std::endl <<
		"	L2_P_data_flag	: " << hex << (int)furuno_ephemeris.L2_P_data_flag << dec << std::endl <<
		"	tgd        : " << furuno_ephemeris.tgd 		<< std::endl <<
		"	toc        : " << furuno_ephemeris.toc 		<< std::endl <<
		"	af2        : " << furuno_ephemeris.af2 		<< std::endl <<
		"	af1        : " << furuno_ephemeris.af1 		<< std::endl <<
		"	af0        : " << furuno_ephemeris.af0 		<< std::endl;	
		OutputFile << hex << 4100 + (float)nav_msg.sat_num << 
			", " << dec << furuno_ephemeris.tow << 
			", " << furuno_ephemeris.week << 
			", " << furuno_ephemeris.toc <<
			", " << furuno_ephemeris.tgd << 
			", " << furuno_ephemeris.af0 <<
			", " << "NaN" <<
			", " << "NaN" << std::endl;
		OutputFile << hex << 4300 + (float)nav_msg.sat_num << 
			", " << dec << furuno_ephemeris.af1 <<			
			", " << furuno_ephemeris.af2 <<
			", " << hex << (float)furuno_ephemeris.ura <<
			", " << (int)furuno_ephemeris.health << dec <<
			", " << "NaN" <<
			", " << "NaN" <<
			", " << "NaN" <<std::endl;
			break;
		case 2:
		std:: cout << std::endl << "Subframe ID Check: " << hex << (int)furuno_ephemeris.subframe_id << dec << std::endl <<
		"	tow        : " << furuno_ephemeris.tow << std::endl <<
		"	alert_flag : " << hex << (int)furuno_ephemeris.alert_flag  << dec << std::endl <<
		"	anti_spoof : " << hex << (int)furuno_ephemeris.anti_spoof  << dec <<std::endl <<
		"	crs        : " << furuno_ephemeris.crs 		<< std::endl <<
		"	delta_n    : " << furuno_ephemeris.delta_n 	<< std::endl <<
		"	m0         : " << furuno_ephemeris.m0 			<< std::endl <<
		"	cuc        : " << furuno_ephemeris.cuc 		<< std::endl <<
		"	ecc        : " << furuno_ephemeris.ecc 		<< std::endl <<
		"	cus        : " << furuno_ephemeris.cus  		<< std::endl <<
		"	sqrta      : " << furuno_ephemeris.sqrta 		<< std::endl <<
		"	toe        : " << furuno_ephemeris.toe 		<< std::endl <<
		"	fit interval flag  : " << hex << (int)furuno_ephemeris.fit_interval_flag << dec << std::endl <<
		"	age of data offset : " << furuno_ephemeris.age_of_data_offset << std::endl;
		OutputFile << hex << 4000 + (float)nav_msg.sat_num << 
			", " << dec << furuno_ephemeris.tow <<
			", " << "NaN" <<
			", " << furuno_ephemeris.toe <<
			", " << furuno_ephemeris.sqrta << 
			", " << furuno_ephemeris.delta_n << 
			", " << furuno_ephemeris.m0 <<
			", " << furuno_ephemeris.ecc << std::endl;
		OutputFile << hex << 4400 + (float)nav_msg.sat_num << 
			", " << dec << furuno_ephemeris.cuc << 
			", " << furuno_ephemeris.cus <<
			", " << furuno_ephemeris.crs <<
			", " << "NaN" <<
			", " << "NaN" <<
			", " << "NaN" <<
			", " << "NaN" << std::endl;
			break;
		case 3:
		std:: cout << std::endl << "Subframe ID Check: "  << hex << (int)furuno_ephemeris.subframe_id << dec << std::endl <<
		"	tow        : " << furuno_ephemeris.tow	<< std::endl <<
		"	alert_flag : " << hex << (int)furuno_ephemeris.alert_flag  << dec << std::endl <<
		"	anti_spoof : " << hex << (int)furuno_ephemeris.anti_spoof  << dec <<std::endl <<
		"	cic        : " << furuno_ephemeris.cic 		<< std::endl <<
		"	omega0     : " << furuno_ephemeris.omega0	<< std::endl <<
		"	cis        : " << furuno_ephemeris.cis		<< std::endl <<
		"	i0         : " << furuno_ephemeris.i0		<< std::endl <<
		"	crc        : " << furuno_ephemeris.crc 	    << std::endl <<
		"	w          : " << furuno_ephemeris.w		<< std::endl <<
		"	omegadot   : " << furuno_ephemeris.omegadot << std::endl << 
		"	idot       : " << furuno_ephemeris.idot		<< std::endl;
		OutputFile << hex << 4500 + (float)nav_msg.sat_num << 
			", " << dec << furuno_ephemeris.tow <<
			", " << furuno_ephemeris.w <<
			", " << furuno_ephemeris.crc <<
			", " << furuno_ephemeris.cic <<
			", " << furuno_ephemeris.cis <<	
			", " << "NaN" <<
			", " << "NaN" << std::endl;
		OutputFile << hex << 4200 + (float)nav_msg.sat_num << 
			", " << dec << furuno_ephemeris.i0 << 
			", " << furuno_ephemeris.idot << 
			", " << furuno_ephemeris.omega0 << 
			", " << furuno_ephemeris.omegadot << 
			", " << "NaN" <<
			", " << "NaN" <<
			", " << "NaN" << std::endl;		
			break;
		default:
			break;
	}
	/*	OutputFile << hex << 4000 + (float)nav_msg.sat_num << 
		", " << dec << furuno_ephemeris.tow << 
		", " << furuno_ephemeris.week << 
		", " << furuno_ephemeris.toe << 
		", " << furuno_ephemeris.sqrta << 
		", " << furuno_ephemeris.delta_n << 
		", " << furuno_ephemeris.m0 <<
		", " << furuno_ephemeris.ecc << std::endl;
		OutputFile << hex << 4100 + (float)nav_msg.sat_num << 
		", " << dec << furuno_ephemeris.w << 
		", " << furuno_ephemeris.cuc << 
		", " << furuno_ephemeris.cus << 
		", " << furuno_ephemeris.crc << 
		", " << furuno_ephemeris.crs << 
		", " << furuno_ephemeris.cic <<
		", " << furuno_ephemeris.cis << std::endl;
		OutputFile << hex << 4200 + (float)nav_msg.sat_num << 
		", " << dec << furuno_ephemeris.i0 << 
		", " << furuno_ephemeris.idot << 
		", " << furuno_ephemeris.omega0 << 
		", " << furuno_ephemeris.omegadot << 
		", " << furuno_ephemeris.toc << 
		", " << furuno_ephemeris.tgd <<
		", " << furuno_ephemeris.af0 << std::endl;
		OutputFile << hex << 4300 + (float)nav_msg.sat_num << 
		", " << dec << furuno_ephemeris.af1 << 
		", " << furuno_ephemeris.af2 << 
		", " << hex << (float)furuno_ephemeris.ura << dec << 
		", " << hex << (int)furuno_ephemeris.health << dec << 
		", " << "NaN" << 
		", " << "NaN" <<
		", " << "NaN" <<std::endl;*/
	OutputFile << "9999, NaN, NaN, NaN, NaN, NaN, NaN, NaN" << std::endl;
}
int main(int argc, char **argv){

    if(argc < 3) {
        std::cerr << "Usage: novatel_example <serial port address> <baud rate>" << std::endl;
        return 0;
    }
    std::string port(argv[1]);
    int baudrate=38400;
    istringstream(argv[2]) >> baudrate;

	Furuno my_gps;
	my_gps.on_position = onBestPositionCallback; 
	my_gps.on_nav_message = onNavMessageCallback;
	OutputFile.open("FURUNO_Logfile_07_17_2012_1200AM.txt");
    bool result = my_gps.Connect(port,baudrate);

    if (result) {
        cout << "Successfully connected." << endl;
    }
    else {
        cout << "Failed to connect." << endl;
        return -1;
    }

    while(1);

    my_gps.Disconnect();

	OutputFile.close();

    return 0;
}
class FurunoNode: public CMOOSApp 
{
	private:
	bool OnStartup(int argc, char **argv)	
		{ 	   
			if(argc < 3) 
			{
				std::cerr << "Usage: novatel_example <serial port address> <baud rate>" << std::endl;
				return 0;
			}
			std::string port(argv[1]);
			int baudrate=38400;
			istringstream(argv[2]) >> baudrate;
			Furuno my_furuno;
			my_furuno.Connect(port,baudrate);
			my_furuno.on_position = onBestPositionCallback; 
			&FurunoNode :: PositionCallback;
		}
	void PositionCallback(PositionOutput best_position, double timestamp)
	{
	 bool PublishBestPosToDB(PositionOutput best_position, double timestamp);
	}

Furuno my_furuno;
};