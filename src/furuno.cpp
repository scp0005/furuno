#include "furuno\furuno.h"
#include <iostream>
#include <sstream>
#include <intrin.h>
#include <fstream>
//#include <string_utils/string_utils.h>
using namespace std;
using namespace furuno;



/////////////////////////////////////////////////////
// includes for default time callback
#define WIN32_LEAN_AND_MEAN
#include "boost/date_time/posix_time/posix_time.hpp"

////////////////////////////////////////////////////


/*!
 * Default callback method for timestamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
double DefaultGetTime() {
	boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
	boost::posix_time::time_duration duration(present_time.time_of_day());
	return duration.total_seconds();
}
inline void DefaultDebugMsgCallback(const std::string &msg) {
    std::cout << "Furuno Debug: " << msg << std::endl;
}
inline void DefaultInfoMsgCallback(const std::string &msg) {
    std::cout << "Furuno Info: " << msg << std::endl;
}
inline void DefaultWarningMsgCallback(const std::string &msg) {
    std::cout << "Furuno Warning: " << msg << std::endl;
}
inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cout << "Furuno Error: " << msg << std::endl;
}
void DefaultBestPositionCallback(PositionOutput best_position, double time_stamp){
	/*std::cout << "BESTPOS: \nGPS Week: " << best_position.gps_week <<
		" GPS seconds :" << best_position.gps_time_sec << std::endl <<
		" GPS milliseconds :" << best_position.gps_time_msec << std::endl <<
		"  ECEF X Pos: " << best_position.position_x << std::endl <<
		"  ECEF Y Pos: " << best_position.position_y << std::endl <<
		"  ECEF Z Pos: " << best_position.position_z << std::endl <<
		"  ECEF X Vel: " << best_position.velocity_x << std::endl <<
		"  ECEF Y Vel: " << best_position.velocity_y << std::endl <<
		"  ECEF Z Vel: " << best_position.velocity_z << std::endl <<
		" Number of Satellites: " << hex << (int)best_position.num_of_sats << std::endl <<
		" GPS Status: " << (int)best_position.gps_pos_status << dec << std::endl << std::endl;
		
	for (int i=0;i<14;i++)
	{
		if (best_position.channelOutput[i].prn <= 0.0){
		//std:: cout << std::endl << "CH #: " << i+1 <<std::endl;
		}
		else{
		std:: cout << std::endl << "CH #: " << i+1 <<std::endl <<
		"	Satellite #      :" << hex <<(float)best_position.channelOutput[i].prn			<< std::endl <<
		"	Tracking Mode    :" << (float)best_position.channelOutput[i].tracking_mode		<< std::endl <<
		"	Signal level     :" << (float)best_position.channelOutput[i].carrier_to_noise	<< std::endl <<
		"	# of cycle slips :" << (float)best_position.channelOutput[i].num_cycle_slip << dec << std::endl <<
		"	ADR              :" << best_position.channelOutput[i].carrier_phase	<< std::endl <<
		"	Doppler Frequency:" << best_position.channelOutput[i].doppler		<< std::endl <<
		"	Pseudorange      :" << best_position.channelOutput[i].pseudorange	<< std::endl;
		}
	}*/
}
void DefaultNavMessageCallback(NavMessage nav_msg, GpsEphemeris furuno_ephemeris, double time_stamp){
	/*std:: cout << "Nav Message Callback" << std::endl <<
		"Satellite #" << nav_msg.sat_num << std::endl <<
		"GPS Week : "<< furuno_ephemeris.week << std::endl <<
		"GPS Time Of Week : "<< furuno_ephemeris.tow << std::endl;

	switch (furuno_ephemeris.subframe_id) {
		case 1:

		std:: cout << std::endl << "Subframe ID Check: "  << hex << (int)furuno_ephemeris.subframe_id << dec <<std::endl << 
		"tow 		: " << furuno_ephemeris.tow			<< std::endl <<
		"alert_flag : " << furuno_ephemeris.alert_flag	<< std::endl <<
		"anti_spoof : " << hex << (int)furuno_ephemeris.anti_spoof	<< dec << std::endl <<
		"week 		: " << furuno_ephemeris.week		<< std::endl <<
		"code_on_L2 : " << hex << (int)furuno_ephemeris.code_on_L2	<< dec <<std::endl <<
		"ura 		: " << furuno_ephemeris.ura 	 	<< std::endl <<
		"health 	: " << furuno_ephemeris.health 		<< std::endl <<
		"iodc 		: " << furuno_ephemeris.iodc 		<< std::endl <<
		"L2_P_data_flag	: " << furuno_ephemeris.L2_P_data_flag << std::endl <<
		"tgd 		: " << furuno_ephemeris.tgd 		<< std::endl <<
		"toc 		: " << furuno_ephemeris.toc 		<< std::endl <<
		"af2 		: " << furuno_ephemeris.af2 		<< std::endl <<
		"af1 		: " << furuno_ephemeris.af1 		<< std::endl <<
		"af0 		: " << furuno_ephemeris.af0 		<< std::endl;

		case 2:

		std:: cout << std::endl << "Subframe ID Check: " << hex << (int)furuno_ephemeris.subframe_id << dec << std::endl <<
		"tow 		: " << furuno_ephemeris.tow << std::endl <<
		"alert_flag : " << furuno_ephemeris.alert_flag  << std::endl <<
		"anti_spoof : " << hex << (int)furuno_ephemeris.anti_spoof  << dec <<std::endl <<
		"crs  		: " << furuno_ephemeris.crs 		<< std::endl <<
		"delta_n  	: " << furuno_ephemeris.delta_n 	<< std::endl <<
		"m0  		: " << furuno_ephemeris.m0 			<< std::endl <<
		"cuc  		: " << furuno_ephemeris.cuc 		<< std::endl <<
		"ecc  		: " << furuno_ephemeris.ecc 		<< std::endl <<
		"cus   		: " << furuno_ephemeris.cus  		<< std::endl <<
		"sqrta 		: " << furuno_ephemeris.sqrta 		<< std::endl <<
		"toe  		: " << furuno_ephemeris.toe 		<< std::endl <<
		"fit interval flag  : " << furuno_ephemeris.fit_interval_flag  << std::endl <<
		"age of data offset : " << furuno_ephemeris.age_of_data_offset << std::endl;

		case 3:

		std:: cout << std::endl << "Subframe ID Check: "  << hex << (int)furuno_ephemeris.subframe_id << dec << std::endl <<
		"tow 	    : " << furuno_ephemeris.tow	<< std::endl <<
		"alert_flag : " << furuno_ephemeris.alert_flag  << std::endl <<
		"anti_spoof : " << hex << (int)furuno_ephemeris.anti_spoof  << dec <<std::endl <<
		"cic 	    : "	<< furuno_ephemeris.cic 	 << std::endl <<
		"omega0     : " << furuno_ephemeris.omega0	 << std::endl <<
		"cis 	    : " << furuno_ephemeris.cis		 << std::endl <<
		"i0 	    : " << furuno_ephemeris.i0		 << std::endl <<
		"crc 	    : " << furuno_ephemeris.crc 	 << std::endl <<
		"w			: " << furuno_ephemeris.w		 << std::endl <<
		"omegadot   : " << furuno_ephemeris.omegadot << std::endl <<
		"idot 	    : " << furuno_ephemeris.idot	 << std::endl;

		default:
			break;
	}*/


}
Furuno::Furuno() {
	serial_port_ = NULL;
	reading_status_ = false;
	on_get_time = DefaultGetTime;
	on_position = DefaultBestPositionCallback;
	on_nav_message = DefaultNavMessageCallback;
    on_log_debug = DefaultDebugMsgCallback;
    on_log_info = DefaultInfoMsgCallback;
    on_log_warning = DefaultWarningMsgCallback;
    on_log_error = DefaultErrorMsgCallback;
    buffer_index_ = 0;
    read_timestamp_ = 0;
    parse_timestamp_ = 0;
}

Furuno::~Furuno() {
    Disconnect();
	//data_file_.close;
}
bool Furuno::Connect(std::string port, int baudrate) {
	//serial_port_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(1000));
	serial::Timeout my_timeout(1000,50,0,50,0);
	serial_port_ = new serial::Serial(port,baudrate,my_timeout);

	if (!serial_port_->isOpen()){
        std::stringstream output;
        output << "Serial port: " << port << " failed to open." << std::endl;
        on_log_error(output.str());
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	} else {
        std::stringstream output;
        output << "Serial port: " << port << " opened successfully." << std::endl;
        on_log_info(output.str());
	}
	// stop any incoming data and flush buffers
	serial_port_->write("$PFEC,GPint,GGA00,ZDA00,GSV00,VTG00,tps00\r\n"); // Message to turn off or on NMEA messages (binary always on) see Furuno GT-8032 Protocol Final pdf page 15 
	// wait for data to stop cominig in
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	// clear serial port buffers
	//serial_port_->flush();

	// look for GPS by sending ping and waiting for response
	/*if (!Ping()){
        std::stringstream output;
        output << "Furuno GPS not found on port: " << port << std::endl;
        log_error_(output.str());
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	}*/

	// start reading
	StartReading();
	return true;
}
void Furuno::Disconnect() {
	StopReading();
	serial_port_->close();
	delete serial_port_;
	serial_port_=NULL;
}
/*bool Furuno::ParseVersion(std::string packet) {
	// parse the results - message should start with "#VERSIONA"
        size_t found_version=packet.find("VERSIONA");
		if (found_version==string::npos)
			return false;

		// parse version information
		// remove header
		size_t pos=packet.find(";");
		if (pos==string::npos) {
            log_error_("Error parsing received version."
                       " End of message was not found");
			return false;
		}

		// remove header from message
		std::string message=packet.substr(pos+1, packet.length()-pos-2);
		// parse message body by tokening on ","
		typedef boost::tokenizer<boost::char_separator<char> >
			tokenizer;
		boost::char_separator<char> sep(",");
		tokenizer tokens(message, sep);
		// set up iterator to go through token list
		tokenizer::iterator current_token=tokens.begin();
		string num_comps_string=*(current_token);
		int number_components=atoi(num_comps_string.c_str());
		// make sure the correct number of tokens were found
		int token_count=0;
		for(current_token=tokens.begin(); current_token!=tokens.end();++current_token)
			token_count++;

		// should be 9 tokens, if not something is wrong
		if (token_count!=(8*number_components+1)) {
            log_error_("Error parsing received version. "
                       "Incorrect number of tokens found.");
			return false;
		}

		current_token=tokens.begin();
		// device type is 2nd token
		string device_type=*(++current_token);
		// model is 3rd token
		model_=*(++current_token);
		// serial number is 4th token
		serial_number_=*(++current_token);
		// model is 5rd token
		hardware_version_=*(++current_token);
		// model is 6rd token
		software_version_=*(++current_token);

		// parse the version:
		if (hardware_version_.length()>3)
            protocol_version_=hardware_version_.substr(1,4);
		else
			protocol_version_="UNKNOWN";

		// parse model number:
		// is the receiver capable of raw measurements?
        if (model_.find("L")!=string::npos)
			raw_capable_=true;
		else
			raw_capable_=false;

		// can the receiver receive L2?
		if (model_.find("12")!=string::npos)
			l2_capable_=true;
		else
			l2_capable_=false;

		// can the receiver receive GLONASS?
		if (model_.find("G")!=string::npos)
			glonass_capable_=true;
		else
			glonass_capable_=false;

		// Is this a SPAN unit?
		if ((model_.find("I")!=string::npos)||(model_.find("J")!=string::npos))
			span_capable_=true;
		else
			span_capable_=false;

		// Can the receiver process RTK?
		if (model_.find("R")!=string::npos)
			rtk_capable_=true;
		else
			rtk_capable_=false;


        // fix for oem4 span receivers - do not use l12 notation
        // i think all oem4 spans are l1 l2 capable and raw capable
        if ((protocol_version_=="OEM4")&&(span_capable_)) {
            l2_capable_=true;
            raw_capable_=true;
        }

		return true;

}*/

void Furuno::StartReading() {
	// create thread to read from sensor
	reading_status_=true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread >
		(new boost::thread(boost::bind(&Furuno::ReadSerialPort, this)));
}
void Furuno::StopReading() {
	reading_status_=false;
	// TODO: Consider joining the reading
}
void Furuno::ReadSerialPort() {
	unsigned char buffer[MAX_NOUT_SIZE];
	//char * buffer;
	//buffer = new char [MAX_NOUT_SIZE];
	size_t len;
	//int len;
	// continuously read data from serial port

		while (reading_status_) {
			// read data
			len = serial_port_->read(buffer, MAX_NOUT_SIZE);
			//data_file_.read(buffer, MAX_NOUT_SIZE);
			// timestamp the read
			read_timestamp_ = on_get_time();
			// add data to the buffer to be parsed
			BufferIncomingData(buffer,len);
		}
}

void Furuno::BufferIncomingData(unsigned char *message, unsigned int length)
{
	/*std::cout << "Second Byte: "<<(int)message[1] <<std::endl;*/

	// add incoming data to buffer
	for (unsigned int ii=0; ii<length; ii++)
	{
		// make sure bufIndex is not larger than buffer
		if (buffer_index_>=MAX_NOUT_SIZE)
		{
			buffer_index_=0;
            on_log_warning("Overflowed receive buffer. Buffer cleared.");
		}

		if (buffer_index_==0)
		{	// looking for beginning of message
			if (message[ii]==0x8B)
			{	// beginning of msg found - add to buffer
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_=0;
			}	// end if (msg[ii]
/*			else if (message[ii]=='<')
			{
				// received beginning of acknowledgement
				reading_acknowledgement_=true;
				buffer_index_=1;
			}*/
			else
			{
                on_log_warning("BufferIncomingData::Received unknown data.");
			}
		} // end if (bufIndex==0)
		else if (buffer_index_==1)
		{	// verify 2nd character of header
			if (message[ii]==POS_MSG_ID)
			{	// 2nd byte ok - add to buffer
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_ = POS_MSG_LEN-2; // number of bytes left if message type 1
			}
			/*else if ((message[ii]=='O')&&reading_acknowledgement_)
			{
				// 2nd byte of acknowledgement
				buffer_index_=2;
			}*/
			else if (message[ii]==NAV_MSG_ID)
			{
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_ = NAV_MSG_LEN-2; // number of bytes left if message type 2 (NAV message)
			}
			else
			{
				// start looking for new message again
				buffer_index_=0;
				bytes_remaining_=0;
				reading_acknowledgement_=false;
			} // end if (msg[i]==0x50)
		}	// end else if (bufIndex==1)
		

		else if (bytes_remaining_==1)
		{	// add last byte and parse
			data_buffer_[buffer_index_++] = message[ii];
			unsigned char message_id = data_buffer_[1];
			ParseBinary(data_buffer_,message_id);
			// reset counters
			buffer_index_=0;
			bytes_remaining_=0;
		}  // end else if (bytesRemaining==1)
		else
		{	// add data to buffer
			data_buffer_[buffer_index_++]=message[ii];
			bytes_remaining_--;
		}
	}	// end for
}
void Furuno::ParseBinary(unsigned char *message, unsigned char message_id)
{
    stringstream output;
    output << "Parsing Log: " << hex << message_id << dec << endl;
    on_log_debug(output.str());

    switch (message_id) {
        case POS_MSG_ID:
            Position pos_msg;
			
			/*std::cout << "read data: " << std::endl;
			for (int ii=0; ii<sizeof(pos_msg); ii++)
				std::cout << hex << (int) message[ii] << std::endl;
			std::cout << dec << std::endl;*/
			//boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			//std::cout<< "Debug Msg: " << sizeof(pos_msg)<< std::endl;
            memcpy(&pos_msg, message, sizeof(pos_msg));
			ParsePositionMessage(pos_msg);
            on_position(furuno_position_data_, read_timestamp_);
            break;
        case NAV_MSG_ID:
            NavMessage nav_msg;
            memcpy(&nav_msg, message, sizeof(nav_msg));
			ParseNavMessage(nav_msg);
			on_nav_message(nav_msg, furuno_ephemeris_, read_timestamp_);

            break;
        default:
            break;
    }
}
void Furuno::ParsePositionMessage(Position &position)
{
	unsigned long WeekCount=0;
	
	// Converting byte streams from little endian to big endian;
	position.gps_time_sec = _byteswap_ulong(position.gps_time_sec);
	position.gps_time_msec = _byteswap_ushort(position.gps_time_msec);

	position.position_x = _byteswap_ulong(position.position_x);
	position.position_y = _byteswap_ulong(position.position_y);
	position.position_z = _byteswap_ulong(position.position_z);
	position.velocity_x = _byteswap_ulong(position.velocity_x);
	position.velocity_y = _byteswap_ulong(position.velocity_y);
	position.velocity_z = _byteswap_ulong(position.velocity_z);
	position.sample_offset = _byteswap_ulong(position.sample_offset);

//	furuno_position_data_.gps_week = position.gps_time_sec/604800 ; // seconds in a week (converts seconds from epoch to weeks)
	furuno_position_data_.gps_time_sec = position.gps_time_sec - 1024*7*24*3600;
		
	while (furuno_position_data_.gps_time_sec > 604799) {
		WeekCount=WeekCount+1;
		furuno_position_data_.gps_time_sec = position.gps_time_sec - 1024*604800 - WeekCount*604800;	
	}
	
	furuno_position_data_.gps_week = WeekCount;// since last epoch (add 1024 if counting from first epoch)
	furuno_position_data_.gps_time_msec = position.gps_time_msec;
	furuno_position_data_.num_of_sats = position.num_of_sats;
	furuno_position_data_.gps_pos_status = position.gps_pos_status;

 // multiply the values in the position structure by their respective scale factors
	furuno_position_data_.position_x = position.position_x * pow(2.,-5.);
	furuno_position_data_.position_y = position.position_y * pow(2.,-5.);
	furuno_position_data_.position_z = position.position_z * pow(2.,-5.);
	furuno_position_data_.velocity_x = position.velocity_x * pow(2.,-16.);
	furuno_position_data_.velocity_y = position.velocity_y * pow(2.,-16.);
	furuno_position_data_.velocity_z = position.velocity_z * pow(2.,-16.);
	furuno_position_data_.hdop = position.hdop * pow(10.,-1.);
	furuno_position_data_.vdop = position.vdop * pow(10.,-1.);
	furuno_position_data_.sample_offset = position.sample_offset * pow(2.,-30.);
	
// loop through each of the 14 tracking channels and multiply by scale factors
	for (int i=0;i<14;i++)
	{

		position.channel[i].carrier_phase = _byteswap_ulong(position.channel[i].carrier_phase);
		position.channel[i].doppler = _byteswap_ulong(position.channel[i].doppler);
		position.channel[i].pseudorange = _byteswap_ulong(position.channel[i].pseudorange);

		furuno_position_data_.channelOutput[i].prn = position.channel[i].prn;
		furuno_position_data_.channelOutput[i].tracking_mode = position.channel[i].tracking_mode;
		furuno_position_data_.channelOutput[i].carrier_to_noise = position.channel[i].carrier_to_noise;
		furuno_position_data_.channelOutput[i].num_cycle_slip = position.channel[i].num_cycle_slip;

		furuno_position_data_.channelOutput[i].carrier_phase= position.channel[i].carrier_phase * pow(2.,-8.);
		furuno_position_data_.channelOutput[i].doppler = position.channel[i].doppler * pow(2.,-15.);
		furuno_position_data_.channelOutput[i].pseudorange = position.channel[i].pseudorange * pow(2.,-5.);
	}

}
void Furuno::ParseNavMessage(NavMessage &nav_msg)
{
	unsigned char subframe[30];

	char  s8;
	short s16;
	int   s32;
	unsigned short u16a, u16b;
	unsigned u32a, u32b, u32c, u32d;


	for (int i=0;i<3;i++){

		subframe[i] = nav_msg.nav_word[0][i];
		subframe[i+3] = nav_msg.nav_word[1][i];
		subframe[i+6] = nav_msg.nav_word[2][i];
		subframe[i+9] = nav_msg.nav_word[3][i];
		subframe[i+12] = nav_msg.nav_word[4][i];
		subframe[i+15] = nav_msg.nav_word[5][i];
		subframe[i+18] = nav_msg.nav_word[6][i];
		subframe[i+21] = nav_msg.nav_word[7][i];
		subframe[i+24] = nav_msg.nav_word[8][i];
		subframe[i+27] = nav_msg.nav_word[9][i];

	}
		furuno_ephemeris_.subframe_id=(int)((subframe[5] & 0x1C)>>2);

		std::cout << std::endl << "Subframe ID : "  << hex << (int)furuno_ephemeris_.subframe_id << dec << std::endl;

	switch (furuno_ephemeris_.subframe_id) {
		case 1:
		   // time of week,  actually a 19 bit value, 17 MSBs are available, 2 LSB bits are always zero
		   u32a = subframe[3] << 11;
		   u32b = subframe[4] << 3;
		   u32c = (subframe[5] & 0x80) >> 5;
		   furuno_ephemeris_.tow = (u32a | u32b | u32c); // [z-count 1.5s intervals]
		   furuno_ephemeris_.tow = (furuno_ephemeris_.tow * 3) / 2; // converted to [s]
		  
		   // alert_flag
		   furuno_ephemeris_.alert_flag = (unsigned char)( (subframe[5] & 0x40) >> 6 );
		 
		   // anti-spoof
		   furuno_ephemeris_.anti_spoof = (unsigned char)( (subframe[5] & 0x20) >> 5 );
		 
		   // confirm that this is subframe 1 
		   furuno_ephemeris_.subframe_id = (int)( (subframe[5] & 0x1C) >> 2 );
		   /*if( subframe_id != 1 )
			return FALSE;*/
		  
		   // GPS Week
		   u16a  = (unsigned short)( subframe[6] << 2 );
		   u16b  = (unsigned short)( subframe[7] >> 6 );
		   furuno_ephemeris_.week = (unsigned short)( u16a | u16b );
		   
		   /// code_on_L2
		   furuno_ephemeris_.code_on_L2 = (unsigned char)( (subframe[7] & 0x30) >> 4 );
		 
		   // ura
		   furuno_ephemeris_.ura = (unsigned char)( (subframe[7] & 0x0F) );
		 
		   // health
		   furuno_ephemeris_.health = (unsigned char)( subframe[8] >> 2 );

		   // issue of data clock
		   u16a  = (unsigned short)( (subframe[8] & 0x03) << 8 );
		   u16b  = (unsigned short)( subframe[21] );
		   furuno_ephemeris_.iodc = (unsigned short)( u16a | u16b ); // []
		   
		  // iode subframe for consistency checking
//	   iode_subframe = subframe[21];
		 
		   // L2_P_data_flag 
		   furuno_ephemeris_.L2_P_data_flag = (unsigned char)( (subframe[9] & 0x80) >> 7 );
		 
		   // tgd
		   s8   = subframe[20]; // signed
		   furuno_ephemeris_.tgd = s8 / TWO_TO_THE_POWER_OF_31;
		 
		   // toc
		   u16a = (unsigned short)( subframe[22] << 8 );
		   u16b = (unsigned short)( subframe[23] );
		   furuno_ephemeris_.toc = (unsigned)( (u16a | u16b) ) * 16;  
		   
		   // af2
		   s8  = subframe[24]; // signed
		   furuno_ephemeris_.af2 = s8;
		   furuno_ephemeris_.af2 /= TWO_TO_THE_POWER_OF_55;
		   
		   // af1
		   u16a = (unsigned short)( subframe[25] << 8 );
		   u16b = subframe[26];   
		   s16 = (unsigned short)( u16a | u16b ); // signed value
		   furuno_ephemeris_.af1 = s16;
		   furuno_ephemeris_.af1 /= TWO_TO_THE_POWER_OF_43;
		   
		   // af0
		   u32a = subframe[27] << 24;
		   u32b = subframe[28] << 16;
		   u32c = subframe[29] & 0xFC;
		   u32c <<= 8; // align to the sign bit (two's complement integer)
		   u32d = (u32a | u32b | u32c);
		   s32 = (int)(u32d);
		   s32 >>= 10; // 22 bit value
		   furuno_ephemeris_.af0  = s32;
		   furuno_ephemeris_.af0 /= TWO_TO_THE_POWER_OF_31;
		   break;

		case 2:
		  // confirm that this is subframe 2
		 
		// time of week,  actually a 19 bit value, 17 MSBs are available, 2 LSB bits are always zero
		   u32a = subframe[3] << 11;
		   u32b = subframe[4] << 3;
		   u32c = (subframe[5] & 0x80) >> 5;
		   furuno_ephemeris_.tow = (u32a | u32b | u32c); // [z-count 1.5s intervals]
		   furuno_ephemeris_.tow = (furuno_ephemeris_.tow * 3) / 2; // converted to [s]
		 
		   // alert_flag
		   furuno_ephemeris_.alert_flag = (unsigned char)( (subframe[5] & 0x40) >> 6 );
		 
		   // anti-spoof
		   furuno_ephemeris_.anti_spoof = (unsigned char)( (subframe[5] & 0x20) >> 5 );
		   // iode subframe
//	   iode_subframe = subframe[6];

		   // confirm that this is subframe 2 
		   furuno_ephemeris_.subframe_id = (int)( (subframe[5] & 0x1C) >> 2 );
		   /*if( subframe_id != 2 )
			return FALSE;*/
		 
		   // crs
		   u16a = (unsigned short)( subframe[7] << 8 );
		   u16b = subframe[8];
		   s16  = (unsigned short)( u16a | u16b ); // signed value
		   furuno_ephemeris_.crs = s16;
		   furuno_ephemeris_.crs /= 32.0; // [m]
		 
		   // delta_n
		   u16a = (unsigned short)( subframe[9] << 8 );
		   u16b = subframe[10];  
		   s16  = (short)( u16a | u16b ); // signed value
		   furuno_ephemeris_.delta_n  = s16;
		   furuno_ephemeris_.delta_n *= PI / TWO_TO_THE_POWER_OF_43; // [rad/s]
		 
		   // m0
		   u32a = subframe[11] << 24;
		   u32b = subframe[12] << 16;
		   u32c = subframe[13] << 8;
		   u32d = subframe[14];
		   s32 = (u32a | u32b | u32c | u32d); // signed value
		   furuno_ephemeris_.m0  = s32;
		   furuno_ephemeris_.m0 *= PI / TWO_TO_THE_POWER_OF_31; // [rad]
		 
		   // cuc
		   u16a = (unsigned short)( subframe[15] << 8 );
		   u16b = subframe[16];
		   s16  = (short)( u16a | u16b ); // signed value
		   furuno_ephemeris_.cuc  = s16;
		   furuno_ephemeris_.cuc /= TWO_TO_THE_POWER_OF_29; // [rad]
		   
		   // ecc
		   u32a = subframe[17] << 24;
		   u32b = subframe[18] << 16;
		   u32c = subframe[19] << 8;
		   u32d = subframe[20];
		   furuno_ephemeris_.ecc  = u32a | u32b | u32c | u32d;
		   furuno_ephemeris_.ecc /= TWO_TO_THE_POWER_OF_33;  // []
		 
		   // cus
		   u16a = (unsigned short)( subframe[21] << 8 );
		   u16b = subframe[22];
		   s16  = (short)( u16a | u16b );
		   furuno_ephemeris_.cus  = s16;
		   furuno_ephemeris_.cus /= TWO_TO_THE_POWER_OF_29; // [rad]
		   
		   // sqrta
		   u32a = subframe[23] << 24;
		   u32b = subframe[24] << 16;
		   u32c = subframe[25] << 8;
		   u32d = subframe[26];
		   furuno_ephemeris_.sqrta = u32a | u32b | u32c | u32d; 
		   furuno_ephemeris_.sqrta /= TWO_TO_THE_POWER_OF_19; // [sqrt(m)]
		 
		   // toe
		   u16a = (unsigned short)( subframe[27] << 8 );
		   u16b = subframe[28];
		   furuno_ephemeris_.toe = (unsigned)( (u16a | u16b) ) * 16; // [s]
		 
		   // fit_interval_flag
		   furuno_ephemeris_.fit_interval_flag  = (unsigned char)( subframe[29] >> 7 );
		 
		   // age_of_data_offset
		   furuno_ephemeris_.age_of_data_offset = (unsigned short)( (subframe[29] & 0x74) >> 2 );
		   furuno_ephemeris_.age_of_data_offset *= 900; // [s]
		   break;
		case 3:
		   //------------------------------------------------------------------
		   //                         SUBFRAME3
		   //------------------------------------------------------------------
		 
			// cic
			// time of week,  actually a 19 bit value, 17 MSBs are available, 2 LSB bits are always zero
		   u32a = subframe[3] << 11;
		   u32b = subframe[4] << 3;
		   u32c = (subframe[5] & 0x80) >> 5;
		   furuno_ephemeris_.tow = (u32a | u32b | u32c); // [z-count 1.5s intervals]
		   furuno_ephemeris_.tow = (furuno_ephemeris_.tow * 3) / 2; // converted to [s]
		 
		   // alert_flag
		   furuno_ephemeris_.alert_flag = (unsigned char)( (subframe[5] & 0x40) >> 6 );
		 
		   // anti-spoof
		   furuno_ephemeris_.anti_spoof = (unsigned char)( (subframe[5] & 0x20) >> 5 );

		   // confirm that this is subframe 3 
		   furuno_ephemeris_.subframe_id = (int)( (subframe[5] & 0x1C) >> 2 );
		   /*if( subframe_id != 3 )
			return FALSE;*/

		   u16a  = (unsigned short)( subframe[6] << 8 );
		   u16b  = subframe[7];
		   s16   = (short)( u16a | u16b ); // signed value
		   furuno_ephemeris_.cic  = s16;
		   furuno_ephemeris_.cic /= TWO_TO_THE_POWER_OF_29; // [rad]
		 
		   // omego0
		   u32a = subframe[8] << 24;
		   u32b = subframe[9] << 16;
		   u32c = subframe[10] << 8;
		   u32d = subframe[11];
		   s32  = u32a | u32b | u32c | u32d; // signed value
		   furuno_ephemeris_.omega0  = s32;
		   furuno_ephemeris_.omega0 *= PI / TWO_TO_THE_POWER_OF_31; // [rad]
		 
		   // cis
		   u16a  = (unsigned short)( subframe[12] << 8 );
		   u16b  = subframe[13];
		   s16   = (short)( u16a | u16b ); // signed value
		   furuno_ephemeris_.cis  = s16;
		  furuno_ephemeris_.cis /= TWO_TO_THE_POWER_OF_29; // [rad]
		 
		   // i0
		   u32a = subframe[14] << 24;
		   u32b = subframe[15] << 16;
		   u32c = subframe[16] << 8;
		   u32d = subframe[17];
		   s32  = u32a | u32b | u32c | u32d;
		   furuno_ephemeris_.i0  = s32;
		   furuno_ephemeris_.i0 *= PI / TWO_TO_THE_POWER_OF_31; // [rad]
		   
		   // crc
		   u16a  = (unsigned short)( subframe[18] << 8 );
		   u16b  = subframe[19];
		   s16   = (short)( u16a | u16b ); // signed value
		   furuno_ephemeris_.crc  = s16;
		   furuno_ephemeris_.crc /= 32.0; // [m]
		 
		   // w
		   u32a = subframe[20] << 24;
		   u32b = subframe[21] << 16;
		   u32c = subframe[22] << 8;
		   u32d = subframe[23];
		   s32  = u32a | u32b | u32c | u32d; // signed value
		   furuno_ephemeris_.w   = s32;
		   furuno_ephemeris_.w  *= PI / TWO_TO_THE_POWER_OF_31; // [rad]
		 
		   // omegadot
		   u32a = subframe[24] << 24;
		   u32b = subframe[25] << 16;
		   u32c = subframe[26] << 8;  
		   s32  = u32a | u32b | u32c; // signed value
		   s32  = s32 >> 8;
		   furuno_ephemeris_.omegadot  = s32;
		   furuno_ephemeris_.omegadot *= PI / TWO_TO_THE_POWER_OF_43; // [rad/s]
		 
		   // iode subframe
//	   iode_subframe = subframe[27];
		 
		   // idot
		   u16a  = (unsigned short)( subframe[28] << 8 );
		   u16b  = (unsigned short)( subframe[29] & 0xFC );
		   s16   = (short)( u16a | u16b ); // signed value
		   s16   = (short)( s16 >> 2 );
		   furuno_ephemeris_.idot = s16;
		   furuno_ephemeris_.idot *= PI / TWO_TO_THE_POWER_OF_43; // [rad/s]  
		break;
		   
		default:
			break;
	}
}