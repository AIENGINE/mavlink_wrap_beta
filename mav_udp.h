#pragma once

#ifndef MAV_UDP_H
#define MAV_UDP_H

#include "stdafx.h"
#include "mavlink_params.h"
#define bufsize 1024
class mav_udp
{

private:
	
	boost::asio::ip::udp::socket socket;
	//boost::asio::ip::udp::endpoint ep;
	char recv_buffer[bufsize]; //will be used by asio memory buffer to put the incoming bytes into
	char send_buffer[bufsize];
	size_t recv_data;
	mavlink_message_t *mav_msg; //after receiving bytes are put into mav_msg from there it wil be picked up decoder routine
	mavlink_status_t *mav_status;
	
	
public:
	boost::asio::ip::udp::endpoint serv_ep;

	mav_udp(boost::asio::io_service & service, mavlink_message_t *mavlink_msg, mavlink_status_t *mavlink_st, unsigned short openport = 14555)
		:socket(service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), openport)),
		mav_msg(mavlink_msg), mav_status(mavlink_st)
		
	{
		
		cout << "listening started  = " << openport << endl;
		socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
	}

	//puts incoming/recieved bytes in mavlink_message_t * passed to the mav_udp ctor.
	void mav_udp_recvfrom();
	/*
	1. paramid as string 1st argument, value as float to be set, current value
	2. current value is set if the set value is out of bounds
	3. Defaults 4th arg system_id = 0, 5th arg component_id = 255, 6th target system = 1, target component = 0
	*/
	bool mav_udp_sendto_paramset(string &, float &, float &,uint8_t = 0, uint8_t = 255, uint8_t = 1, uint8_t = 0);
	//no compulsory args to pass, systemid, compid, target_sysid, target_compid can be passed as desired
	bool mav_udp_sendto_paramrefresh(uint8_t = 0, uint8_t = 255, uint8_t = 1, uint8_t = 0);

	bool mav_udp_sendto_paramread_byid(string & , uint8_t = 0, uint8_t = 255, uint8_t = 1, uint8_t = 0);
	bool mav_udp_sendto_paramread_byindex(int , uint8_t = 0, uint8_t = 255, uint8_t = 1, uint8_t = 0);

	void mav_udp_readstatus();

	friend bool mav_udp_sendto_paramrefresh_init(mav_udp &, mavlink_params &, uint8_t , uint8_t , uint8_t , uint8_t );

};
#endif // !MAV_UDP
