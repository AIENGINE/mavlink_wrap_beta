#pragma once
#ifndef MAVLINK_MSGS_H
#define MAVLINK_MSGS_H

#include "stdafx.h"

struct msg_data
{
	//all elements of single member of the message set 
	string name;
	string long_identifier;
	string layout;
	string conversion_method;
	string integer_resolution;
	string float_accuracy;
	string lower_limit;
	string upper_limit;
	map<int, string> enum_map;
};
typedef map<string, msg_data> msgmap;
extern msgmap msg_meta;

class mavlink_msgs
{

public:
	//metadata_msg_map to be populated with routine in the ctor.
	//has meta data on uav attributes send in mavlink message
	msgmap metamap_msg;
	mavlink_message_t *mav_msg;
	typedef map<int, function<void()>> decoders;
	
	/*
	usage under recieve mavlink msg thread
	if (mavlink_msg.msgid == 30){
	auto fcall = msgs_instance.decoder_call[msgs_instance.mav_msg->msgid];
	fcall();}
	*/
	decoders decoder_call;
	
	mavlink_msgs(mavlink_message_t *mavlink_msg)
		:mav_msg(mavlink_msg)
	{

		pre_init();

	}



	mavlink_heartbeat_t HEARTBEAT;
	mavlink_attitude_t ATTITUDE;
	mavlink_vfr_hud_t VFR_HUD;
	mavlink_gps_raw_int_t GPS_RAW_INT;

	
private:
	mavlink_msgs() {};
	void pre_init();


	function<void()> decoder_heartbeat = [&]()
	{
		mavlink_msg_heartbeat_decode(mav_msg, &HEARTBEAT);
	};
	

	function<void()> decoder_attitde = [&]()
	{
		mavlink_msg_attitude_decode(mav_msg, &ATTITUDE);
	};
	
	function<void()> decoder_vfr= [&]()
	{
		mavlink_msg_vfr_hud_decode(mav_msg, &VFR_HUD);
	};

	function<void()> decoder_gps_raw_int = [&]()
	{
		mavlink_msg_gps_raw_int_decode(mav_msg, &GPS_RAW_INT);
	};

};
//called in pre_init(), populate msgmap with uav attributes metadata
void msg_meta_populate();


#endif // !MAVLINK_MSGS_H

