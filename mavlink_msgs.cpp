#include"mavlink_msgs.h"

using namespace std;

void mavlink_msgs::pre_init()
{
		
	decoder_call.insert(decoders::value_type(0, decoder_heartbeat));
	decoder_call.insert(decoders::value_type(30, decoder_attitde));
	decoder_call.insert(decoders::value_type(74, decoder_vfr));
	decoder_call.insert(decoders::value_type(24, decoder_gps_raw_int));

	msg_meta_populate();
	for (auto& mdata : msg_meta)
	{
		metamap_msg[mdata.first] = mdata.second;
	}

}

map<int, string> MAV_STATE
{
	{ 0, "MAV_STATE_UNINIT" }, { 1, "MAV_STATE_BOOT"}, { 2, "MAV_STATE_CALIBRATING" }, { 3,"MAV_STATE_STANDBY" },
	{ 4, "MAV_STATE_ACTIVE" }, { 5, "MAV_STATE_CRITICAL" }, { 6, "MAV_STATE_EMERGENCY" }, { 7, "MAV_STATE_POWEROFF" }
};

map<int, string> MAV_MODE_FLAG
{
	{ 128, "MAV_MODE_FLAG_SAFETY_ARMED" }, { 64, "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED" }, { 32, "MAV_MODE_FLAG_HIL_ENABLED" },
	{ 16, "MAV_MODE_FLAG_STABILIZE_ENABLED" }, { 8, "MAV_MODE_FLAG_GUIDED_ENABLED" }, { 4, "MAV_MODE_FLAG_AUTO_ENABLED" },
	{ 2, "MAV_MODE_FLAG_TEST_ENABLED" }, { 1, "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED" }
};


msg_data MT_VFR_HUD_AIRSPEED, MT_VFR_HUD_GROUNDSPEED, MT_HEARTBEAT_BASE_MODE, MT_HEARTBEAT_SYSTEM_STATUS; 
msgmap msg_meta;
void msg_meta_populate()
{
	MT_VFR_HUD_AIRSPEED.name = "VFR_HUD.airspeed";
	MT_VFR_HUD_AIRSPEED.long_identifier = "Current airspeed in m/s";
	MT_VFR_HUD_AIRSPEED.layout = "FLOAT32_IEEE";
	MT_VFR_HUD_AIRSPEED.conversion_method = "METERS_PER_SECOND";
	MT_VFR_HUD_AIRSPEED.integer_resolution = "0";
	MT_VFR_HUD_AIRSPEED.float_accuracy = "0";
	MT_VFR_HUD_AIRSPEED.lower_limit = "-3.40282e+38";
	MT_VFR_HUD_AIRSPEED.upper_limit = "3.40282e+38"; //can use atof or stod 
	msg_meta["VFR_HUD.airspeed"] = MT_VFR_HUD_AIRSPEED;

	MT_VFR_HUD_GROUNDSPEED.name = "VFR_HUD.groundspeed";
	MT_VFR_HUD_GROUNDSPEED.long_identifier = "Current ground speed in m/s";
	MT_VFR_HUD_GROUNDSPEED.layout = "FLOAT32_IEEE";
	MT_VFR_HUD_GROUNDSPEED.conversion_method = "METERS_PER_SECOND";
	MT_VFR_HUD_GROUNDSPEED.integer_resolution = "0";
	MT_VFR_HUD_GROUNDSPEED.float_accuracy = "0";
	MT_VFR_HUD_GROUNDSPEED.lower_limit = "-3.40282e+38";
	MT_VFR_HUD_GROUNDSPEED.upper_limit = "3.40282e+38";
	msg_meta["VFR_HUD.groundspeed"] = MT_VFR_HUD_GROUNDSPEED;

	MT_HEARTBEAT_BASE_MODE.name = "HEARTBEAT.base_mode";
	MT_HEARTBEAT_BASE_MODE.long_identifier = "System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h";
	MT_HEARTBEAT_BASE_MODE.layout = "UBYTE";
	MT_HEARTBEAT_BASE_MODE.conversion_method = "MAV_MODE_FLAG";
	MT_HEARTBEAT_BASE_MODE.integer_resolution = "0";
	MT_HEARTBEAT_BASE_MODE.float_accuracy = "0";
	MT_HEARTBEAT_BASE_MODE.lower_limit = "0"; 
	MT_HEARTBEAT_BASE_MODE.upper_limit = "255";
	MT_HEARTBEAT_BASE_MODE.enum_map = MAV_MODE_FLAG;
	msg_meta["HEARTBEAT.base_mode"] = MT_HEARTBEAT_BASE_MODE;

	MT_HEARTBEAT_SYSTEM_STATUS.name = "HEARTBEAT.system_status";
	MT_HEARTBEAT_SYSTEM_STATUS.long_identifier = "System status flag, see MAV_STATE ENUM";
	MT_HEARTBEAT_SYSTEM_STATUS.layout = "UBYTE";
	MT_HEARTBEAT_SYSTEM_STATUS.conversion_method = "MAV_STATE";
	MT_HEARTBEAT_SYSTEM_STATUS.integer_resolution = "0";
	MT_HEARTBEAT_SYSTEM_STATUS.float_accuracy = "0";
	MT_HEARTBEAT_SYSTEM_STATUS.lower_limit = "0";
	MT_HEARTBEAT_SYSTEM_STATUS.upper_limit = "255";
	MT_HEARTBEAT_SYSTEM_STATUS.enum_map = MAV_STATE;
	msg_meta["HEARTBEAT.system_status"] = MT_HEARTBEAT_SYSTEM_STATUS;
}
