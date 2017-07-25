#include "mav_udp.h"
#include "mava2l.h"

using namespace std;
//using namespace boost;
void mav_udp::mav_udp_recvfrom()
{
	
	recv_data = socket.receive_from(boost::asio::buffer(recv_buffer, sizeof(recv_buffer)), serv_ep);
	recv_buffer[recv_data] = 0; //null termination of the mem. array space
	if (recv_data > 0)
	{
		for (int i = 0;i < recv_data; ++i)
		{
			mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(recv_buffer[i]), mav_msg, mav_status);
				//mavlink_msg = *mav_msg;
		}
	}
}

bool mav_udp::mav_udp_sendto_paramset
(string & paramid, float & paramvalue, float & paramvalue_current,uint8_t systemid, uint8_t compid, uint8_t target_sysid, uint8_t target_compid)
{
	//possible asserts check for paramid and param value in the limit
	//check paramid and their respective values before passing to param set pack
	//if the value exceeds the upper and lower bounds set the value which is already in the populated struct
	//garbage cleaning routine for paramid in case paramid string comes from params iterator

	
	//paramid clean routine internal map paramid is not clean
	regex allowstr("[A-Z_0-9]+"); //rejecting garbage symbols
	smatch mstring;
	string paramid_match;
	string *paramid_set;

	regex_search(paramid, mstring, allowstr);
	paramid_match = mstring.str(0);
	//find paramid_set in param_c and also the index that index is used to get bounds
	paramid_set = find(begin(param_c), end(param_c), paramid_match);
	size_t ind_str = distance(begin(param_c), paramid_set);

	//paramvalue bounds checking on param_min and param_max
	if ((paramvalue > param_max[ind_str]) || (paramvalue < param_min[ind_str]))
		paramvalue = paramvalue_current;

	uint16_t rqbytes = mavlink_msg_param_set_pack(systemid, compid, mav_msg, target_sysid, target_compid,
		paramid_set->c_str(), paramvalue, mavlink_msg_param_set_get_param_type(mav_msg));
	int tlen = mavlink_msg_to_send_buffer((uint8_t*)send_buffer, mav_msg);
	
	if(socket.send_to(boost::asio::buffer(send_buffer, sizeof(send_buffer)), serv_ep) == -1)
		return false;
	else
		return true;
}


bool mav_udp::mav_udp_sendto_paramrefresh(uint8_t systemid, uint8_t compid, uint8_t target_sysid, uint8_t target_compid)
{
	
	uint16_t rqbytes = mavlink_msg_param_request_list_pack(systemid, compid, mav_msg, target_sysid, target_compid);
	int tlen = mavlink_msg_to_send_buffer((uint8_t*)send_buffer, mav_msg);

	if (socket.send_to(boost::asio::buffer(send_buffer, sizeof(send_buffer)), serv_ep) == -1)
		return false;
	else
		return true;
}

bool mav_udp::mav_udp_sendto_paramread_byid(string &paramid, uint8_t systemid, uint8_t compid, uint8_t target_sysid, uint8_t target_compid)
{
	uint16_t rqbytes = mavlink_msg_param_request_read_pack(systemid, compid, mav_msg, target_sysid, target_compid, paramid.c_str(), -1);
	int tlen = mavlink_msg_to_send_buffer((uint8_t*)send_buffer, mav_msg);
	if (socket.send_to(boost::asio::buffer(send_buffer, sizeof(send_buffer)), serv_ep) == -1)
		return false;
	else
		return true;
}

bool mav_udp::mav_udp_sendto_paramread_byindex(int index, uint8_t systemid, uint8_t compid, uint8_t target_sysid, uint8_t target_compid)
{
	string dummyparam = "TUNE";
	uint16_t rqbytes = mavlink_msg_param_request_read_pack(systemid, compid, mav_msg, target_sysid, target_compid, dummyparam.c_str(), index);
	int tlen = mavlink_msg_to_send_buffer((uint8_t*)send_buffer, mav_msg);
	if (socket.send_to(boost::asio::buffer(send_buffer, sizeof(send_buffer)), serv_ep) == -1)
		return false;
	else
		return true;
}

void mav_udp::mav_udp_readstatus()
{
	//use in UI to get the mavlink pckts stats
	printf("\rRX_SEQ: %d, TX_SEQ : %d, PCKT_ID: %d, PARSE_STATE: %d, SUCESS_COUNT : %d, DROP_COUNT: %d "
		, mav_status->current_rx_seq, mav_status->current_tx_seq,
		mav_status->packet_idx, mav_status->parse_state, mav_status->packet_rx_success_count,
		mav_status->packet_rx_drop_count);
}


bool mav_udp_sendto_paramrefresh_init
(mav_udp &mavudp, mavlink_params &params_inst, uint8_t systemid, uint8_t compid, uint8_t target_sysid, uint8_t target_compid)
{
	
	//EXIT OR MESSAGE AFTER CERTAIN NUMBER OF TRIES
	while (mavudp.serv_ep.port() == 0)
	{
		cout<<"\rsend address and port are not valid!!," 
			"make sure the SIL/UAS connection is valid and wait 5sec!!! waiting......";
		cout.flush();
		this_thread::sleep_for(5s);
	}

	this_thread::sleep_for(1200ms);
	if (params_inst.recv_param_count >= params_inst.uas_param_count 
		&& params_inst.recv_param_count != 0)
	{
		++params_inst.refresh_count;
		return true;
	}
	this_thread::sleep_for(1200ms);
	if (params_inst.recv_param_count < params_inst.uas_param_count ||
		params_inst.recv_param_count == 0)
	{

		mavudp.mav_udp_sendto_paramrefresh(systemid, compid, target_sysid, target_compid);
		++params_inst.refresh_count;
		return true;
	}

	return false;

		
}
