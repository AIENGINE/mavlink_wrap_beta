#pragma once
#ifndef MAVLINK_PARAMS_H
#define MAVLINK_PARAMS_H

#include "stdafx.h"

class E : public std::exception
{
private:
	E() {}; 
	const char * msg; 
public:
	explicit E(const char *s) throw() : msg(s)
	{} 
	const char * what() const throw()
	{
		return msg;
	}

};

struct param_data
{
	float param_value{ 0 };
	string longidentifier{ "NA" };
	string conversionmethod{ "IDENTICAL" };
	string layout{ "Scalar_FLOAT32_IEEE" };
	string type{ "VALUE" };
	map<int, string> enum_map; //query to empty if not then it has enums
	float lower_limit{ 0 };
	float upper_limit{ 0 };
	float increment{ 0 }; //maximum difference
	uint16_t param_indexuav{ 0 }; //index in uav
};

typedef map <string, param_data> params;

extern params param_meta;

class mavlink_params
{
public:
	mavlink_message_t *mav_msg;
	mavlink_param_value_t *param_recv = new mavlink_param_value_t;

private:

	params param_map;
	
	param_data param_mdata;
	
	mutable boost::shared_mutex m_smutex;

	void pre_init();

public:
	int refresh_count{ 0 }; //only init./incremented after a recv_param_count holds true
							//eg. increment 1 on receiving total number of params in the uav
	int uas_param_count{ 0 };
	int recv_param_count{ 0 };

	mavlink_params(mavlink_message_t *mavlink_msg)
		:mav_msg(mavlink_msg)
	{
		pre_init();

	}
	
	~mavlink_params()
	{
		delete param_recv;
	}

	/*
	1. use under thread where mavlink messages are recieved
	2. if (params_instance.mav_msg->msgid == 22)
	{params_instance.decoder_param();}
	*/
	function<void()> decoder_param = [&]()
	{
		mavlink_msg_param_value_decode(mav_msg, param_recv);
	};

	/*
	1. use under thread where mavlink messages are recieved 
	2. if (params_instance.mav_msg->msgid == 22)
		{params_instance();}
	*/ 
	void operator()()
	{
		mavlink_msg_param_value_decode(mav_msg, param_recv);
	}


	typedef params::const_iterator const_iterator;
	typedef params::iterator iterator;

	//use iterators boost::shared_lock under locks
	const_iterator begin()
	{
		return param_map.begin();
	}

	const_iterator end()
	{
		return param_map.end();
	}

	//returns the size of map used inside the clas 
	int params_size()
	{
		boost::shared_lock<boost::shared_mutex> slck(m_smutex);
		return param_map.size();
	}
	
	bool param_populate();

	//1. param_populate version to use with mavlink_params obj's iterator
	//2. requires boost::shared_mutex to be defined under client code
	//3. use boost::shared_lock when using iterator of mavlink_params instance
	bool param_populate(boost::shared_mutex &o_mutex);

	//1. return pair from the map inside the mavlink_params
	//2. string = paramid and param_data has param_value and meta data
	//3. if param_data has enums they can be enums_map, check of not empty 
	pair<string, param_data> operator[](int index);

	//friend bool mav_udp_sendto_paramrefresh_init(mav_udp &, mavlink_params &, uint8_t , uint8_t , uint8_t , uint8_t );

};
//non-member function to populate structures
void param_meta_populate(); 
#endif // !MAVLINK_PARAMS