#include "mavlink_params.h"

using namespace std;

void mavlink_params::pre_init()
{

	param_data deft;
	param_meta_populate();
	params::iterator iter_param_map; //class member map that has to be populated

	
	//iter_param_map = param_meta.begin(); //populated map with all the init. structs + paramid
	
	for (iter_param_map = param_meta.begin(); iter_param_map != param_meta.end(); iter_param_map++)
	{
		param_mdata.longidentifier = iter_param_map->second.longidentifier;
		param_mdata.lower_limit = iter_param_map->second.lower_limit;
		param_mdata.upper_limit = iter_param_map->second.upper_limit;
		param_mdata.conversionmethod = iter_param_map->second.conversionmethod;
		param_mdata.layout = iter_param_map->second.layout;
		param_mdata.type = iter_param_map->second.type;
		param_mdata.increment = iter_param_map->second.increment;
		
		//iter_enum_map = iter_param_map->second.enum_map.begin();
		if (iter_param_map->second.enum_map.size() > 0)
		{
			for (auto& enm : iter_param_map->second.enum_map)
			{
				//param_mdata.enum_map[iter_enum_map->first] = iter_enum_map->second;
				param_mdata.enum_map[enm.first] = enm.second;
			}
		}
		//init. all struct member from param_meta_populate
		//param_map.insert(params::value_type(iter->first, iter->second));
		//param_map[iter_param_map->first] = iter_param_map->second;
		param_map[iter_param_map->first] = param_mdata;
		
	}
	param_mdata = deft; //defaulting out so that copy does not carry out on the rest of the params
}


bool mavlink_params::param_populate()
{
	pair<params::iterator, bool> result;
	//CHECK IF PARAMID IS ALTOGETHER A GARBAGE STRING
	string keystring = param_recv->param_id; 
	param_mdata.param_value = param_recv->param_value;
	param_mdata.param_indexuav = param_recv->param_index;

	unique_lock<boost::shared_mutex> slck(m_smutex);
	result = param_map.insert(params::value_type(keystring, param_mdata));

	uas_param_count = param_recv->param_count;
	recv_param_count++; //number of times params are downloaded

	if (result.second == false)//insert during mav_udp_sendto_paramrefresh
	{

		param_map[keystring].param_value = param_mdata.param_value;
		param_map[keystring].param_indexuav = param_mdata.param_indexuav;
		
		return true;
	}

	if (result.second == true)
		return true;
	else
		
		return false;
}

bool mavlink_params::param_populate(boost::shared_mutex &o_mutex)
{

	pair<params::iterator, bool> result;

	string keystring = param_recv->param_id;
	param_mdata.param_value = param_recv->param_value;
	param_mdata.param_indexuav = param_recv->param_index;
	
	unique_lock<boost::shared_mutex> slck(o_mutex);
	result = param_map.insert(params::value_type(keystring, param_mdata));

	uas_param_count = param_recv->param_count;
	recv_param_count++; //number of times params are downloaded

	if (result.second == false)//insert during mav_udp_sendto_paramrefresh
	{

		param_map[keystring].param_value = param_mdata.param_value;
		param_map[keystring].param_indexuav = param_mdata.param_indexuav;

		return true;
	}

	if (result.second == true)
		return true;
	else

		return false;
}

pair<string, param_data> mavlink_params::operator[](int index)
{
	//paramid clean routine internal map paramid is not clean
	//try catch the regex
	
	regex allowstr("[A-Z_0-9]+"); //rejecting garbage symbols
	smatch mstring;
	string keystring;

	pair<string, param_data> ret_pair;
	params::const_iterator iter = param_map.cbegin();
	boost::shared_lock<boost::shared_mutex> slck(m_smutex);
	advance(iter, index);
	//paramid clean routine
	std::regex_search(iter->first, mstring, allowstr);

	if (mstring.empty() == true)
		throw E("invalid param string !!");
	keystring = mstring.str(0);
	
	ret_pair = make_pair(keystring, iter->second);
	//ret_pair = make_pair(iter->first, iter->second);
	return ret_pair;

	
}
