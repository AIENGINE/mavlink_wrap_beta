/*
Author = Ali Danish
email = alidanish@outlook.de
*/
#pragma once
#include<iostream>
#include<thread>
#include<string>
#include<memory>
#include<mutex>
#include<map>
#include<ardupilotmega\mavlink.h>
#include<ardupilotmega\ardupilotmega.h>
#include<functional>
#include<algorithm>
#include<cstring>
#include<chrono>
#include<regex>
#include<cassert>
#include<fstream>
#include<future>
using namespace std;


#include<boost\asio.hpp>
#include<boost\thread\shared_mutex.hpp>
#include<boost\thread\locks.hpp>
namespace asio = boost::asio;

class mav_udp;
class mavlink_params;
bool mav_udp_sendto_paramrefresh_init
(mav_udp &, mavlink_params &, uint8_t = 0, uint8_t = 255, uint8_t = 1, uint8_t = 0);

class THREAD
{
private:
	thread thr;
public:
	explicit THREAD(thread &t) :thr(move(t))
	{
		if (!thr.joinable())
		{
			cout << thr.get_id() << " : ";
			throw logic_error("THREAD NOT JOINABLE");
		}
	}
	~THREAD()
	{
		thr.join();
	}
};