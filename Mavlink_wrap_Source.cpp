/*
Author = Ali Danish
email = alidanish@outlook.de
*/
/*
THIS SOURCE FILE GIVES AN IMPLEMENTATION DEMO ON RECIEVING AND SENDING MAVLINK MESSAGE TO THE CONNECTED ARDUCOPTER SIL.
TWO SIL INSTANCES MUST BE LAUNCHED TO FULLFILL THE DEMO REQUIREMENTS, UNDER FLAG --out <ip>:<port> eg. --out 192.168.12.1:14555.
THIS EXAMPLE CODE USES 2 INSTANCES LISTENING ON 14555, 14556, TO PRESENT A DEMO USAGE OF APIs. INSTANCES THAT ARE INVOLVED IN SENDING
PARAMETER REFRESH REQUEST AND RECIEVING ALL THE PARAMETES/ATTRIBUTES ARE PRESENTED IN THE DEMO. THE EXAMPLE ELABORATES ON USAGE OF 
APIS FROM MAVLINK_PARAMS, MAVLINK_MSGS AND MAV_UDP. INCOMING PARAMETERS POPULATE THE INSTANCE OF mavlink_params, mavlink_params INSTANCE
AFTER RECIEVING, HAS ALL THE VALUES THAT ARE CURRENTLY PRESENT IN COPTER/UAV IN SIL AND HAS ALL THE META DATA RELATED TO THE PARAMETER 
STRING AND VALUE, eg upper_limit, lower_limit, full description, unit/conversion method and enum data AS MAP. MAVLINK_PARAMS CLASS HAS
OPERATOR[] OVERLOADED TO GIVE ACCESS TO THE INTERNAL POPULATED PARAMETERS MAP, THIS GIVES CLIENT CODE LOOP ACCESS TO THE INSTANCE. 
THE MAVLINK_PARAMS CLASS IS BUILT UNDER MULTI READER/WRITER LOCK, CONSEQUENTLY ONLY ONE THREAD (RECIEVING ON UDP THREAD) CAN WRITE 
TO IT WHILE READING FROM MULTIPLE THREADS ARE SAFE. THE MAVLINK_PARAMS CLASS ALSO EXPOSES CONST_ITERATOR, IN ORDER TO USE IT AN OVERLOADED 
param_populate(boost::shared_mutex &o_mutex) THAT TAKES CLIENT CODE DEFINED SHARED MUTEX IN BOOST NAMESPACE, IS PROVIDED.

MAVLINK_MSGS CLASS PROVIDES DECODER FUNCTION MAP THAT DECODES THE INCOMING BYTES TO THEIR RESPECTIVE ATTRIBUTE STRUCTURES. THOSE
MAVLINK BASED MSG STRUCTURES ARE ALSO PART OF MAVLINK_MSGS CLASS. THE CLASS ALSO PROVIDES MAP metamap_msg THAT HAS FEW META DATA
STRUCTURES POPULATED FOR DEMO REASON. MAV_UDP CLASS PROVIDES METHODS TO SET, GET AND REFRESH PARAMETERS. THERE IS A METHOD 
mav_udp_sendto_paramrefresh_init WHICH THIS DEMO USES IN OPTION b and c CAN BE USED IN ANY INTERFACE DURING START UP PHASE
BUT IF INTERFACE REQUIRES REFRESH THEN REFRESH API OF MAV_UDP SHOULD BE USED AS DONE IN OPTION a OF THE DEMO. DEMO USES 6 THREADS,
2 THREADS FOR RECEVING AND DECODING THE MAVLINK MESSAGES FROM 2 INSTANCES OF SIL, 2 THREADS FOR PRINTING MAVLINK MESSAGES AND 
2 THREADS ARE FOR PRINTING PARAMETERS FROM ITERATOR AND OPERATOR [] PROVIDED BY MAVLINK_PARAMS CLASS.

APART FROM DEMO THERE ARE FEW REQUIREMENTS TO MAKE FULL USE OF THE APIs. RECEIVING OF MAVLINK MESSAGES SHOULD BE DONE IN A THREAD
WHERE mav_udp METHOD mav_udp_recvfrom IS USED AND ARE DIFFRENTIATED ON THE BASIS OF THEIR MSG IDS. SUCH A DESIGN IS SUITABLE FOR
FURTHER EXPANSION INTO MAVLINK MISSION COMMANDS WHICH WILL SUPPORT ANY UI BASED ON GOOGLE MAP SERVICES BUILT UNDER VISUAL C++ OR QT.

DEMO INTERFACE : DEMO PUTS COMMAND INTERACTIVE INTERFACE ONCE RUN. TWO INSTANCES OF SIL/UAS MUST ALSO BE RUNNING. ROUTINE CAN TAKE 
ANY ORDER SIL CAN BE LAUNCHED FIRST OR THE DEMO PROGRAM. DEMO USES OPTION 5 OPTIONS WHICH ARE SELF EXPLANATORY. SOMETIMES SIL UNDER
VIRTUAL OS THROW GARABGE SYMBOLS IN PARAMID WHICH IS A STRING WHICH CAUSES OPTION A TO MALFUNCTION BUT 9 OUT 10 TIMES SITUATION DOES 
NOT OCCUR. TO AVOID SUCH A SITUATION QUIT THE DEMO AND RESTART AGAIN, RESTART THE SIL INSTANCES AS WELL. DEMO INPUTS IN OPTION
A(param_read_set) ARE NOT CHECKED FOR ERRORS SO IF GIVEN OPTIONS ARE MISTYPED AND ENTERED THEN RESTART ONLY THE DEMO, SIL INSTANCES 
DOES NOT REQUIRE A RESTART IN THIS CASE. ONCE ANY OPTION b OR c IS STARTED ONE CAN GO BACK TO THE DEMO MENU BY PRESSING 'q', IN THESE
CASES DUE TO THREADS ARE SHARING STDOUT/COUT RESOURCE, LAST STRINGS IN THE STDOUT BUFFER ARE REFLECTED BACK IN TO THE DEMO MENU AS WELL. 
JUST TYPING RANDOM LETTERS AND PRESS ENTER TO GET THE FULL DEMO MENU BACK.

*/


#include "stdafx.h"
#include "mav_udp.h"
#include "mavlink_msgs.h"
#include "mavlink_params.h"
#include <atomic>
#include<conio.h>
//#include"mava2l.h"


mutex mx;
//mutex mx2;
boost::shared_mutex smutex;

condition_variable convar_msgs1;
condition_variable convar_msgs2;
bool read_msgs1 = false;
bool read_msgs2 = false;

condition_variable convar_params1;
condition_variable convar_params2;
bool read_params1 = false;
bool read_params2 = false;

string sexit;
atomic_char cexit;

void mavlink_recv_run1(mavlink_msgs&, mavlink_params &, mav_udp&);
void mavlink_recv_run2(mavlink_msgs&, mavlink_params &, mav_udp&);
void read_run1(mavlink_msgs &);
void read_run2(mavlink_msgs &);
void param_run1(mavlink_params &);
void param_run2(mavlink_params &);
int main(int argc, char *argv[])
{
	
	ofstream paramfile("param.txt"); //checking params return under mavlink_params []
	try 
	{
		boost::asio::io_service os_service;
		
		mavlink_message_t mavlink_msg1,*msg_ptr1, *param_ptr1;
		mavlink_status_t mavlink_status1;
		msg_ptr1 = &mavlink_msg1;  
		param_ptr1 = &mavlink_msg1;
		mav_udp mavlink_udp1(os_service, &mavlink_msg1, &mavlink_status1, 14555); //default port 14555
		mavlink_msgs msgs_instance1(msg_ptr1);
		mavlink_params params_instance1(param_ptr1);

		
		mavlink_message_t mavlink_msg2, *msg_ptr2, *param_ptr2;
		mavlink_status_t mavlink_status2;
		mav_udp mavlink_udp2(os_service, &mavlink_msg2, &mavlink_status2, 14556); //default port 14555
		msg_ptr2 = &mavlink_msg2;
		param_ptr2 = &mavlink_msg2;
		mavlink_msgs msgs_instance2(msg_ptr2);
		mavlink_params params_instance2(param_ptr2);

		/*an essential requirement to read and write request to uas, is to start receiving mavlink message stream
		and decode them into the respective class structure*/
		thread thread_recv_run1(&mavlink_recv_run1, ref(msgs_instance1),ref(params_instance1), ref(mavlink_udp1));
		THREAD t_recv1(thread_recv_run1);
		thread thread_recv_run2(&mavlink_recv_run2, ref(msgs_instance2), ref(params_instance2), ref(mavlink_udp2));
		THREAD t_recv2(thread_recv_run2);
		
		/*threads to read uas attributes in parallel which are decoded under mavlink_recv_run1 and mavlink_recv_run2*/
		thread thread_read_run1(&read_run1, ref(msgs_instance1));
		THREAD t_msg_read1(thread_read_run1);
		thread thread_read_run2(&read_run2, ref(msgs_instance2));
		THREAD t_msg_read2(thread_read_run2);

		/*threads to read parameters in parallel, which uses const_iterator and operator of mavlink_params class*/
		thread thread_param_run1(&param_run1, ref(params_instance1));
		THREAD t_read1(thread_param_run1);
		thread thread_param_run2(&param_run2, ref(params_instance2));
		THREAD t_read2(thread_param_run2);


		auto param_read_set = [&]()
		{

			pair<string, param_data> recv_pair;
			auto params_loop = [&]()
			{
				for (int i = 0; i < params_instance1.params_size(); ++i)
				{

					recv_pair = params_instance1[i];
					//this_thread::sleep_for(400ms);
					cout << " >Reading param @ " << i << " = " << recv_pair.first << " : " << recv_pair.second.param_value <<
						" : index in uav : " << recv_pair.second.param_indexuav << " ";
					if (paramfile.is_open())
					{
						paramfile << " >Reading param @ index " << i << " = " << recv_pair.first << " : " << recv_pair.second.param_value <<
							" : index in uav>> " << recv_pair.second.param_indexuav << " : " << recv_pair.second.conversionmethod << endl;
					}

				}
				paramfile.close();
			};


			string inp_str;
			string setparam;
			string refresh;
			string loop;

			float setvalue;
			float currentvalue;
			int index = 0;
			bool ret;

			while (cexit != 'q')
			{

				this_thread::sleep_for(1200ms);
				cout << "write refresh for parameter refresh, if parameters are not loaded : " << " ";
				getline(cin, inp_str);
				refresh = inp_str;
				if (refresh == "refresh")
				{
					try
					{
						//this_thread::sleep_for(1s);
						mavlink_udp1.mav_udp_sendto_paramrefresh();

						//ret = mav_udp_sendto_paramrefresh_init(mavlink_udp1, params_instance1);

					}
					catch (exception &e)
					{

						cerr << e.what() << endl;
						exit(1);
					}

				}
				
				this_thread::sleep_for(1200ms);
				//cout << "init refresh return status : " << ret<< endl;
				//cout << "refresh count = "<<params_instance1.refresh_count << endl;
				//cout << mavlink_udp1.serv_ep.address() << " : " << mavlink_udp1.serv_ep.port() << endl;
				cout << params_instance1.uas_param_count << ":" << params_instance1.recv_param_count << endl;
				cout << " Write < loop > to loop through the params stored or ignore : " << " ";
				getline(cin, inp_str);
				loop = inp_str;
				if (loop == "loop")
					params_loop();

				cout << "\nEnter the index for mavlink_params instance operator[] access check : ";
				getline(cin, inp_str);
				stringstream(inp_str) >> index;
				cout << endl;
				recv_pair = params_instance1[index];
				cout << " param @ index = " << index << " : " << recv_pair.first << " : " << recv_pair.second.param_value <<
					" index in uav->>> " << recv_pair.second.param_indexuav << " : "
					<< recv_pair.second.longidentifier << " : " << recv_pair.second.conversionmethod
					<< " : " << recv_pair.second.lower_limit << ":" << recv_pair.second.upper_limit << endl;

				if (recv_pair.second.enum_map.size() > 0)
				{
					cout << "recv_pair size : " << recv_pair.second.enum_map.size() << endl;
					for (auto& en : recv_pair.second.enum_map)
						cout << "enums in param : " << "number = " << en.first << " name " << " : " << en.second << endl;
				}
				currentvalue = recv_pair.second.param_value;

				cout << "Enter the paramid as string of 16 chars to set and PRESS ENTER "
					"or write < params > to use params_instance1[index] in the previous stage and PRESS ENTER = " << " : ";
				cin.width(17);
				getline(cin, inp_str);
				setparam = inp_str;

				if (setparam == "params")
					setparam = recv_pair.first;

				cout << "input float value and PRESS ENTER" << " : ";
				getline(cin, inp_str);
				stringstream(inp_str) >> setvalue;

				cout << endl;
				cout << "pair to send : " << setparam << " : " << setvalue;
				cout << endl;
				mavlink_udp1.mav_udp_sendto_paramset(setparam, setvalue, currentvalue);

				//cout << "enter quit to exit " << endl;
				cout << "enter q to demo menu " << endl << "$";
				cexit = getch();
				//getline(cin, inp_str);


			}


		};

		string inpstr;
		string op;
		char rest;
		auto parallel_params = [&]() 
		{
			cexit = 'e'; //change char on entry as 3rd state
			bool ret1, ret2;
			ret1 = mav_udp_sendto_paramrefresh_init(mavlink_udp1, params_instance1);
			ret2 = mav_udp_sendto_paramrefresh_init(mavlink_udp2, params_instance2);
			if (ret1 == true && ret2 == true)
				cout << "PARAMREFRESH_INIT SET STATE..............." << endl;
			else
				cout << "INVALID STAT PARAMREFRESH" << endl;
			convar_params1.notify_one();
			convar_params2.notify_one();
			read_params1 = true;
			read_params2 = true;

			cexit = getch();

			read_params2 = false;
			read_params1 = false;
		};

		auto parallel_msgs = [&]() 
		{

			convar_msgs1.notify_one();
			convar_msgs2.notify_one();
			read_msgs1 = true;
			read_msgs2 = true;
			cexit = getch();
			
			read_msgs1 = false;
			read_msgs2 = false;

		};
		auto pquit = []() {exit(1);};
		//auto fquit = [&]() 
		//{
		//	cexit = op.c_str()[0];
		//};
		typedef map<string, function<void()>> option_map;
		option_map opmap{ { "param_read_set", param_read_set }, {"parallel_params", parallel_params}, {"parallel_msgs", parallel_msgs},
		{"quit", pquit} };
		

		auto demo_option = [&]() 
		{
			cout.flush();
			cout << "DEMO NOTE1: DEMO ASSUMES TWO SIL INSTANCES WILL CONNECT TO THE DEMO. HOWEVER SINGLE INSTANCE STRUCTURES CAN BE READ IN PARALLEL BY AS MANY THREADS AS DESIRED" << endl;
			cout << "DEMO NOTE2: OPTION b and c USED TWO DIFFERENT INTANCES RUNNING ON PORT 14555 AND 14556" << endl;
			cout << "DEMO : CHOOSE OPTION, EVERY OPTION USES SPECIFIC MALVINK WRAP API, OPTION b AND c USES TWO THREADS " << endl
				<< "a. write < param_read_set > to see demo on mavlink_params APIs used for UAS parameters set and read" << endl
				<< "b. write < parallel_params > to see demo on mavlink_params API used for iterative access to the parameters value + meta" << endl
				<< "c. write < parallel_msgs > to see demo on mavlink_msgs API used to decode and print msgs" << endl
				<< "d. write < q > DO NOT PRESS ENTER quit from running state back to demo menu " << endl
				<< "e. write < quit > to quit the demo " << endl;
				
			cout<<"option $";
			getline(cin, inpstr);

			op = inpstr;

		};
		auto findop = [&](string &ipstr)
		{
			auto res = opmap.find(ipstr);
			if (res != opmap.end())
				return true;
			else
				return false;
		};
		while (op != "quit")
		{
			
			cexit = 'r'; //reset char 
			demo_option();
			if (findop(op))
			{
				auto opcall = opmap[op];
				opcall();
				
				
			}
			else
				cout << "invalid option please type exactly as given" << endl;
		}
	}
	catch(std::exception &ex)
	{
		cerr << ex.what() << endl;
	}
	return 0;
}

void mavlink_recv_run1(mavlink_msgs &msgs_instance, mavlink_params &params_instance, mav_udp& mavudp)
{
	bool insert;
	cout << "recv thread started.....  " << endl;
	while (true)
	{
		mavudp.mav_udp_recvfrom();
		//mavudp.mav_udp_readstatus();
		if (msgs_instance.mav_msg->msgid == 30 || msgs_instance.mav_msg->msgid == 74 ||
			msgs_instance.mav_msg->msgid == 24 || msgs_instance.mav_msg->msgid == 0)

		{
			//mavudp.mav_udp_readstatus();
			auto fcall = msgs_instance.decoder_call[msgs_instance.mav_msg->msgid];
			
			fcall();

			//read_1 = true;
			//convar_1.notify_one();
		}

		if (params_instance.mav_msg->msgid == 22)
		{
			params_instance();
			cout << " >paramid : " << params_instance.param_recv->param_id <<" : "<< params_instance.param_recv->param_value<< " ";
	
			insert = params_instance.param_populate();
			cout <<"INSERT STATUS : " <<insert << endl;
			//params_instance.param_populate(smutex);
			//read_1 = true;
			//convar_1.notify_one();

		}


	}
}

void mavlink_recv_run2(mavlink_msgs &msgs_instance, mavlink_params &params_instance, mav_udp& mavudp)
{
	cout << "recv thread 2 started.....  " << endl;
	while (true)
	{
		mavudp.mav_udp_recvfrom();
		
		if (msgs_instance.mav_msg->msgid == 30 || msgs_instance.mav_msg->msgid == 74 ||
			msgs_instance.mav_msg->msgid == 24 || msgs_instance.mav_msg->msgid == 0)

		{

			auto fcall = msgs_instance.decoder_call[msgs_instance.mav_msg->msgid];
			fcall();
		}

		if (params_instance.mav_msg->msgid == 22)
		{
			
			params_instance.decoder_param();
			cout << "paramid : " << params_instance.param_recv->param_id << " : " << params_instance.param_recv->param_value << endl;

			//params_instance.param_populate();
			params_instance.param_populate(smutex);
			//read_2 = true;
			//convar_2.notify_one();

		}


	}
}

void read_run1(mavlink_msgs &msgs_instance1)
{	
	cout << "read thread 1 for msgs started :" << endl;
	while (true)
	{
		std::unique_lock<std::mutex> ulck1(mx);
		convar_msgs1.wait(ulck1, [] {return read_msgs1;});
		
		{
			this_thread::sleep_for(500ms);
			fflush(stdout);
			printf("\rsystem status: %s, basemode : %d \n"
				"pitch : %f, yaw2 : %f\n"
				"groundspeed : %f unit : %s lowerlimit : %e\n"
				"lat : %f lon: %f\n",
				msgs_instance1.metamap_msg["HEARTBEAT.system_status"].enum_map[msgs_instance1.HEARTBEAT.system_status].c_str(),
				msgs_instance1.HEARTBEAT.base_mode, msgs_instance1.ATTITUDE.pitch, msgs_instance1.ATTITUDE.yaw,
				msgs_instance1.VFR_HUD.groundspeed, msgs_instance1.metamap_msg["VFR_HUD.groundspeed"].conversion_method.c_str(),
				atof(msgs_instance1.metamap_msg["VFR_HUD.groundspeed"].lower_limit.c_str()),
				double(msgs_instance1.GPS_RAW_INT.lat) / 10E6, double(msgs_instance1.GPS_RAW_INT.lon) / 10E6);

		}
		ulck1.unlock();
		if (cexit == 'q')
		{
			read_msgs1 = false;
			fflush(stdout);
		}
	}
}

void read_run2(mavlink_msgs &msgs_instance2)
{
	cout << "read thread 2 for msgs started :" << endl;
	while (true)
	{
		std::unique_lock<std::mutex> ulck2(mx);
		convar_msgs2.wait(ulck2, [] {return read_msgs2;});
		
		{
			this_thread::sleep_for(500ms);
			fflush(stdout);
			printf("\rheartbeat status: %s, basemode : %d \n"
				"pitch : %f, yaw : %f\n"
				"groundspeed : %f unit : %s lowerlimit : %e\n"
				"lat : %f lon: %f\n",
				msgs_instance2.metamap_msg["HEARTBEAT.system_status"].enum_map[msgs_instance2.HEARTBEAT.system_status].c_str(),
				msgs_instance2.HEARTBEAT.base_mode, msgs_instance2.ATTITUDE.pitch, msgs_instance2.ATTITUDE.yaw,
				msgs_instance2.VFR_HUD.groundspeed, msgs_instance2.metamap_msg["VFR_HUD.groundspeed"].conversion_method.c_str(),
				atof(msgs_instance2.metamap_msg["VFR_HUD.groundspeed"].lower_limit.c_str()),
				double(msgs_instance2.GPS_RAW_INT.lat) / 10E6, double(msgs_instance2.GPS_RAW_INT.lon) / 10E6);

		}
		ulck2.unlock();
		if (cexit == 'q')
		{
			read_msgs2 = false;
			fflush(stdout);
		}

	}
}

void param_run1(mavlink_params &params_instance)
{
	pair<string, param_data> recv_pair;
	
	cout << "parallel params read thread 1 started : " << endl;
	while (true)
	{
		unique_lock<mutex> ulck(mx);
		convar_params1.wait(ulck, [] {return read_params1;});
		ulck.unlock();
		
		{
			
			cout << "RUN1...WAIT..............." << endl;
			this_thread::sleep_for(2500ms);
			//boost::shared_lock<boost::shared_mutex>slck(smutex);
			//for (auto &read : params_instance)
			//{
			//	cout << "printing map_instance1 READ_RUN1 " << endl;
			//	this_thread::sleep_for(500ms);
			//	cout << read.first << " : " << read.second.param_value << endl;
			//}
			
			
			for (int i = 0; i < params_instance.params_size() && cexit != 'r' && cexit != 'q'; ++i)
			{

				recv_pair = params_instance[i];
				cout << "printing PARAM_RUN1 " << " :: ";
				this_thread::sleep_for(500ms);
				cout << recv_pair.first << " : " << recv_pair.second.param_value << endl;

			}
			//cout << "CEXIT " << cexit << " : " << "READ PARAM1" << read_params1 << endl;
			if (cexit == 'q')
			{
				read_params1 = false;
				cout.flush();
			}
		}
	
	}
		
}


void param_run2(mavlink_params &params_instance)
{
	map<string, param_data>::const_iterator const_iterator;
	
	cout << "parallel params read thread 2 started : " << endl;
	
	while (true)
	{
		unique_lock<mutex> ulck(mx);
		convar_params2.wait(ulck, [] {return read_params2;});
		ulck.unlock();
		{
			cout << "RUN2...WAIT..............." << endl;
			this_thread::sleep_for(2500ms);
			boost::shared_lock<boost::shared_mutex>slck(smutex);
			
			for (const_iterator = params_instance.begin(); const_iterator != params_instance.end() && cexit != 'r' && cexit != 'q'; const_iterator++)
			{

				cout << "printing PARAM_RUN2 " << " :: ";
				this_thread::sleep_for(500ms);
				cout << const_iterator->first << " " << const_iterator->second.param_value << endl;

			}
			//cout << "CEXIT " << cexit << " : " << "READ PARAM2" << read_params2 << endl;
			if (cexit == 'q')
			{
				read_params2 = false;
				cout.flush();
			}
		}

		
	}
}