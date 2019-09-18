#include "communication.h"  
#include "winsock2.h"
#include <iostream>
#include <string>
#pragma comment(lib, "ws2_32.lib")

SOCKET sHost;
const char ip_address[] = "192.168.0.1";

int lds_connect() {
	// init socket lib
	WSADATA wsd;
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {
		std::cout << "WSAStartup failed!" << std::endl;
		return LDS_CONNECT_ERROR;
	}

	// create socket
	sHost = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sHost == INVALID_SOCKET) {
		std::cout << "create socket failed!" << std::endl;
		WSACleanup();
		return LDS_CONNECT_ERROR;
	}

	// set ip address and port
	SOCKADDR_IN servAddr;
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(ip_address);
	servAddr.sin_port = htons(2111);

	// connect server 
	if (connect(sHost, (LPSOCKADDR)&servAddr, sizeof(servAddr)) == SOCKET_ERROR) {
		std::cout << "connect failed!" << std::endl;
		closesocket(sHost);
		WSACleanup();
		return LDS_CONNECT_ERROR;
	}

	return 0;
}

int lds_disconnect() {
	closesocket(sHost);
	WSACleanup();
	return 0;
}

int tcp_receive(std::string &recv_buf) {
	const int RECEIVE_BUF_SIZE = 5000;
	char buf[RECEIVE_BUF_SIZE];
	ZeroMemory(buf, RECEIVE_BUF_SIZE);
	recv(sHost, buf, RECEIVE_BUF_SIZE, 0);
	int i = 1;
	recv_buf.clear();
	recv_buf = std::string(buf);
	return 0;
}

int tcp_send_receive(const char *send_buf, std::string &recv_buf, const int send_len) {
	if (send(sHost, send_buf, send_len, 0) == SOCKET_ERROR) {
		std::cout << "send failed!" << std::endl;
		closesocket(sHost);
		WSACleanup();
		return LDS_CONNECT_ERROR;
	}

	const int RECEIVE_BUF_SIZE = 100;
	char buf[RECEIVE_BUF_SIZE];
	ZeroMemory(buf, RECEIVE_BUF_SIZE);
	recv(sHost, buf, RECEIVE_BUF_SIZE, 0);
	int i = 1;
	recv_buf.clear();
	while (buf[i] != 0x03) {
		recv_buf += buf[i];
		++i;
	}
	return 0;
}

int lds_init() {
	std::string login_recv;
	while (login_recv.size() <= 18 || login_recv[18] != '1') {
		const int login_size = 31;
		char login[login_size] = " sMN SetAccessMode 03 F4724744";
		login[0] = 0x02;
		login[login_size - 1] = 0x03;
		tcp_send_receive(login, login_recv, login_size);
		std::cout << "receive£º" << login_recv << std::endl;
		std::cout << "require: " << "sAN SetAccessMode 1" << std::endl;
		std::cout << std::endl;
	}

	/*
	  Scan frequency
		LMS1xx	25 Hz:	 +2500d (09C4h)
				50 Hz:   +5000d (1388h)
		LMS5xx  25 Hz:   +2500d (09C4h)
				35 Hz:   +3500d (DACh)
				50 Hz:   +5000d (1388h)
				75 Hz:   +7500d (1A0Bh)
				100 Hz:  +10000d (2710h)
	  
	  Number of active sectors (not changeable)

	  Angular resolution
		LMS1xx  0.25¡ã:   +2500d (09C4h)
				0.5¡ã:    +5000d (1388h)
		LMS5xx  0.1667¡ã: +1667d (0683h)
				0.25¡ã:   +2500d (09C4h)
				0.333¡ã:  +3333d (0D05h)
				0.5¡ã:    +5000d (1388h)
				0.667¡ã:  +6667d (1A0Bh)
				1¡ã:      +10000d (2710h)

	  Start angle (not changeable)
	  Stop angle (not changeable)
	*/
	std::string set_freq_recv;
	while (set_freq_recv.size() <= 19 || set_freq_recv[19] != '0') {
		const int set_freq_size = 46;
		char set_freq[set_freq_size] = " sMN mLMPsetscancfg 9C4 1 9C4 FFF92230 225510";
		set_freq[0] = 0x02;
		set_freq[set_freq_size - 1] = 0x03;
		tcp_send_receive(set_freq, set_freq_recv, set_freq_size);
		std::cout << "receive: " << set_freq_recv << std::endl;
		std::cout << "require: " << "sAN mLMPsetscancfg 0 9C4 1 9C4 FFF92230 225510" << std::endl;
		std::cout << std::endl;
	}

	/*
	  Data channel
		LMS1xx  Output channel 1: 01 00
				Output channel 2: 02 00
				Output channel 1+2: 03 00
		LMS5xx  Set via Echo Filter,
				therefore: 00

	  Remission & Angle
		No values: 0
		RSSI: 1
	  
	  Resolution LMS5xx since V1.10, 8 bit only.
		8 Bit: 0
		16 Bit: 1

	  Unit (not changeable)
		Digits: 0

	  Encoder
		No encoder: 00 00
		Channel 1: 01 00

	  Position
		No: 0
		Yes: 1
	  
	  Device name
		No: 0
		Yes: 1

	  Comment
		No: 0
		Yes: 1

	  Time
		No: 0
		Yes: 1

	  Output rate
		All scans: +1d (1h)
		Each 2nd scan: +2d (2h)
		Each 50000th scan: +50000d (C350h)
	*/
	std::string cfg_content_recv;
	const int cfg_content_size = 49;
	char cfg_content[cfg_content_size] = " sWN LMDscandatacfg 01 00 1 1 0 01 00 0 0 0 0 +1";
	cfg_content[0] = 0x02;
	cfg_content[cfg_content_size - 1] = 0x03;
	tcp_send_receive(cfg_content, cfg_content_recv, cfg_content_size);
	std::cout << "receive£º" << cfg_content_recv << std::endl;
	std::cout << "require: " << "sWA LMDscandatacfg" << std::endl;
	std::cout << std::endl;

	/*
	  Status code (not changeable)

	  Angular resolution (not changeable)

	  Start angle
	  Stop angle
		LMS1xx -450000d ¡­ +2250000d (FFF92230h ¡­ 225510h)
		LMS5xx -50000d ¡­ +1850000d (FFFF3CB0h ¡­ 1C3A90h)
	*/
	std::string output_recv;
	const int cfg_output_size = 39;
	char cfg_output[cfg_output_size] = " sWN LMPoutputRange 1 9C4 6DDD0 149970";
	cfg_output[0] = 0x02;
	cfg_output[cfg_output_size - 1] = 0x03;
	tcp_send_receive(cfg_output, output_recv, cfg_output_size);
	std::cout << "receive£º" << output_recv << std::endl;
	std::cout << "require: " << "sWA LMPoutputRange" << std::endl;
	std::cout << std::endl;
	
	std::string store_para_recv;
	while (store_para_recv.size() <= 16 || store_para_recv[16] != '1') {
		const int store_para_size = 17;
		char store_para[store_para_size] = " sMN mEEwriteall";
		store_para[0] = 0x02;
		store_para[store_para_size - 1] = 0x03;
		tcp_send_receive(store_para, store_para_recv, store_para_size);
		std::cout << "receive£º" << store_para_recv << std::endl;
		std::cout << "require: " << "sAN mEEwriteall 1" << std::endl;
		std::cout << std::endl;
	}

	std::string lds_run_recv;
	while (lds_run_recv.size() <= 8 || lds_run_recv[8] != '1') {
		const int lds_run_size = 9;
		char lds_run[lds_run_size] = " sMN Run";
		lds_run[0] = 0x02;
		lds_run[lds_run_size - 1] = 0x03;
		tcp_send_receive(lds_run, lds_run_recv, lds_run_size);
		std::cout << "receive£º" << lds_run_recv << std::endl;
		std::cout << "require: " << "sAN Run 1" << std::endl;
		std::cout << std::endl;
	}

	return 0;
}
	
int lds_start() {
	// Send data permanently
	std::string send_data_recv;
	while (send_data_recv.size() <= 16 || send_data_recv[16] != '1') {
		const int send_data_size = 19;
		char send_data[send_data_size] = " sEN LMDscandata 1";
		send_data[0] = 0x02;
		send_data[send_data_size - 1] = 0x03;
		tcp_send_receive(send_data, send_data_recv, send_data_size);
		std::cout << "receive£º" << send_data_recv << std::endl;
		std::cout << "require: " << "sEA LMDscandata 1" << std::endl;
		std::cout << std::endl;
	}
	//for (int i = 0; i < 1000000; ++i) {
	//	// Poll one telegram
	//	char poll_one[17] = " sRN LMDscandata";
	//	poll_one[0] = 0x02;
	//	poll_one[16] = 0x03;
	//	std::string data_recv;
	//	tcp_send_receive(poll_one, data_recv, 17, 1500);
	//	std::cout << "receive£º" << data_recv << std::endl;
	//}
	return 0;
}

int lds_stop() {
	std::string send_data_recv;
	while (send_data_recv.size() <= 16 || send_data_recv[16] != '0') {
		const int send_data_size = 19;
		char send_data[send_data_size] = " sEN LMDscandata 0";
		send_data[0] = 0x02;
		send_data[send_data_size - 1] = 0x03;
		tcp_send_receive(send_data, send_data_recv, send_data_size);
		std::cout << "receive£º" << send_data_recv << std::endl;
		std::cout << "require: " << "sEA LMDscandata 0" << std::endl;
		std::cout << std::endl;
	}
	return 0;
}

int get_lds_data(std::vector<int> &range, std::vector<int> &energy, int &encoder_pos) {
	/*
		 sSN LMDscandata 1 1 109F5FA 0 0 B5A0 A6D9 69FAC4A1 69FB7AD1 0 0 7 0 0 9C4 168 1 0 3E8 2 
		DIST1 3F800000 00000000 6DDD0 9C4 169 ...
		RSSI1 3F800000 00000000 6DDD0 9C4 169 ...

		Version number: 1
		Device number: 1
		Serial number: 109F5FA
		Device status: 0 0
			Ok: 0 0
			Error: 0 1
			Pollution warning: 0 2
			Pollution error: 0 5
		Telegram counter: 28B6
		Scan counter: 5
		Time since start up in ¦Ìs: 12C7544
		Time of transmission in ¦Ìs: 12D2C0F
		Status of digital inputs: 0 0
			All inputs low: 0 0
			All inputs high: 0 3
		Status of digital outputs: 7 0
			All outputs low: 0 0
			LMS1xx:
				All internal outputs high: 00 07
				All outputs high (inkl.Ext. Out): 07 FF
			LMS5xx:
				All internal outputs high: 00 3F
				All outputs high (inkl.Ext. Out): 3F FF
		former Reserved: 0
		Scan frequency: 9C4 (2500)
		Measurement frequency: 168 (360)
		Amount of encoder: 1
		Encoder position: 0
		Encoder speed: 3E8
		Amount of 16 bit channels: 2
		Content: DIST1 (Distance values of first pulse)
		Scale factor: 3F800000
		Scale factor offset: 00000000
		Start angle: 6DDD0 (450000)
		Size of single angular step: 9C4 (2500)
		Amount of data: 169 (361)
		data: 361 * 12 bit
		Content: RSSI1 (Energy values of first pulse)
		Scale factor: 3F800000
		Scale factor offset: 00000000
		Start angle: 6DDD0 (450000)
		Size of single angular step: 9C4 (2500)
		Amount of data: 169 (361)
		Amount of 8 bit channels: 0
		Position: 0
		Name: 0
		Comment: 0
		Time: 0
		Event info: 0
	*/

	// receive data
	std::string lds_data_recv;
	tcp_receive(lds_data_recv);
	std::cout << "receive data size£º" << lds_data_recv.size() << std::endl;
	// std::cout << lds_data_recv << std::endl;

	// check the device status 
	int data_offset = 1;
	for (int i = 0; i < 5; ++i) {
		data_offset = lds_data_recv.find(' ', data_offset) + 1;
		// std::cout << "data_offset£º" << data_offset << std::endl;
	}
	if (lds_data_recv[data_offset] == '0' || lds_data_recv[data_offset + 2] == '0') {
		std::cout << "Device status ok" << std::endl;
	}
	else if (lds_data_recv[data_offset] == '0' || lds_data_recv[data_offset + 2] == '2') {
		std::cout << "Device status pollution warning" << std::endl;
	}
	else if (lds_data_recv[data_offset] == '0' || lds_data_recv[data_offset + 2] == '5') {
		std::cout << "Device status pollution error" << std::endl;
		return LDS_POLLUTION_ERROR;
	}
	else {
		std::cout << "Device status ERROR" << std::endl;
		return LDS_RUNNING_ERROR;
	}
	data_offset += 4;

	// get the encoder output
	encoder_pos = 0;
	for (int i = 0; i < 11; ++i) {
		data_offset = lds_data_recv.find(' ', data_offset) + 1;
		// std::cout << "data_offset£º" << data_offset << std::endl;
	}
	if (lds_data_recv[data_offset] != '1') {
		std::cout << "No Encoder" << std::endl;
		return -1;
	}
	data_offset += 2;
	while (lds_data_recv[data_offset] != ' ') {
		encoder_pos *= 16;
		if (lds_data_recv[data_offset] >= '0' && lds_data_recv[data_offset] <= '9')
			encoder_pos += lds_data_recv[data_offset] - '0';
		else if (lds_data_recv[data_offset] >= 'A' && lds_data_recv[data_offset] <= 'F')
			encoder_pos += lds_data_recv[data_offset] - 'A' + 10;
		data_offset++;
	}
	data_offset += 1;
	std::cout << "encoder_pos£º" << encoder_pos << std::endl;

	// get data size
	int scan_data_num = 0;
	for (int i = 0; i < 7; ++i) {
		data_offset = lds_data_recv.find(' ', data_offset) + 1;
		// std::cout << "data_offset£º" << data_offset << std::endl;
	}
	while (lds_data_recv[data_offset] != ' ') {
		scan_data_num *= 16;
		if (lds_data_recv[data_offset] >= '0' && lds_data_recv[data_offset] <= '9')
			scan_data_num += lds_data_recv[data_offset] - '0';
		else if (lds_data_recv[data_offset] >= 'A' && lds_data_recv[data_offset] <= 'F')
			scan_data_num += lds_data_recv[data_offset] - 'A' + 10;
		data_offset++;
	}
	std::cout << "scan_data_num£º" << scan_data_num << std::endl;
	
	// get range data
	for (int i = 0; i < scan_data_num; ++i) {
		data_offset = lds_data_recv.find(' ', data_offset) + 1;
		int range_data = 0;
		while (lds_data_recv[data_offset] != ' ') {
			range_data *= 16;
			if (lds_data_recv[data_offset] >= '0' && lds_data_recv[data_offset] <= '9')
				range_data += lds_data_recv[data_offset] - '0';
			else if (lds_data_recv[data_offset] >= 'A' && lds_data_recv[data_offset] <= 'F')
				range_data += lds_data_recv[data_offset] - 'A' + 10;
			data_offset++;
		}
		range.push_back(range_data);
	}

	// get energy data
	for (int i = 0; i < 6; ++i) {
		data_offset = lds_data_recv.find(' ', data_offset) + 1;
		// std::cout << "data_offset£º" << data_offset << std::endl;
	}
	for (int i = 0; i < scan_data_num; ++i) {
		data_offset = lds_data_recv.find(' ', data_offset) + 1;
		int range_data = 0;
		while (lds_data_recv[data_offset] != ' ') {
			range_data *= 16;
			if (lds_data_recv[data_offset] >= '0' && lds_data_recv[data_offset] <= '9')
				range_data += lds_data_recv[data_offset] - '0';
			else if (lds_data_recv[data_offset] >= 'A' && lds_data_recv[data_offset] <= 'F')
				range_data += lds_data_recv[data_offset] - 'A' + 10;
			data_offset++;
		}
		energy.push_back(range_data);
	}
	
	return scan_data_num;
}
