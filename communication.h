#pragma once
#include <vector>

typedef enum {
	LDS_CONNECT_ERROR = -1,
	LDS_RUNNING_ERROR = -2,
	LDS_POLLUTION_WARNING = -3,
	LDS_POLLUTION_ERROR = -4
}LDS_ERROR_CODE;

int lds_connect();
int lds_disconnect();
int tcp_receive(std::string &recv_buf);
int tcp_send_receive(const char *send_buf, std::string &recv_buf, const int send_len);
int lds_init();
int lds_start();
int lds_stop();
int get_lds_data(std::vector<int> &range, std::vector<int> &energy, int &encoder_pos);
