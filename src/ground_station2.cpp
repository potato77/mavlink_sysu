//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>

#include <sysu/mavlink.h>

#include "generic_port.h"
#include "serial_port.h"
#include "udp_port.h"
#include "math_utils.h"
#include "mavlink_aes.c"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>


//  解析签名
mavlink_signing_t signing;
mavlink_signing_streams_t signing_streams;


void print_hex(BYTE str[], int len)
{
	int idx;

	for(idx = 0; idx < len; idx++)
		printf("%02x", str[idx]);
}


using std::string;
using namespace std;

int	system_id    = 0; // system id
int	component_id = 0; // component id
//---------------------------------------gazebo真值相关------------------------------------------
Eigen::Vector3d pos_drone_gazebo;
Eigen::Quaterniond q_gazebo;
Eigen::Vector3d Euler_gazebo;   
geometry_msgs::Point setpoint;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void handle_msg(const mavlink_message_t* msg);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station2");
    ros::NodeHandle nh("~");

    bool use_udp = false;
    // 串口参数
    int baudrate,tx_port, rx_port;
    string uart_name,udp_ip;

    nh.param<bool>("use_udp", use_udp, false);
    nh.param<string>("uart_name", uart_name, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 57600);
    nh.param<string>("udp_ip", udp_ip, "127.0.0.1");
    nh.param<int>("rx_port", rx_port, -1);
    nh.param<int>("tx_port", tx_port, -1);

	const char *uart_name_adr = uart_name.c_str();
	const char *udp_ip_adr = udp_ip.c_str();

	Generic_Port *port;
	if(use_udp)
	{
		port = new UDP_Port(udp_ip_adr, rx_port, tx_port);
	}
	else
	{
		port = new Serial_Port(uart_name_adr, baudrate);
	}

	port->start();

    int passward_num;

    // hash密钥 1
    static const uint8_t secret_sign_key_1[32] = 
    {
        0x42, 0x71, 0xb5, 0xe9,
        0x42, 0x71, 0xb5, 0xe9,
        0x42, 0x71, 0xb5, 0xe9,
        0x42, 0x71, 0xb5, 0xe9,
        0x42, 0x71, 0xb5, 0xe9,
        0x42, 0x71, 0xb5, 0xe9,
        0x42, 0x71, 0xb5, 0xe9,
        0x42, 0x71, 0xb5, 0xe9
    };

    // hash密钥 2
    static const uint8_t secret_sign_key_2[32] = 
    {
        0x42, 0x72, 0xb5, 0xe9,
        0x42, 0x74, 0xb5, 0xe9,
        0x42, 0x75, 0xb5, 0xe9,
        0x43, 0x76, 0xc5, 0xe3,
        0x42, 0x71, 0xb5, 0xe5,
        0x47, 0x71, 0xb5, 0xe9,
        0x41, 0x71, 0x35, 0xe9,
        0x42, 0x71, 0xb5, 0x59
    };

    // hash密钥 3
    static const uint8_t secret_sign_key_3[32] = 
    {
        0x42, 0x72, 0xb5, 0xe9,
        0x43, 0x74, 0xb5, 0xe9,
        0x44, 0x76, 0x15, 0x29,
        0x45, 0x75, 0xc5, 0xe3,
        0x46, 0x74, 0xb5, 0xe5,
        0x47, 0x73, 0xb5, 0xe9,
        0x48, 0x72, 0x35, 0xe9,
        0x42, 0x71, 0xb5, 0x59
    };

    // 频率
    ros::Rate rate(1.0);

    // 读取到的消息
    mavlink_message_t message_received;

//     signing_streams.stream
    
// typedef struct __mavlink_signing_streams {
//     uint16_t num_signing_streams;
//     struct __mavlink_signing_stream {
//         uint8_t link_id;              ///< ID of the link (MAVLINK_CHANNEL)
//         uint8_t sysid;                ///< Remote system ID
//         uint8_t compid;               ///< Remote component ID
//         uint8_t timestamp_bytes[6];   ///< Timestamp, in microseconds since UNIX epoch GMT
//     } stream[MAVLINK_MAX_SIGNING_STREAMS];
// } mavlink_signing_streams_t;
    while(ros::ok())
    {

        passward_num =2;

        // cout << "Please enter 1-3 to choose the passward"<<endl;
        // cin >> passward_num;

		if( passward_num == 1 )
		{
            // 密钥初始化及读取
            memset(&signing, 0, sizeof(signing));
            memcpy(signing.secret_key, secret_sign_key_1, 32);

            //签名stream初始化
            memset(&signing_streams, 0, sizeof(signing_streams));
            signing_streams.num_signing_streams = 0;

        }
        else if ( passward_num == 2 )
        {
            //密钥初始化及读取
            memset(&signing, 0, sizeof(signing));
            memcpy(signing.secret_key, secret_sign_key_2, 32);

            //签名stream初始化
            memset(&signing_streams, 0, sizeof(signing_streams));
            signing_streams.num_signing_streams = 0;
        }
        else
        {
            //密钥初始化及读取
            memset(&signing, 0, sizeof(signing));
            memcpy(signing.secret_key, secret_sign_key_3, 32);

            //签名stream初始化
            memset(&signing_streams, 0, sizeof(signing_streams));
            signing_streams.num_signing_streams = 0;
        }
        


        bool success = port->read_message(message_received);

		if( success )
		{
            handle_msg(&message_received);

        }else
        {
            //printf("No message !!! \n");
        }

        //回调一次 
        //ros::spinOnce();
        usleep(100);
    }

    return 0;

}

void handle_msg(const mavlink_message_t* msg)
{
    // Handle Message ID
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_DN_FXSJ:
        {
            mavlink_dn_fxsj_t dn_fxsj_cipher = { 0 };
            mavlink_dn_fxsj_t dn_fxsj_plain  = { 0 };
            mavlink_msg_dn_fxsj_decode( msg, &dn_fxsj_cipher);

            // 初始化
            //256 bit密码
            BYTE key[1][32] = 
            {
                {0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4}
            };
            WORD key_schedule[60];
            BYTE iv[1][16] = 
            {
                {0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff},
            };
            BYTE ciphertext[1][MAVLINK_MSG_ID_DN_FXSJ_LEN];
            BYTE enc_buf[MAVLINK_MSG_ID_DN_FXSJ_LEN];
            
            // 设置密钥
            aes_key_setup(key[0], key_schedule, 256);
            // 将mavlink payload结构体转为密文
            memcpy(&ciphertext, &dn_fxsj_cipher, MAVLINK_MSG_ID_DN_FXSJ_LEN);
            //解密，此处key_schedule为秘钥
            aes_decrypt_ctr(ciphertext[0], MAVLINK_MSG_ID_DN_FXSJ_LEN, enc_buf, key_schedule, 256, iv[0]);
            // printf("\nCiphertext   : ");
            // print_hex(ciphertext[0], MAVLINK_MSG_ID_DN_FXSJ_LEN);
            // printf("\n-decrypted to: ");
            // print_hex(enc_buf, MAVLINK_MSG_ID_DN_FXSJ_LEN);

            memcpy(&dn_fxsj_plain, &enc_buf, MAVLINK_MSG_ID_DN_FXSJ_LEN);
            //pass = pass && !memcmp(enc_buf, plaintext[0], MAVLINK_MSG_ID_DN_FXSJ_LEN);
            
            printf("Got msg DN_FXSJ \n");
            printf("    magic:          %02X    \n", msg->magic );
            printf("    len:            %02X    \n", msg->len );
            printf("    incompat_flags: %02X    \n", msg->incompat_flags );
            printf("    compat_flags:   %02X    \n", msg->compat_flags );
            printf("    seq:            %02X    \n", msg->seq );
            printf("    sysid:          %02X    \n", msg->sysid );
            printf("    compid:         %02X    \n", msg->compid );
            printf("    msgid:          %02X    \n", msg->msgid );
            printf("    ckecksum:       %02X-%02X    \n", msg->ck[0],msg->ck[1] );
            printf("    link id:        %02X    \n", msg->signature[0] );
            printf("    time stamp:     %02X-%02X-%02X-%02X-%02X-%02X    \n", msg->signature[1], msg->signature[2], msg->signature[3], msg->signature[4], msg->signature[5], msg->signature[6] );
            printf("    signature:      %02X-%02X-%02X-%02X-%02X-%02X    \n", msg->signature[7], msg->signature[8], msg->signature[9], msg->signature[10], msg->signature[11], msg->signature[12] );
            printf("    payload_cipher:   \n" );
            printf("    HEAD, MSG_ID, UAV_ID, LEN, Checksum:   %s  %d %d %d %d      \n", dn_fxsj_cipher.head, dn_fxsj_cipher.msg_id, dn_fxsj_cipher.uav_id, dn_fxsj_cipher.length, dn_fxsj_cipher.checksum );
            printf("    GPS_lat, GPS_lon, GPS_alt:    %d %d %d                      \n", dn_fxsj_cipher.GPS_lat, dn_fxsj_cipher.GPS_lon, dn_fxsj_cipher.GPS_alt );
            printf("    GPS_Vn, GPS_Ve :    %d %d                                   \n", dn_fxsj_cipher.GPS_Vn, dn_fxsj_cipher.GPS_Ve );
            printf("    GPS_num, GPS_time, GPS_sec :    %d %d %d                    \n", dn_fxsj_cipher.GPS_num, dn_fxsj_cipher.GPS_time, dn_fxsj_cipher.GPS_sec );
            printf("    pos  (NED):     %d %d %d (cm)                               \n", dn_fxsj_cipher.x, dn_fxsj_cipher.y, dn_fxsj_cipher.z );
            printf("    vel  (NED):     %d %d %d (cm/s)                             \n", dn_fxsj_cipher.vx, dn_fxsj_cipher.vy, dn_fxsj_cipher.vz );
            printf("    acc  (NED):     %d %d %d (cm/s^2)                           \n", dn_fxsj_cipher.ax, dn_fxsj_cipher.ay, dn_fxsj_cipher.az );
            printf("    angle     :     %d %d %d (degree)                           \n", dn_fxsj_cipher.pitch, dn_fxsj_cipher.roll, dn_fxsj_cipher.yaw );
            printf("    acc_vibe, gyro_vibe :    %d %d                              \n", dn_fxsj_cipher.acc_vibe, dn_fxsj_cipher.gyro_vibe );
            printf("    payload_plain:   \n" );
            printf("    HEAD, MSG_ID, UAV_ID, LEN, Checksum:     %s  %d %d %d %d    \n", dn_fxsj_plain.head, dn_fxsj_plain.msg_id, dn_fxsj_plain.uav_id, dn_fxsj_plain.length, dn_fxsj_plain.checksum );
            printf("    GPS_lat, GPS_lon, GPS_alt:    %d %d %d                      \n", dn_fxsj_plain.GPS_lat, dn_fxsj_plain.GPS_lon, dn_fxsj_plain.GPS_alt );
            printf("    GPS_Vn, GPS_Ve :    %d %d                                   \n", dn_fxsj_plain.GPS_Vn, dn_fxsj_plain.GPS_Ve );
            printf("    GPS_num, GPS_time, GPS_sec :    %d %d %d                    \n", dn_fxsj_plain.GPS_num, dn_fxsj_plain.GPS_time, dn_fxsj_plain.GPS_sec );
            printf("    pos  (NED):     %d %d %d (cm)                               \n", dn_fxsj_plain.x, dn_fxsj_plain.y, dn_fxsj_plain.z );
            printf("    vel  (NED):     %d %d %d (cm/s)                             \n", dn_fxsj_plain.vx, dn_fxsj_plain.vy, dn_fxsj_plain.vz );
            printf("    acc  (NED):     %d %d %d (cm/s^2)                           \n", dn_fxsj_plain.ax, dn_fxsj_plain.ay, dn_fxsj_plain.az );
            printf("    angle     :     %d %d %d (degree)                           \n", dn_fxsj_plain.pitch, dn_fxsj_plain.roll, dn_fxsj_plain.yaw );
            printf("    acc_vibe, gyro_vibe :    %d %d                              \n", dn_fxsj_plain.acc_vibe, dn_fxsj_plain.gyro_vibe );
            printf("\n");

            uint64_t timestamp_test = get_time_msec();
            signing.timestamp = timestamp_test; 

            bool check_signature = mavlink_signature_check(&signing, &signing_streams, msg);

            if(check_signature)
            {
                printf("check_signature pass ! \n");
            }else
            {
                printf("check_signature not pass ! \n");
            }
            

            break;
        }


        case MAVLINK_MSG_ID_CM_ZDFX:
        {
            mavlink_cm_zdfx_t cm_zdfx_cipher = { 0 };
            mavlink_cm_zdfx_t cm_zdfx_plain  = { 0 };
            mavlink_msg_cm_zdfx_decode( msg, &cm_zdfx_cipher);

            // 初始化
            //256 bit密码
            BYTE key[1][32] = 
            {
                {0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4}
            };
            WORD key_schedule[60];
            BYTE iv[1][16] = 
            {
                {0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff},
            };
            BYTE ciphertext[1][MAVLINK_MSG_ID_DN_FXSJ_LEN];
            BYTE enc_buf[MAVLINK_MSG_ID_DN_FXSJ_LEN];
            
            // 设置密钥
            aes_key_setup(key[0], key_schedule, 256);
            // 将mavlink payload结构体转为密文
            memcpy(&ciphertext, &cm_zdfx_cipher, MAVLINK_MSG_ID_DN_FXSJ_LEN);
            //解密
            aes_decrypt_ctr(ciphertext[0], MAVLINK_MSG_ID_DN_FXSJ_LEN, enc_buf, key_schedule, 256, iv[0]);
            // printf("\nCiphertext   : ");
            // print_hex(ciphertext[0], MAVLINK_MSG_ID_DN_FXSJ_LEN);
            // printf("\n-decrypted to: ");
            // print_hex(enc_buf, MAVLINK_MSG_ID_DN_FXSJ_LEN);

            memcpy(&cm_zdfx_plain, &enc_buf, MAVLINK_MSG_ID_DN_FXSJ_LEN);
            //pass = pass && !memcmp(enc_buf, plaintext[0], MAVLINK_MSG_ID_DN_FXSJ_LEN);
            
            printf("Got msg CM_ZDFX \n");
            printf("    magic:          %02X    \n", msg->magic );
            printf("    len:            %02X    \n", msg->len );
            printf("    incompat_flags: %02X    \n", msg->incompat_flags );
            printf("    compat_flags:   %02X    \n", msg->compat_flags );
            printf("    seq:            %02X    \n", msg->seq );
            printf("    sysid:          %02X    \n", msg->sysid );
            printf("    compid:         %02X    \n", msg->compid );
            printf("    msgid:          %02X    \n", msg->msgid );
            printf("    ckecksum:       %02X-%02X    \n", msg->ck[0],msg->ck[1] );
            printf("    link id:        %02X    \n", msg->signature[0] );
            printf("    time stamp:     %02X-%02X-%02X-%02X-%02X-%02X    \n", msg->signature[1], msg->signature[2], msg->signature[3], msg->signature[4], msg->signature[5], msg->signature[6] );
            printf("    signature:      %02X-%02X-%02X-%02X-%02X-%02X    \n", msg->signature[7], msg->signature[8], msg->signature[9], msg->signature[10], msg->signature[11], msg->signature[12] );
            printf("    payload_cipher:   \n" );
            printf("    HEAD, MSG_ID, UAV_ID, LEN, Checksum:   %s  %d %d %d %d      \n", cm_zdfx_cipher.head, cm_zdfx_cipher.msg_id, cm_zdfx_cipher.uav_id, cm_zdfx_cipher.length, cm_zdfx_cipher.checksum );
            printf("    WP_lat, WP_lon, WP_alt:    %d %d %d                      \n", cm_zdfx_cipher.WP_lat, cm_zdfx_cipher.WP_lon, cm_zdfx_cipher.WP_alt );
            printf("    payload_plain:   \n" );
            printf("    HEAD, MSG_ID, UAV_ID, LEN, Checksum:     %s  %d %d %d %d    \n", cm_zdfx_plain.head, cm_zdfx_plain.msg_id, cm_zdfx_plain.uav_id, cm_zdfx_plain.length, cm_zdfx_plain.checksum );
            printf("    WP_lat, WP_lon, WP_alt:    %d %d %d                      \n", cm_zdfx_plain.WP_lat, cm_zdfx_plain.WP_lon, cm_zdfx_plain.WP_alt );
            printf("\n");

            uint64_t timestamp_test = get_time_msec();
            signing.timestamp = timestamp_test; 

            bool check_signature = mavlink_signature_check(&signing, &signing_streams, msg);

            if(check_signature)
            {
                printf("check_signature pass ! \n");
            }else
            {
                printf("check_signature not pass ! \n");
            }
            

            break;
        }
    }
}