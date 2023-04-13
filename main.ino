#include <mavlink.h>
#include "mytimer.h"
#include "mymavlink.h"

#define LED_PIN A10        // LEDピンを定義

// Setup関数
void setup() {
  Serial.begin(115200);      // シリアル通信（USB）を開始
  pinMode(LED_PIN, OUTPUT);  // LEDピンを出力に設定

  setupTimer();	// タイマー設定

  Serial2.begin(57600);      // シリアル通信（to Pixhawk6c）を開始

  // Mav_Heartbeat() ;
  // Serial2.write(buf, len);
}

// Loop関数
void loop() {
  /* =========================================================================================================
	MAVLinkの設定
  ========================================================================================================= */
  // MAVLink config
  /* The default UART header for your MCU */
  int sysid = 1;     ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 167;  ///< The component sending the message
  //int compid = 158;               ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;  ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane -> on-board controller
  //uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 0;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];  // バッファの確保

  mavlink_msg_heartbeat_pack(
    sysid,           // System ID
    compid,          // Component ID
    &msg,            // メッセージ変数
    type,            // タイプ
    autopilot_type,  // Autopilotタイプ
    system_mode,     // System mode
    custom_mode,     // Custom mode
    system_state);   // System State

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  
  if (Flag_timer > 0 ){
  	//Mav_Request_Data();
  	Serial2.write(buf, len);
  	Flag_timer = 0;

    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Request streams from Pixhawk
      Mav_Request_Data();
      num_hbs_pasados=0;
    }
  }
 comm_receive();
}