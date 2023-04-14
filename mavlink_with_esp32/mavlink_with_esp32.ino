#include <mavlink.h>
#include "mytimer.h"    // 自作タイマー用ヘッダファイルを追加
#include "mymavlink.h"  // 自作

#define LED_PIN A10        // LEDピンを定義（デバッグ用）

// Setup関数
void setup() {
  Serial.begin(115200);      // シリアル通信（USB）を開始
  pinMode(LED_PIN, OUTPUT);  // LEDピンを出力に設定

  setupTimer();	// タイマー設定用の関数

  Serial2.begin(57600);      // シリアル通信（to Pixhawk6c）を開始
}

// Loop関数
void loop() {
  /* =========================================================================================================
	 HeartBeat通信用の設定
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

  // リクエスト用のメッセージとバッファの初期化
  mavlink_message_t msg_hb;
  uint8_t buf_hb[MAVLINK_MAX_PACKET_LEN];  // バッファの確保

  // HeartBeat のメッセージを作成
  mavlink_msg_heartbeat_pack(
    sysid,           // System ID
    compid,          // Component ID
    &msg_hb,         // メッセージを格納する変数
    type,            // タイプ
    autopilot_type,  // Autopilotタイプ
    system_mode,     // System mode
    custom_mode,     // Custom mode
    system_state);   // System State

  // 送信用バッファにメッセージの内容をコピー
  uint16_t len_hb = mavlink_msg_to_send_buffer(buf_hb, &msg_hb);
  /* =========================================================================================================
	 HeartBeat通信用の設定（ここまで）
  ========================================================================================================= */

  // // リクエスト用のメッセージとバッファの初期化
  // // Send the message with the standard UART send function
  // // uart0_send might be named differently depending on
  // // the individual microcontroller / library in use.
  // mavlink_message_t msg;  // メッセージの宣言
  // uint8_t buf[MAVLINK_MAX_PACKET_LEN];  // バッファの確保

  // タイマー割り込みフラグが上がっていたら
  if (Flag_timer > 0 ){
    // HeartBeat 通信の送信
    Serial2.write(buf_hb, len_hb);

    // MAV data の要求
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Pixhawk からの stream を要求する
      Mav_Request_Data();
      num_hbs_pasados=0;
    }

    // タイマー割り込みフラグをおろす
  	Flag_timer = 0;
  }

  // メッセージ受信関数
  comm_receive();
}