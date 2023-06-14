#include "mymavlink.h"  // 自作MAVLink通信用ヘッダファイルを追加
#include "timer.h"      // 自作タイマー用ヘッダファイルを追加
#include "serialBT.h"

#define LED_PIN A10        // LEDピンを定義（デバッグ用）

// Setup関数
void setup() {
  // シリアル通信（USB）
  Serial.begin(115200);      // シリアル通信（USB）を開始
  pinMode(LED_PIN, OUTPUT);  // LEDピンを出力に設定

  setupTimer();	// タイマー設定用の関数

  // シリアル通信（Pixhaek）
  Serial2.begin(57600);      // シリアル通信（to Pixhawk6c）を開始
  // Serial2.begin(115200);      // シリアル通信（to Pixhawk6c）を開始

  // シリアル通信（Pixhaek）
  SerialBT.begin("ESP32_for_pixhawk");      // シリアル通信（to Pixhawk6c）を開始
  // Serial2.begin(115200);      // シリアル通信（to Pixhawk6c）を開始
}

// Loop関数
void loop() {

  // タイマー割り込みフラグが上がっていたら
  if (Flag_timer > 0 ){
    // Heartbeatの送信
    SendHeartBeat();

    SendCmdServo( 1, 1500);
//    SendCmdServo( 1, 2000);


    // MAV data の要求
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Pixhawk からの stream を要求する
      Mav_Request_Data();
      num_hbs_pasados=0;
    }

    // BT serial 送信
    SerialBT.println("hello!");

    // タイマー割り込みフラグをおろす
  	Flag_timer = 0;
  }

  // メッセージ受信関数
  comm_receive();
}