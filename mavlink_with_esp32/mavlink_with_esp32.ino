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

    // タイマー割り込みフラグをおろす
  	Flag_timer = 0;
  }

  // メッセージ受信関数
  comm_receive();
}