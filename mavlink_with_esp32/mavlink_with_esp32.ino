#include <mavlink.h>
#include "mytimer.h"    // 自作タイマー用ヘッダファイルを追加
#include "mymavlink.h"  // 自作MAVLink通信用ヘッダファイルを追加

#define LED_PIN A10        // LEDピンを定義（デバッグ用）

// Setup関数
void setup() {
  Serial.begin(115200);      // シリアル通信（USB）を開始
  pinMode(LED_PIN, OUTPUT);  // LEDピンを出力に設定

  setupTimer();	// タイマー設定用の関数

  // Serial2.begin(57600);      // シリアル通信（to Pixhawk6c）を開始
  Serial2.begin(115200);      // シリアル通信（to Pixhawk6c）を開始
}

// Loop関数
void loop() {
  // HeartBeat の送信
  SendHeartBeat();
  // データストリーム の要求
  // Mav_Request_Data();

  // 計測値格納用変数の定義
  uint32_t ctl;
  float roll;
  float pitch;
  float yaw;
  float rollspeed;
  float pitchspeed;
  float yawspeed;

  // メインループ
  while(1){
    // タイマー割り込みフラグの判定
    if (Flag_timer > 0 ){
      // Heartbeat信号の要求（?）
      // SendHeartBeat();

      // PWM値の計算
      float outputfloat = 1000+(roll+PI)*1000/(2*PI) ;
      //Serial.println(roll);

      // PWM出力
      // SendCmdServo( 1, 1500);
      SendCmdServo( 1, (int)outputfloat);
      SendCmdServo( 2, (int)outputfloat);
      // SendCmdServo( 3, (int)outputfloat);
      // comm_receive();
      // SendCmdServo( 4, (int)outputfloat);
      // SendCmdServo( 5, (int)outputfloat);
      // SendCmdServo( 6, (int)outputfloat);
  //    SendCmdServo( 1, 2000);
      Serial.println((int)outputfloat);

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

    // センサ値更新
    updateSensorValues(&ctl,&roll,&pitch,&yaw,&rollspeed,&pitchspeed,&yawspeed);

    // メッセージ受信関数
    comm_receive();
  }
}