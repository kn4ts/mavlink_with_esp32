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

  // Serial2.begin(57600);      // シリアル通信（to Pixhawk6c）を開始
  Serial2.begin(115200);      // シリアル通信（to Pixhawk6c）を開始

  // シリアル通信（Bluetooth）
  SerialBT.begin("ESP32_for_pixhawk");      // シリアル通信（to Pixhawk6c）を開始
  // Serial2.begin(115200);      // シリアル通信（to Pixhawk6c）を開始
}

// Loop関数
void loop() {
  // HeartBeat の送信
  SendHeartBeat();
  // データストリーム の要求
  // Mav_Request_Data();

  // 計測値格納用変数の定義
  uint32_t ctl;
  // float roll;
  // float pitch;
  // float yaw;
  // float rollspeed;
  // float pitchspeed;
  // float yawspeed;
  float sensor_values[6];

  // MAVLink メッセージ受信用変数
  int cnt = 0; // byte型の読み込み回数カウンタ
  char buf[280] ; // MAVLink2は最長280[byte]

  // メインループ
  while(1){
    // タイマー割り込みフラグの判定
    if (Flag_timer > 0 ){
      // Heartbeat信号の要求（?）
      // SendHeartBeat();

      // PWM値の計算
      float outputfloat = 1000+(sensor_values[0]+PI)*1000/(2*PI) ;
      //Serial.println(roll);

      // PWM出力
      // SendCmdServo( 1, 1500);
      SendCmdServo( 1, (int)outputfloat);
      SendCmdServo( 2, (int)outputfloat);
      SendCmdServo( 3, (int)outputfloat);
      SendCmdServo( 4, (int)outputfloat);
      SendCmdServo( 5, (int)outputfloat);
      SendCmdServo( 6, (int)outputfloat);

      // メッセージ(#30)の受信内容を表示      
      printSensorValues( &ctl, &sensor_values[0]);

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
    // updateSensorValues(&ctl,&roll,&pitch,&yaw,&rollspeed,&pitchspeed,&yawspeed);
    // BT serial 送信
    SerialBT.println("hello!");

    // タイマー割り込みフラグをおろす
    Flag_timer = 0;
  }
  // メッセージ受信関数
  // comm_receive();
  cnt = receive_message( &buf[0], &ctl, &sensor_values[0], cnt);
  }
}
