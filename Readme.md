# MAVLinkを用いたESP32とPixhawkの接続

## ESP32のインストール
参考1：https://spiceman.jp/esp32-arduino-ide/

## Mission Planner のインストール
1. Mission Planner のインストール  
	参考2： https://ardupilot.org/planner/docs/mission-planner-installation.html
2. ArduPilot用のファームウェアをPixhawkにインストール（Mission Plannerから）

## ArduinoのMAVLINKライブラリの使い方  
参考3：https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566

## MAVLink経由でPixhawkのサーボを動かす
MAVLINKのページ（参考4：https://mavlink.io/en/services/mission.html#mavlink_commands ）によると，Pixhawkに指示を送る際にはコマンド（``Command``）を使用する．
コマンドの種類はいくつかあって，その中でも直ちに実行するものは``DO``コマンドを使用するとのこと．

サンプルコードでは，次の``MAV_LINK_DO_SET_SERVO``コマンド (183番)に着目してMAVLink経由でサーボを動かすことを試みる．  
参考5：https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO  

ではコマンドはどうやって送信するのか．参考4を調べていると，コマンドプロトコルのページ（参考5：https://mavlink.io/en/services/command.html#command-protocol ）に，使用するメッセージの一覧が記載されていた．
メッセージ``COMMAND_INT``と``COMMAND_LONG``がコマンドの格納に使われるようなので，参考3のライブラリで使えそうなパッキング関数を探したところ``mavlink_msg_command_long_pack``が見つかった．

そこで``mavlink_msg_command_long_pack``関数でメッセージを作成して送信する以下の関数を作って実行したところ，Pixhawk6cの``I/O PWM OUT``端子から指令値通りのPWM信号が出力されることを確認した．  
```
// サーボ出力変更コマンドの送信関数
//  引数： int servo_num ... 対象のサーボ番号
//      ： int output ... PWM出力（[us]）
//  COMMAND_LONG(#76)メッセージを送信したい
//    コマンドは MAV_CMD_DO_SET_SERVO (183)
//    送信結果は MAV_RESULT で確認できる
void SendCmdServo( int servo_num, int output ){
  mavlink_message_t msg;  // messageの初期化
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];  // 送信用バッファの確保
  
  // コマンドのメッセージを作成する関数
  mavlink_msg_command_long_pack(
    2,
    200,
    &msg,
    1,
    0,
    183, // MAV_LINK_DO_SET_SERVOコマンドの番号
    0,
    servo_num, // 対象のサーボの番号
    output, // PWM出力 [us]
    0,
    0,
    0,
    0,
    0
  );
  // 送信用バッファにメッセージの内容をコピー
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // 送信用バッファの内容を Serial2 に書き込み
  Serial2.write(buf, len);
}
```

なお，コマンドを送信したのち，Pixhawk側から返ってくるメッセージ``COMMAND_ACK``（#77）（https://mavlink.io/en/messages/common.html#COMMAND_ACK ）にコマンドに対する結果が格納されている．
その中の``MAV_RESULT``（https://mavlink.io/en/messages/common.html#MAV_RESULT ）が``0``なら成功しているとのこと．

## MAVLinkの関連リンク
1. パケットフォーマット：https://mavlink.io/en/guide/serialization.html
2. メッセージタイプ：https://mavlink.io/en/messages/common.html
