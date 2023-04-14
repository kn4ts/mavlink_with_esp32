// メッセージ要求関数
void Mav_Request_Data() {
  // リクエスト用のメッセージとバッファの初期化
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int maxStreams = 1;  // ストリームの数の設定
  // ストリームの内訳の設定
//  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_EXTENDED_STATUS } ; //,  // extended status
  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_RAW_SENSORS } ; //,  //
                                           //MAV_DATA_STREAM_EXTRA1 };         // extra1
  // データストリーム毎のデータレート指定[Hz]
  const uint16_t MAVRates[maxStreams] = { 0x01 }; //,    // 2[Hz]
                                         // 0x01 };  // 5[Hz]

  // ストリームごとの繰り返し文
  for (int i = 0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    // 要求用 メッセージの作成関数
    mavlink_msg_request_data_stream_pack(
      2,              // System ID
      200,            // Component ID
      &msg,           // メッセージを格納する変数
      1,              // Target System ID
      0,              // Target Component ID
      MAVStreams[i],  // 要求ストリーム ID
      MAVRates[i],    // 要求メッセージ ID
      1);             // 開始・停止？
    
    // 送信用バッファにメッセージの内容をコピー
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // 送信用バッファの内容を Serial2 に書き込み
    Serial2.write(buf, len);

    // デバッグ用表示
    Serial.println();
    Serial.print("MAV Request data, len = ");
    Serial.print(len);
    Serial.print(", max len = ");
    Serial.println(MAVLINK_MAX_PACKET_LEN);
  }
}

// メッセージ受信関数
void comm_receive() {

  int cnt = 0; // byte型の読み込み回数カウンタ

  // Serial2の受信バッファにデータがあればループ
  while( Serial2.available() > 0 ){

    uint8_t c = Serial2.read(); // 1文字読み込み

    // 0. 開始点（"253"）の検知
    if( c == 253 ){
      // デバッグ用表示
      // Serial.println(cnt); // 読み込み数カウントの表示
      char cbuf[4] ;
      sprintf(cbuf, "%03d ", c);
      Serial.print(cbuf);

      // 1. len を読む
      while( Serial2.available() < 1 ); // 受信バッファに1文字たまるまで待つ
      uint8_t clen = Serial2.read();    // 1文字読み込み
      sprintf(cbuf, "%03d ", clen);     // 文字列を10進整数として整形
      Serial.print(cbuf);               // デバッグ用表示

      // 2. incompatibility flag を読む
      while( Serial2.available() < 1 );
      c = Serial2.read();
      sprintf(cbuf, "%03d ", c);
      Serial.print(cbuf);

      // 3. compatibility flag を読む
      while( Serial2.available() < 1 );
      c = Serial2.read();
      sprintf(cbuf, "%03d ", c);
      Serial.print(cbuf);

      // 4. packet sequence number を読む
      while( Serial2.available() < 1 );
      c = Serial2.read();
      sprintf(cbuf, "%03d ", c);
      Serial.print(cbuf);

      // 5. System id を読む
      while( Serial2.available() < 1 );
      c = Serial2.read();
      sprintf(cbuf, "%03d ", c);
      Serial.print(cbuf);

      // 6. Component id を読む
      while( Serial2.available() < 1 );
      c = Serial2.read();
      sprintf(cbuf, "%03d ", c);
      Serial.print(cbuf);

      // 7 to 9. Message id を読む
      while( Serial2.available() < 3 );
      uint8_t c0 = Serial2.read();
      uint8_t c1 = Serial2.read();
      uint8_t c2 = Serial2.read();
      char cbuf3[18+1] ;
      sprintf(cbuf3, "[ %03d, %03d, %03d ] ", c0, c1, c2);
      Serial.print(cbuf3);

      // 9+clen. Payload を読む
      if ( clen > 0 ){      // Payload の長さが0より大きいとき
        Serial.print("[ "); // かっこはじまり
        for ( int i = 0; i<clen ; i++ ){  // Payload の長さ分読み込み・表示ループ
          while( Serial2.available() < 1 );
          c = Serial2.read();
          sprintf(cbuf, "%03d ", c);
          Serial.print(cbuf);
        }
        Serial.print("] "); // かっこ終わり
      }

      // 10+clen to 11+clen. checksum を読む
      while( Serial2.available() < 2 );
      c0 = Serial2.read();
      c1 = Serial2.read();
      char cbuf2[13+1] ;
      sprintf(cbuf2, "[ %03d, %03d ] ", c0, c1);
      Serial.print(cbuf2);

      // // 12+clen to 25+clen. checksum を読む
      // while( Serial2.available() < 2 );
      // c0 = Serial2.read();
      // c1 = Serial2.read();
      // char cbuf2[13+1] ;
      // sprintf(cbuf2, "[ %03d, %03d ] ", c0, c1);
      // Serial.print(cbuf2);

      Serial.println(); // 改行
      
      break ; // while文を抜ける
    }
    // Serial.print(c);
    cnt++ ; // 空の読み込み回数カウントアップ
  }
}