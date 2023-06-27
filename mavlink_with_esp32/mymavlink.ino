// Heartbeat メッセージ作成関数
void SendHeartBeat(){
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
  // HeartBeat 通信の送信
  Serial2.write(buf_hb, len_hb);
}

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
//  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_RAW_SENSORS } ; //,  //
  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_EXTRA1 } ; //,  //
                                           //MAV_DATA_STREAM_EXTRA1 };         // extra1
  // データストリーム毎のデータレート指定[Hz]
 const uint16_t MAVRates[maxStreams] = { 0x14 }; //,    // 1[Hz]
  // const uint16_t MAVRates[maxStreams] = { 0x64 }; //,    // 100[Hz]
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
      1, //2,              // System ID
      167, //200,            // Component ID
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

// サーボ出力変更コマンドの送信関数
//  引数  int servo_num ... 対象のサーボ番号
//        int output ... PWM出力（[us]）
//  COMMAND_LONG(#76)メッセージを送信したい
//    コマンドは MAV_CMD_DO_SET_SERVO (183)
//    送信結果は MAV_RESULT で確認できる
void SendCmdServo( int servo_num, int output ){
  mavlink_message_t msg;  // messageの初期化
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];  // 送信用バッファの確保
  
  // コマンドのメッセージを作成する関数
  mavlink_msg_command_long_pack(
    2, //1, //2,
    200, //167, //200,
    &msg,
    1,
    0,
    183, //187,  // 183
    0,
    servo_num, //0.5,
    output, //1200, //0.5,
    0, //0.5,
    0, //0.5,
    0, //0.5,
    0, //0.5,
    0
  );

  // 送信用バッファにメッセージの内容をコピー
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // 送信用バッファの内容を Serial2 に書き込み
  Serial2.write(buf, len);
}

// メッセージ受信関数
//  Serial受信バッファにデータがある限りループする
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
//      sprintf(cbuf, "%2#X ", c);
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
    cnt++ ; // 空の読み込み回数カウントアップ
  }
}

// メッセージ受信関数
void updateSensorValues(uint32_t* ctl,float* roll,float* pitch,float* yaw,float* rollspeed,float* pitchspeed,float* yawspeed) {
  // mavlink_message_t msg;
  // mavlink_status_t statetus;
  // float ct, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed;

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
      char cbuf3[18+1+11] ;
      uint32_t ca = 0;
      uint32_t cb = 0;
      uint32_t cc = 0;
      uint32_t cq = 0;
      ca = c0;
      cb = c1 << 8;
      cc = c2 << 16;
      cq = ca + cb + cc;
      sprintf(cbuf3, "[ %03d, %03d, %03d (%08d) ] ", c0, c1, c2, cq );
      Serial.print(cbuf3);

      // if (cq==116){ 
      uint32_t ct=0;

      if (cq==30){ 
        // uint32_t ct=0;
        // float roll=0;
        int32_t roll32=0;
        int32_t rollf=0;
        int32_t rolle=0;
        // float pitch=0;
        int32_t pitch32=0;
        int32_t pitchf=0;
        int32_t pitche=0;
        // float yaw=0;
        int32_t yaw32=0;
        int32_t yawf=0;
        int32_t yawe=0;
        // float rollspeed=0;
        int32_t rollspeed32=0;
        int32_t rollspeedf=0;
        int32_t rollspeede=0;
        // float pitchspeed=0;
        int32_t pitchspeed32=0;
        int32_t pitchspeedf=0;
        int32_t pitchspeede=0;
        // float yawspeed=0;
        int32_t yawspeed32=0;
        int32_t yawspeedf=0;
        int32_t yawspeede=0;
   
        if ( clen > 0 ){      // Payload の長さが0より大きいとき
          Serial.print("[ "); // かっこはじまり
          int c_ =0;
        
          for ( int i = 0; i<clen ; i++ ){  // Payload の長さ分読み込み・表示ループ
            while( Serial2.available() < 1 );
            c = Serial2.read();
                  
            if (i<4){  //4バイトを合わせて時間の数値を10進数で表す
            ct=ct+(c<<(8*i));

            }else if (i<8){  //roll計算
              if(i==7){ //rollの負の判定
                c_=c>>7;
              }
              
            roll32=roll32+(c<<(8*(i-4))); //rolの1~4つ目の数字を合わせてrollを32bitで出す
            
            }else if (i<12){  //pitch計算
              if(i==11){ //rollの負の判定
                c_=c>>7;
              }
              
            pitch32=pitch32+(c<<(8*(i-8))); //rolの1~4つ目の数字を合わせてrollを32bitで出す
            
            }else if (i<16){  //yaw計算
              if(i==15){ //rollの負の判定
                c_=c>>7;
              }
              
            yaw32=yaw32+(c<<(8*(i-12))); //rolの1~4つ目の数字を合わせてrollを32bitで出す
            
            }else if (i<20){  //rollspeed計算
              if(i==19){ //rollの負の判定
                c_=c>>7;
              }
              
            rollspeed32=rollspeed32+(c<<(8*(i-16))); //rolの1~4つ目の数字を合わせてrollを32bitで出す
            
            }else if (i<24){  //pitchspeed計算
              if(i==23){ //rollの負の判定
                c_=c>>7;
              }
              
            pitchspeed32=pitchspeed32+(c<<(8*(i-20))); //rolの1~4つ目の数字を合わせてrollを32bitで出す
            
            }else if (i<28){  //yawspeed計算
              if(i==27){ //rollの負の判定
                c_=c>>7;
              }
              
            yawspeed32=yawspeed32+(c<<(8*(i-24))); //rolの1~4つ目の数字を合わせてrollを32bitで出す
            
            }

            if(i==4){ //時間数値（10桁）の表示
              char cbuft[14];
              sprintf(cbuft, "(%010d) ", ct);
              Serial.print(cbuft);
              *ctl=ct;
              
            }
            if(i==8){//rollの表示
              char cbufroll[54];

              rollf = roll32 & (0x7FFFFF);
              rolle = roll32 & (0x7FFFFFFF);
              rolle = rolle>>23;

              *roll = pow(-1,c_)*(1+(rollf*pow(2,-23)))*pow(2,rolle-127);
            //   if (c_==1){
            //   cxacc=-(65536)+cxacc; //加速度が負の場合の数値を出す

            // }
              sprintf(cbufroll, "(%.010f) ", *roll);
            
              Serial.print(cbufroll);
              
            }
            if(i==12){//rollの表示
              char cbufpitch[54];

              pitchf = pitch32 & (0x7FFFFF);
              pitche = pitch32 & (0x7FFFFFFF);
              pitche = pitche>>23;

              *pitch = pow(-1,c_)*(1+(pitchf*pow(2,-23)))*pow(2,pitche-127);
            //   if (c_==1){
            //   cxacc=-(65536)+cxacc; //加速度が負の場合の数値を出す

            // }
              sprintf(cbufpitch, "(%.010f) ", *pitch);
            
              Serial.print(cbufpitch);
              
            }
            if(i==16){//rollの表示
              char cbufyaw[54];

              yawf = yaw32 & (0x7FFFFF);
              yawe = yaw32 & (0x7FFFFFFF);
              yawe = yawe>>23;

              *yaw = pow(-1,c_)*(1+(yawf*pow(2,-23)))*pow(2,yawe-127);
            //   if (c_==1){
            //   cxacc=-(65536)+cxacc; //加速度が負の場合の数値を出す

            // }
              sprintf(cbufyaw, "(%.010f) ", *yaw);
            
              Serial.print(cbufyaw);
              
            }
            if(i==20){//rollの表示
              char cbufrollspeed[54];

              rollspeedf = rollspeed32 & (0x7FFFFF);
              rollspeede = rollspeed32 & (0x7FFFFFFF);
              rollspeede = rollspeede>>23;

              *rollspeed = pow(-1,c_)*(1+(rollspeedf*pow(2,-23)))*pow(2,rollspeede-127);
            //   if (c_==1){
            //   cxacc=-(65536)+cxacc; //加速度が負の場合の数値を出す

            // }
              sprintf(cbufrollspeed, "(%.010f) ", *rollspeed);
            
              Serial.print(cbufrollspeed);
              
            }
            if(i==24){//rollの表示
              char cbufpitchspeed[54];

              pitchspeedf = pitchspeed32 & (0x7FFFFF);
              pitchspeede = pitchspeed32 & (0x7FFFFFFF);
              pitchspeede = pitchspeede>>23;

              *pitchspeed = pow(-1,c_)*(1+(pitchspeedf*pow(2,-23)))*pow(2,pitchspeede-127);
            //   if (c_==1){
            //   cxacc=-(65536)+cxacc; //加速度が負の場合の数値を出す

            // }
              sprintf(cbufpitchspeed, "(%.010f) ", *pitchspeed);
            
              Serial.print(cbufpitchspeed);
              
            }
            
            sprintf(cbuf, "%03d ", c);
            Serial.print(cbuf);

            if(i==27){//rollの表示
              char cbufyawspeed[54];

              yawspeedf = yawspeed32 & (0x7FFFFF);
              yawspeede = yawspeed32 & (0x7FFFFFFF);
              yawspeede = yawspeede>>23;

              *yawspeed = pow(-1,c_)*(1+(yawspeedf*pow(2,-23)))*pow(2,yawspeede-127);
            //   if (c_==1){
            //   cxacc=-(65536)+cxacc; //加速度が負の場合の数値を出す

            // }
              sprintf(cbufyawspeed, "(%.010f) ", *yawspeed);
            
              Serial.print(cbufyawspeed);
              
            }

          }
          
          Serial.print("] "); // かっこ終わり
        }
      }
      else {
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
      }

      // 10+clen to 11+clen. checksum を読む
      while( Serial2.available() < 2 );
      c0 = Serial2.read();
      c1 = Serial2.read();
      char cbuf2[13+1] ;
      sprintf(cbuf2, "[ %03d, %03d ] ", c0, c1);
      Serial.print(cbuf2);

      Serial.println(); // 改行
      
      break ; // while文を抜ける
    }
    // Serial.print(c);
    cnt++ ; // 空の読み込み回数カウントアップ
  }
}
