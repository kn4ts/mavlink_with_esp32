// 関数定義
// void Mav_Heartbeat() {
//   // Pack the message

// }

void Mav_Request_Data() {
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
  const int maxStreams = 2;  // ストリームの数の設定
  // ストリームの内訳の設定
//  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_EXTENDED_STATUS } ; //,  // extended status
  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_ALL } ; //,  //
                                           //MAV_DATA_STREAM_EXTRA1 };         // extra1
  // データストリーム毎のデータレート指定[Hz]
  const uint16_t MAVRates[maxStreams] = { 0x01 }; //,    // 2[Hz]
                                         // 0x01 };  // 5[Hz]

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
    mavlink_msg_request_data_stream_pack(
      2,              // System ID
      200,            // Component ID
      &msg,           // メッセージ変数
      1,              // Target System ID
      0,              // Target Component ID
      MAVStreams[i],  // 要求ストリーム ID
      MAVRates[i],    // 要求メッセージ ID
      1);             // 開始・停止？
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    Serial.print("MAV Request data, len = ");
    Serial.println(len);
  }
}

// 受信関数
void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial2.available() > 0) {
    uint8_t c = Serial2.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      Serial.println("a message could be decorded !!!!!!!!");

      // Handle message
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            Serial.println("Switch 1");
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            Serial.println("Switch 2");
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
            Serial.println("Switch 3");
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            Serial.println("Switch 4");
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);

            //if(attitude.roll>1) leds_modo = 0;
            //else if(attitude.roll<-1) leds_modo = 2;
            //else leds_modo=1;
            //Serial.println(attitude) ;
            Serial.println("Switch 5");
          }
          break;

        default:
          {
            Serial.println("No data...");
      	  }
          break;
      }
    }else{
        Serial.println("no message could be decorded...");
    }
  }
}
