void SendData2MATLAB(uint32_t a_1,float a_2,float a_3,float a_4,float a_5,float a_6,float a_7,unsigned long a_8){
  String k = Data2String(a_1,a_2,a_3,a_4,a_5,a_6,a_7,a_8);//送信用に文字列に変換・結合
  SerialBT.println(k);
}

String Data2String(uint32_t a_1,float a_2,float a_3,float a_4,float a_5,float a_6,float a_7,unsigned long a_8){
  String j = String(a_1)+String(",")+String(a_2)+String(",")+String(a_3)+String(",")+String(a_4)+String(",")+String(a_5)+String(",")+String(a_6)+String(",")+String(a_7)+String(",")+String(a_8);
  return j;
}

//BluetoothSerialで1文字受信する関数
void Receive_Mode(char* Mode_Flag){
  while(SerialBT.available()>0){
    *Mode_Flag = SerialBT.read();
  }
}