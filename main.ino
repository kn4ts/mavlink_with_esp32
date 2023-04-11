#define LED_PIN A10        // LEDピンを定義
hw_timer_t* timer = NULL;  // タイマー構造体ポインタを宣言
void IRAM_ATTR onTimer() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // LEDピンをトグルする
}

// Setup関数
void setup() {
  Serial.begin(115200);      // シリアル通信を開始
  pinMode(LED_PIN, OUTPUT);  // LEDピンを出力に設定
  timer = timerBegin(0,      // タイマーID: 0-3
                     80,     // [clock/count]
                     true);  // カウントアップorダウン: true アップ，false ダウン
  timerAttachInterrupt(timer,	// 設定するタイマー
                       &onTimer,// 割り込み関数の指定
                       true);	// 割り込み検知方法: true エッジトリガ， false レベルトリガ
  timerAlarmWrite(timer,	// 設定するタイマー
  		1000000,	// カウント数
		true);		// autoreloadオプション: true 定期実行，false 1ショット実行
  timerAlarmEnable(timer);                // タイマーを有効化
}
// Loop関数
void loop() {
}
