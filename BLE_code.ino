#include <Adafruit_TinyUSB.h>   // USB serial
#include <bluefruit.h>          // BLE stack (Seeed nRF52 core)
#include <Wire.h>               // I2C

// I2C slave address (7-bit)
#define XIAO_I2C_ADDR  0x28

// BLE UUIDs
BLEService        sensorService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic sensorChar   ("12345678-1234-5678-1234-56789abcdef1");

// latest decoded values (from STM32 over I2C)
volatile float g_temp = 23.45f;
volatile float g_hum  = 45.67f;
volatile float g_dist = 143.2f;
volatile bool  g_newData = false;

// -------- I2C receive callback: 7-byte packet --------
void onI2CReceive(int numBytes)
{
  if (numBytes != 7) {
    // wrong length, flush
    while (Wire.available()) Wire.read();
    return;
  }

  uint8_t buf[7];
  for (int i = 0; i < 7; i++) {
    buf[i] = Wire.read();
  }

  if (buf[0] != 0x55) {
    // bad header
    return;
  }

  int16_t t_x100 = (int16_t)((buf[1] << 8) | buf[2]);   // big-endian
  int16_t h_x100 = (int16_t)((buf[3] << 8) | buf[4]);
  int16_t d_x10  = (int16_t)((buf[5] << 8) | buf[6]);

  g_temp = t_x100 / 100.0f;
  g_hum  = h_x100 / 100.0f;
  g_dist = d_x10  / 10.0f;

  g_newData = true;
}

// -------- BLE advertising helper --------
void startAdvertising()
{
  Bluefruit.Advertising.stop();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(sensorService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // 20–152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);

  Bluefruit.Advertising.start(0);  // forever
}

// ---------------- setup ----------------
void setup()
{
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) { }

  Serial.println("XIAO BLE + I2C slave bridge starting...");

  // I2C slave
  Wire.begin(XIAO_I2C_ADDR);      // act as slave @ 0x28
  Wire.onReceive(onI2CReceive);   // callback when STM32 sends data
  Serial.println("I2C slave ready on 0x28");

  // BLE init
  Bluefruit.begin();
  Bluefruit.setName("XIAO-BRIDGE");
  Bluefruit.setTxPower(4);

  sensorService.begin();

  sensorChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  sensorChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensorChar.setMaxLen(32);
  sensorChar.setFixedLen(0);
  sensorChar.begin();

  // initial BLE value
  char s[32];
  snprintf(s, sizeof(s), "T=%.2f,H=%.2f,D=%.1f", g_temp, g_hum, g_dist);
  sensorChar.write((uint8_t*)s, strlen(s));

  startAdvertising();

  Serial.println("BLE advertising as 'XIAO-BRIDGE'");
}

// ---------------- loop ----------------
void loop()
{
  if (g_newData) {
    // copy volatile values atomically
    noInterrupts();
    float t = g_temp;
    float h = g_hum;
    float d = g_dist;
    g_newData = false;
    interrupts();

    char s[32];
    snprintf(s, sizeof(s), "T=%.2f,H=%.2f,D=%.1f", t, h, d);

    sensorChar.write((uint8_t*)s, strlen(s));
    if (Bluefruit.connected()) {
      sensorChar.notify((uint8_t*)s, strlen(s));
    }

    Serial.print("Updated BLE value: ");
    Serial.println(s);
  }

  delay(10);   // keep CPU sane
}
