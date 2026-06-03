#include <Wire.h>

// Bizim belirlediğimiz I2C pinleri
#define I2C_SDA 8
#define I2C_SCL 9

void setup() {
  Serial.begin(115200);
  delay(2000); // Konsolun açılması için bekle
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("\nI2C Tarayıcı Başlıyor...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("I2C Hattı Taranıyor...");

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("BINGO! I2C cihazı bulundu. Adres: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Bilinmeyen hata! Adres: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0)
    Serial.println("Hiçbir I2C cihazı bulunamadı. Kablolarda/Sensörde sorun var.\n");
  else
    Serial.println("Tarama tamamlandı.\n");

  delay(5000); // 5 saniyede bir tekrarla
}
