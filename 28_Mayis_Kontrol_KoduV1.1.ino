#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// --- PIN TANIMLAMALARI ---
#define I2C_SDA 8
#define I2C_SCL 9

#define IR_SOL    3
#define IR_ORTA   5
#define IR_SAG    7

#define HIZ_SENSORU 11
#define SERVO_PIN   12

// DC Motor Sürücü Pinleri 
#define MOTOR_IN1   16
#define MOTOR_IN2   18

// --- NESNELER VE DEĞİŞKENLER ---
Adafruit_MPU6050 mpu;
Servo direksiyonServo;

// Interrupt (Kesme) değişkeni
volatile unsigned long tekerlekPulsSayisi = 0;
unsigned long oncekiZaman = 0;
const int beklemeSuresi = 100; // 100ms'de bir veri gönder

// --- KESME (INTERRUPT) FONKSİYONU ---
void IRAM_ATTR pulsSayici() {
  tekerlekPulsSayisi++;
}

void setup() {
  Serial.begin(115200);
  
  // ESP32 S2 Mini Native USB bağlantısı için KRİTİK bekleme!
  // Bilgisayarın portu tanıması için 2 saniye veriyoruz, böylece ilk verileri kaçırmayacağız.
  delay(2000); 

  Serial.println("\n--- WRO Robot Sistemi Baslatiliyor ---");

  // 1. I2C ve MPU6050 Başlatma (DÜZELTİLEN KISIM)
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Sensör adresini (0x68) ve kendi belirlediğimiz Wire hattını kütüphaneye zorunlu kılıyoruz
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("HATA: MPU6050 Bulunamadi! Kablolari kontrol edin.");
    while (1) { delay(10); } 
  }
  Serial.println("MPU6050 (Gyro/Ivme) basariyla baglandi!");
  
  // WRO için hassasiyet ayarları
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 2. Dijital Pin Ayarları (IR Sensörler ve Motor)
  pinMode(IR_SOL, INPUT);
  pinMode(IR_ORTA, INPUT);
  pinMode(IR_SAG, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // 3. Hız Sensörü Kesmesi (Interrupt)
  pinMode(HIZ_SENSORU, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HIZ_SENSORU), pulsSayici, RISING);

  // 4. Servo Motor Ayarı
  direksiyonServo.attach(SERVO_PIN, 500, 2400); 
  direksiyonServo.write(90); // Tekerlekleri başlangıçta düz konuma getir

  Serial.println("Kurulum tamamlandi. Veri akisi basliyor...\n");
  
  // Verilerin ne anlama geldiğini gösteren başlık satırı
  Serial.println("IvmeX,GyroZ,IR_Sol,IR_Orta,IR_Sag,PulsSayisi");
}

void loop() {
  unsigned long simdikiZaman = millis();

  // Her 100ms'de bir sensörleri oku ve gönder (Ana döngüyü yormamak için)
  if (simdikiZaman - oncekiZaman >= beklemeSuresi) {
    oncekiZaman = simdikiZaman;

    // --- 1. MPU6050 OKUMA ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // --- 2. IR SENSÖRLERİ OKUMA ---
    int solDurum  = digitalRead(IR_SOL);
    int ortaDurum = digitalRead(IR_ORTA);
    int sagDurum  = digitalRead(IR_SAG);

    // --- 3. HIZ SENSÖRÜ OKUMA ---
    noInterrupts(); // Kesmeleri anlık durdur (veri kaybını önlemek için)
    unsigned long anlikPuls = tekerlekPulsSayisi;
    tekerlekPulsSayisi = 0; // Bir sonraki döngü için sıfırla
    interrupts(); // Kesmeleri geri aç

    // --- RASPBERRY PI İÇİN CSV FORMATINDA YAZDIRMA ---
    Serial.print(a.acceleration.x); Serial.print(",");
    Serial.print(g.gyro.z);         Serial.print(",");
    Serial.print(solDurum);         Serial.print(",");
    Serial.print(ortaDurum);        Serial.print(",");
    Serial.print(sagDurum);         Serial.print(",");
    Serial.println(anlikPuls);      
  }

  // --- RASPBERRY PI'DAN GELEN MOTOR/SERVO KOMUTLARINI DİNLEME ---
  if (Serial.available() > 0) {
    String gelenKomut = Serial.readStringUntil('\n'); 
    komutIsle(gelenKomut);
  }
}

// Raspberry Pi'den gelen komutları ayrıştıran fonksiyon
void komutIsle(String komut) {
  komut.trim();
  
  // Örn: "S:120" -> Servoyu 120 derece yap
  if (komut.startsWith("S:")) {
    int aci = komut.substring(2).toInt();
    direksiyonServo.write(aci);
  } 
  // Örn: "M:1" -> Motor ileri, "M:0" -> Motor dur
  else if (komut.startsWith("M:")) {
    int yon = komut.substring(2).toInt();
    if (yon == 1) { 
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
    } else if (yon == 0) { 
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
    }
  }
}