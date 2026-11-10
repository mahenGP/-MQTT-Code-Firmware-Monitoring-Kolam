#include <WiFi.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <time.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
  #define WIFI_SSID     "ISI_SSID"
  #define WIFI_PASSWORD "ISI_PASSWORD"
  #define MQTT_USER     "ISI_USER"
  #define MQTT_PASS     "ISI_PASS"
#endif

#if __has_include("ca_cert.h")
  #include "ca_cert.h"
  #define PAKAI_CA_CERT 1
#endif

enum KalibrasiState {
  KAL_IDLE,
  KAL_SAMPLING_PH4,
  KAL_STABIL_PH4,
  KAL_SAMPLING_PH7,
  KAL_STABIL_PH7,
  KAL_SELESAI,
  KAL_GAGAL
};

KalibrasiState kalibrasiState = KAL_IDLE;

// ================= MANUAL MODE =======================
bool modeManual = false;
bool pompaAsamManual = false;
bool pompaBasaManual = false;

int pwmAsamManualPersen = 100;
int pwmBasaManualPersen = 100;

int persenKePWM(int persen) {
  persen = constrain(persen, 0, 100);
  return round((persen / 100.0) * 255);
}

// ================= ESTIMASI WAKTU ====================
float estimasiMenit = 0.0;
const float A_ESTIMASI = 33.5;
const float B_ESTIMASI = 0.39;

// ================= VARIABEL GLOBAL ===================
String tanggal = "";
String jam = "";

// ================= OUTPUT KONSTANTA ==================
#define BURUK 25
#define CUKUP 55
#define BAIK  85

// ================= LCD ===============================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pesan LCD yang "ditahan" beberapa detik TANPA delay()
bool lcdAdaTahan = false;
unsigned long lcdTahanSampai = 0;

// Tulis satu baris penuh (dipadding spasi) -> tanpa lcd.clear(), tidak berkedip
void lcdBaris(uint8_t baris, String teks) {
  while (teks.length() < 16) teks += ' ';
  if (teks.length() > 16) teks = teks.substring(0, 16);
  lcd.setCursor(0, baris);
  lcd.print(teks);
}

void lcdPesan(const String& a, const String& b, unsigned long tahanMs = 0) {
  lcdBaris(0, a);
  lcdBaris(1, b);
  if (tahanMs > 0) {
    lcdAdaTahan = true;
    lcdTahanSampai = millis() + tahanMs;
  }
}

bool lcdTertahan() {
  if (!lcdAdaTahan) return false;
  if ((long)(lcdTahanSampai - millis()) > 0) return true;
  lcdAdaTahan = false;
  return false;
}

// ================= PIN ===============================
#define PH_PIN 34
#define ONE_WIRE_BUS 26

// Relay aktif LOW
#define POMPA_PIN_ASAM 27
#define POMPA_PIN_BASA 25

// MOSFET PWM
#define PWM_ASAM 33
#define PWM_BASA 32

#define PWM_FREQ 1000
#define PWM_RES 8

// ================= WIFI ==============================
unsigned long waktuCekWiFi = 0;
const unsigned long intervalCekWiFi = 2000;

unsigned long waktuRetryWiFi = 0;
const unsigned long intervalRetryWiFi = 10000;

bool wifiSebelumnya = false;

// ================= MQTT ==============================
const char* mqtt_server = "ka88a0a8.ala.asia-southeast1.emqxsl.com";
const int mqtt_port = 8883;

// ================= TOPIC MQTT ========================
const char* topicData = "monitoring-kolam/data";
const char* topicDeviceStatus = "monitoring-kolam/status";   // online/offline (LWT)

const char* topicMode = "monitoring-kolam/kontrol/mode";
const char* topicPompaAsam = "monitoring-kolam/kontrol/pompa_asam";
const char* topicPompaBasa = "monitoring-kolam/kontrol/pompa_basa";
const char* topicPwmAsamManual = "monitoring-kolam/kontrol/pwm_asam";
const char* topicPwmBasaManual = "monitoring-kolam/kontrol/pwm_basa";
const char* topicKalibrasiMulai = "monitoring-kolam/kontrol/kalibrasi_mulai";
const char* topicKalibrasiKonfirmasi = "monitoring-kolam/kontrol/kalibrasi_konfirmasi";
const char* topicKalibrasiStatus = "monitoring-kolam/kalibrasi/status";

// ================= JADWAL ============================
unsigned long lastSend = 0;
const unsigned long interval = 60000;          // publish tiap 60 detik

unsigned long lastKontrol = 0;
const unsigned long intervalKontrol = 5000;    // sensor + pompa tiap 5 detik

unsigned long waktuUpdateWaktu = 0;

// Paksa siklus kontrol + publish segera di iterasi loop berikutnya
void jadwalUlang() {
  lastKontrol = millis() - intervalKontrol;
  lastSend = millis() - interval;
}

// ================= DATA SENSOR TERAKHIR ==============
bool  dataValid = false;
float gSuhu = 0;
float gPh = 0;
int   gAdc = 0;
float gVoltage = 0;
int   gStatus = BURUK;
int   gDuty = 0;
int   gPwmPersen = 0;

// ================= EEPROM =============================
#define ADDR_M 0
#define ADDR_C 4

float m = -6.289;
float c = 15.307;

// ================= KALIBRASI pH ======================
bool modeKalibrasi = false;

const int KAL_JUMLAH_DATA = 10;
const int KAL_BATAS_STABIL = 10;
const int KAL_MAX_ULANG = 5;                         // maks. percobaan sampling tidak stabil
const unsigned long KAL_TIMEOUT = 10UL * 60UL * 1000UL;   // 10 menit tanpa aktivitas -> batal

int kalibrasiDataADC[KAL_JUMLAH_DATA];
int kalibrasiIndex = 0;
int kalibrasiGagalStabil = 0;

int kalibrasiMinADC = 0;
int kalibrasiMaxADC = 0;
int kalibrasiRataADC = 0;
int kalibrasiSelisihADC = 0;

unsigned long kalibrasiWaktuSample = 0;
const unsigned long kalibrasiIntervalSample = 1000;

unsigned long kalibrasiWaktuStatusPublish = 0;
const unsigned long kalibrasiIntervalStatusPublish = 500;

unsigned long kalibrasiWaktuAktivitas = 0;

String kalibrasiPesan = "Menunggu perintah kalibrasi";

float adcPH4 = 0;
float adcPH7 = 0;
float voltagePH4 = 0;
float voltagePH7 = 0;

// ================= DS18B20 ===========================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ================= MQTT CLIENT =======================
WiFiClientSecure espClient;
PubSubClient client(espClient);

// ================= FORWARD DECLARATIONS ==============
void mulaiKalibrasi();
void batalKalibrasi(const String& pesan);
void konfirmasiKalibrasi(const String& aksi);
void updateKalibrasi();
void publishKalibrasiStatus();
int readADC();

// =====================================================
// ================= KONTROL POMPA =====================
// =====================================================

void matikanSemuaPompa() {
  digitalWrite(POMPA_PIN_ASAM, HIGH);
  digitalWrite(POMPA_PIN_BASA, HIGH);
  ledcWrite(PWM_ASAM, 0);
  ledcWrite(PWM_BASA, 0);
}

void terapkanPompaManual() {
  // Interlock: pompa asam & basa tidak boleh menyala bersamaan
  if (pompaAsamManual && pompaBasaManual) {
    pompaBasaManual = false;
  }

  digitalWrite(POMPA_PIN_ASAM, pompaAsamManual ? LOW : HIGH);
  digitalWrite(POMPA_PIN_BASA, pompaBasaManual ? LOW : HIGH);

  ledcWrite(PWM_ASAM, pompaAsamManual ? persenKePWM(pwmAsamManualPersen) : 0);
  ledcWrite(PWM_BASA, pompaBasaManual ? persenKePWM(pwmBasaManualPersen) : 0);
}

// ================= DOSIS BERDENYUT (MODE AUTO) =======
// pH < 6.5 -> dosis basa, pH > 7.5 -> dosis asam, selain itu pompa mati.
// Durasi dosis PROPORSIONAL terhadap jarak pH dari batas (6.5 / 7.5):
//   jarak 0            -> DOSIS_ON_MIN_MS
//   jarak >= JARAK_MAX -> DOSIS_ON_MAX_MS  (di antaranya linear)
// Setelah dosis, pompa jeda DOSIS_JEDA_MS supaya air tercampur dan sensor pH
// (yang lambat merespons) sempat membaca hasilnya sebelum dosis berikutnya.
// Sesuaikan nilai-nilai ini dengan volume kolam dan debit pompa.
const unsigned long DOSIS_ON_MIN_MS = 2000;
const unsigned long DOSIS_ON_MAX_MS = 10000;
const unsigned long DOSIS_JEDA_MS   = 45000;
const float JARAK_PH_MAX = 1.5;      // selisih pH yang dianggap "jauh"

// Duty PWM minimum agar pompa DC pasti berputar (0-255).
// UJI di alat: turunkan duty sampai pompa berhenti berputar, lalu pakai nilai
// sedikit di atasnya. Default 100 hanya perkiraan.
const int PWM_MIN_POMPA = 100;

#define ARAH_NONE 0
#define ARAH_BASA 1   // pH rendah -> tambah basa
#define ARAH_ASAM 2   // pH tinggi -> tambah asam

int arahAuto = ARAH_NONE;   // diisi siklusKontrol() tiap 5 detik
int dutyAuto = 0;
float jarakPhAuto = 0;                      // selisih pH dari batas terdekat
unsigned long dosisOnAktifMs = DOSIS_ON_MIN_MS;   // durasi dosis yang sedang berjalan

unsigned long hitungDurasiDosis(float jarak) {
  float f = constrain(jarak / JARAK_PH_MAX, 0.0f, 1.0f);
  return DOSIS_ON_MIN_MS + (unsigned long)(f * (float)(DOSIS_ON_MAX_MS - DOSIS_ON_MIN_MS));
}

enum DosisFase { D_IDLE, D_ON, D_JEDA };
DosisFase dosisFase = D_IDLE;
unsigned long dosisFaseMulai = 0;
int dosisArahAktif = ARAH_NONE;

void terapkanDosis(int arah, int duty) {

  if (arah == ARAH_BASA) {
    digitalWrite(POMPA_PIN_ASAM, HIGH);
    digitalWrite(POMPA_PIN_BASA, LOW);
    ledcWrite(PWM_ASAM, 0);
    ledcWrite(PWM_BASA, duty);
  }
  else if (arah == ARAH_ASAM) {
    digitalWrite(POMPA_PIN_BASA, HIGH);
    digitalWrite(POMPA_PIN_ASAM, LOW);
    ledcWrite(PWM_BASA, 0);
    ledcWrite(PWM_ASAM, duty);
  }
}

// Dipanggil tiap iterasi loop() (non-blocking, berbasis millis)
void updatePompaAuto() {

  // Manual / kalibrasi yang mengatur pompa -> reset mesin dosis
  if (modeManual || modeKalibrasi) {
    dosisFase = D_IDLE;
    dosisArahAktif = ARAH_NONE;
    return;
  }

  int arah = (dutyAuto > 0) ? arahAuto : ARAH_NONE;
  unsigned long now = millis();

  switch (dosisFase) {

    case D_IDLE:
      if (arah != ARAH_NONE) {
        dosisArahAktif = arah;
        dosisOnAktifMs = hitungDurasiDosis(jarakPhAuto);   // dikunci selama dosis ini
        terapkanDosis(arah, dutyAuto);
        dosisFase = D_ON;
        dosisFaseMulai = now;
      }
      break;

    case D_ON:
      if (arah != dosisArahAktif) {
        // pH sudah normal, sensor error, atau arah berbalik -> hentikan segera
        matikanSemuaPompa();
        dosisFaseMulai = now;
        dosisFase = (arah == ARAH_NONE) ? D_IDLE : D_JEDA;
        dosisArahAktif = ARAH_NONE;
      }
      else if (now - dosisFaseMulai >= dosisOnAktifMs) {
        matikanSemuaPompa();
        dosisFase = D_JEDA;
        dosisFaseMulai = now;
      }
      break;

    case D_JEDA:
      if (arah == ARAH_NONE || now - dosisFaseMulai >= DOSIS_JEDA_MS) {
        dosisFase = D_IDLE;
      }
      break;
  }
}

// ================= CALLBACK MQTT =====================
void callback(char* topic, byte* payload, unsigned int length) {

  String msg = "";

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  msg.trim();
  msg.toUpperCase();

  String t = String(topic);

  if (t == topicMode) {

    if (msg == "MANUAL") {

      modeManual = true;
      if (!modeKalibrasi) terapkanPompaManual();
      lastSend = millis() - interval;   // laporkan perubahan mode segera
    }
    else if (msg == "AUTO") {

      if (modeManual) {
        modeManual = false;
        // Reset status manual supaya tidak menyala sendiri saat kembali ke MANUAL
        pompaAsamManual = false;
        pompaBasaManual = false;
        if (!modeKalibrasi) matikanSemuaPompa();
        jadwalUlang();
      }
    }
  }

  else if (t == topicPompaAsam) {

    pompaAsamManual = (msg == "ON");
    if (pompaAsamManual) pompaBasaManual = false;   // interlock
    if (modeManual && !modeKalibrasi) terapkanPompaManual();
  }

  else if (t == topicPompaBasa) {

    pompaBasaManual = (msg == "ON");
    if (pompaBasaManual) pompaAsamManual = false;   // interlock
    if (modeManual && !modeKalibrasi) terapkanPompaManual();
  }

  else if (t == topicPwmAsamManual) {

    pwmAsamManualPersen = constrain(msg.toInt(), 0, 100);
    Serial.print("PWM manual asam: ");
    Serial.print(pwmAsamManualPersen);
    Serial.println("%");

    if (modeManual && !modeKalibrasi) terapkanPompaManual();
  }

  else if (t == topicPwmBasaManual) {

    pwmBasaManualPersen = constrain(msg.toInt(), 0, 100);
    Serial.print("PWM manual basa: ");
    Serial.print(pwmBasaManualPersen);
    Serial.println("%");

    if (modeManual && !modeKalibrasi) terapkanPompaManual();
  }

  else if (t == topicKalibrasiMulai) {

    if (msg == "START") {
      mulaiKalibrasi();
    } else if (msg == "BATAL") {
      batalKalibrasi("Kalibrasi dibatalkan dari web");
    }
  }

  else if (t == topicKalibrasiKonfirmasi) {

    konfirmasiKalibrasi(msg);
  }
}

// ================= RECONNECT MQTT ====================
unsigned long waktuMQTT = 0;
unsigned long intervalMQTT = 5000;               // naik bertahap jika gagal (backoff)
const unsigned long intervalMQTTMax = 60000;

void reconnectMQTT() {

  if (WiFi.status() != WL_CONNECTED) return;
  if (client.connected()) return;
  if (millis() - waktuMQTT < intervalMQTT) return;

#ifdef PAKAI_CA_CERT
  // Verifikasi sertifikat butuh jam yang sudah tersinkron (cek masa berlaku)
  if (time(nullptr) < 1700000000) return;
#endif

  waktuMQTT = millis();

  Serial.println("Mencoba koneksi MQTT...");

  String clientId = "ESP32-";
  clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

  // Last Will: broker otomatis menandai "offline" jika ESP32 putus
  if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASS,
                     topicDeviceStatus, 0, true, "offline")) {

    Serial.println("MQTT Connected");
    intervalMQTT = 5000;

    // Kirim data (termasuk "mode" aktual) segera setelah connect, agar web
    // langsung tahu kondisi sebenarnya setelah reboot/reconnect
    lastSend = millis() - interval;

    client.publish(topicDeviceStatus, "online", true);

    client.subscribe(topicMode);
    client.subscribe(topicPompaAsam);
    client.subscribe(topicPompaBasa);
    client.subscribe(topicPwmAsamManual);
    client.subscribe(topicPwmBasaManual);
    client.subscribe(topicKalibrasiMulai);
    client.subscribe(topicKalibrasiKonfirmasi);

  } else {

    Serial.print("MQTT Failed: ");
    Serial.println(client.state());

    intervalMQTT = min(intervalMQTT * 2, intervalMQTTMax);
  }
}

// ================= ADC AVERAGE =======================
int readADC() {

  const int N = 20;
  long sum = 0;

  for (int i = 0; i < N; i++) {
    sum += analogRead(PH_PIN);
    delay(2);                 // total ~40 ms (sebelumnya ~100 ms)
  }
  return sum / N;
}

// ================= METODE DISKRET ====================
#define DINGIN  0
#define NORMAL  1
#define PANAS   2

#define ASAM    0
#define NETRAL  1
#define BASA    2

int klasifikasiSuhu(float suhu) {
  if (suhu < 24.0) return DINGIN;
  else if (suhu <= 30.0) return NORMAL;
  else return PANAS;
}

int klasifikasiPH(float ph) {
  if (ph < 6.5) return ASAM;
  else if (ph <= 7.5) return NETRAL;
  else return BASA;
}

String namaKategoriSuhu(int kategori) {
  switch (kategori) {
    case DINGIN: return "Dingin";
    case NORMAL: return "Normal";
    case PANAS:  return "Panas";
    default:     return "Tidak diketahui";
  }
}

String namaKategoriPH(int kategori) {
  switch (kategori) {
    case ASAM:   return "Asam";
    case NETRAL: return "Netral";
    case BASA:   return "Basa";
    default:     return "Tidak diketahui";
  }
}

//                 pH
// Suhu       ASAM   NETRAL   BASA
// DINGIN     BURUK  CUKUP    BURUK
// NORMAL     CUKUP  BAIK     CUKUP
// PANAS      BURUK  CUKUP    BURUK
int metodeDiskret(float suhu, float ph) {

  int ks = klasifikasiSuhu(suhu);
  int kp = klasifikasiPH(ph);

  if (ks == NORMAL && kp == NETRAL) return BAIK;

  if ((ks == NORMAL && kp != NETRAL) || (ks != NORMAL && kp == NETRAL)) {
    return CUKUP;
  }

  return BURUK;
}

int getPWMDiskret(int status) {
  if (status == BURUK) return 255;
  if (status == CUKUP) return 128;
  return 0;
}

String getKecepatanDiskret(int status) {
  if (status == BURUK) return "Sangat Cepat";
  if (status == CUKUP) return "Sedang";
  return "Mati";
}

String getKondisiDiskret(int status) {
  if (status == BURUK) return "BURUK";
  if (status == CUKUP) return "CUKUP";
  return "BAIK";
}

// ================= EEPROM =============================
void loadCalibration() {

  EEPROM.get(ADDR_M, m);
  EEPROM.get(ADDR_C, c);

  bool kalibrasiValid = true;

  if (isnan(m) || isnan(c)) kalibrasiValid = false;
  if (m >= 0 || m < -100) kalibrasiValid = false;
  if (c < 0 || c > 50) kalibrasiValid = false;

  if (!kalibrasiValid) {

    Serial.println("Kalibrasi EEPROM tidak valid, pakai default");

    m = -6.289;
    c = 15.307;

    EEPROM.put(ADDR_M, m);
    EEPROM.put(ADDR_C, c);
    EEPROM.commit();
  }
  else {

    Serial.println("Kalibrasi pH dari EEPROM");
    Serial.print("m = ");
    Serial.println(m, 6);
    Serial.print("c = ");
    Serial.println(c, 6);
  }
}

// ================= CEK WIFI ==========================
void cekWiFi() {

  if (millis() - waktuCekWiFi < intervalCekWiFi) return;

  waktuCekWiFi = millis();
  bool wifiSekarang = (WiFi.status() == WL_CONNECTED);

  // ================= WIFI TERPUTUS =================
  if (!wifiSekarang) {

    if (wifiSebelumnya) {
      Serial.println("WiFi TERPUTUS!");

      if (!modeKalibrasi) lcdPesan("WiFi Terputus", "Connecting...", 2000);
    }

    wifiSebelumnya = false;

    if (millis() - waktuRetryWiFi >= intervalRetryWiFi) {
      waktuRetryWiFi = millis();
      Serial.println("Mencoba menghubungkan kembali WiFi...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  // ================= WIFI TERHUBUNG =================
  else {

    if (!wifiSebelumnya) {

      Serial.println("WiFi Connected: " + WiFi.SSID());
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());

      if (!modeKalibrasi) {
        lcdPesan(WiFi.SSID(), WiFi.localIP().toString(), 2000);   // tanpa delay()
      }
    }

    wifiSebelumnya = true;
  }
}

// =====================================================
// ============= KALIBRASI pH (NON-BLOCKING) ===========
// =====================================================
void mulaiKalibrasi() {

  modeKalibrasi = true;

  matikanSemuaPompa();

  kalibrasiIndex = 0;
  kalibrasiGagalStabil = 0;
  kalibrasiState = KAL_SAMPLING_PH4;
  kalibrasiWaktuSample = millis();
  kalibrasiWaktuAktivitas = millis();
  kalibrasiPesan = "Mengambil sampel buffer pH 4.00";

  Serial.println("KALIBRASI DIMULAI (via MQTT)");
  Serial.println("Titik 1: Buffer pH 4.00");

  lcdPesan("Buffer pH 4.00", "Sampling...");

  publishKalibrasiStatus();
}

void batalKalibrasi(const String& pesan) {

  kalibrasiState = KAL_IDLE;
  modeKalibrasi = false;
  kalibrasiPesan = pesan;

  Serial.println("KALIBRASI DIBATALKAN");
  Serial.println(pesan);

  lcdPesan("Kalibrasi", "Dibatalkan", 1500);   // tanpa delay()

  publishKalibrasiStatus();

  jadwalUlang();   // kontrol pompa lanjut segera
}

void konfirmasiKalibrasi(const String& aksiMentah) {

  String aksi = aksiMentah;
  aksi.trim();
  aksi.toUpperCase();

  // Konfirmasi hanya berarti jika kalibrasi sedang berjalan
  if (!modeKalibrasi) return;

  kalibrasiWaktuAktivitas = millis();

  if (aksi == "BATAL") {
    batalKalibrasi("Kalibrasi dibatalkan dari web");
    return;
  }

  if (aksi == "ULANGI") {

    if (kalibrasiState == KAL_STABIL_PH4 || kalibrasiState == KAL_SAMPLING_PH4) {
      kalibrasiIndex = 0;
      kalibrasiGagalStabil = 0;
      kalibrasiState = KAL_SAMPLING_PH4;
      kalibrasiPesan = "Mengambil ulang sampel buffer pH 4.00";
      kalibrasiWaktuSample = millis();
      publishKalibrasiStatus();
    }
    else if (kalibrasiState == KAL_STABIL_PH7 || kalibrasiState == KAL_SAMPLING_PH7) {
      kalibrasiIndex = 0;
      kalibrasiGagalStabil = 0;
      kalibrasiState = KAL_SAMPLING_PH7;
      kalibrasiPesan = "Mengambil ulang sampel buffer pH 7.00";
      kalibrasiWaktuSample = millis();
      publishKalibrasiStatus();
    }

    return;
  }

  if (aksi != "OK") return;

  // ---- Konfirmasi pH 4.00 ----
  if (kalibrasiState == KAL_STABIL_PH4) {

    adcPH4 = kalibrasiRataADC;
    voltagePH4 = adcPH4 * (3.3 / 4095.0);

    Serial.println("Titik pH 4.00 disimpan.");
    Serial.print("ADC = "); Serial.println(adcPH4);
    Serial.print("V   = "); Serial.println(voltagePH4, 4);

    kalibrasiIndex = 0;
    kalibrasiGagalStabil = 0;
    kalibrasiState = KAL_SAMPLING_PH7;
    kalibrasiPesan = "Bilas sensor, lalu mengambil sampel buffer pH 7.00";
    kalibrasiWaktuSample = millis();

    lcdPesan("Buffer pH 7.00", "Sampling...");

    publishKalibrasiStatus();
    return;
  }

  // ---- Konfirmasi pH 7.00, hitung dan simpan ----
  if (kalibrasiState == KAL_STABIL_PH7) {

    adcPH7 = kalibrasiRataADC;
    voltagePH7 = adcPH7 * (3.3 / 4095.0);

    Serial.println("Titik pH 7.00 disimpan.");
    Serial.print("ADC = "); Serial.println(adcPH7);
    Serial.print("V   = "); Serial.println(voltagePH7, 4);

    if (fabs(voltagePH4 - voltagePH7) < 0.001) {

      kalibrasiState = KAL_GAGAL;
      kalibrasiPesan = "Tegangan pH 4 dan pH 7 terlalu dekat. Periksa sensor.";

      Serial.println("KALIBRASI GAGAL: " + kalibrasiPesan);

      lcdPesan("Kalibrasi Gagal", "Tegangan dekat", 3000);

      modeKalibrasi = false;
      publishKalibrasiStatus();
      jadwalUlang();
      return;
    }

    // pH = m*V + c
    float mBaru = (4.00 - 7.00) / (voltagePH4 - voltagePH7);
    float cBaru = 4.00 - (mBaru * voltagePH4);

    bool hasilValid = true;

    if (mBaru >= 0 || mBaru < -100) hasilValid = false;
    if (cBaru < 0 || cBaru > 50) hasilValid = false;

    if (!hasilValid) {

      kalibrasiState = KAL_GAGAL;
      kalibrasiPesan = "Nilai m/c hasil kalibrasi tidak masuk akal. Tidak disimpan.";

      Serial.println("KALIBRASI GAGAL: " + kalibrasiPesan);

      lcdPesan("Kalibrasi Gagal", "Nilai invalid", 3000);

      modeKalibrasi = false;
      publishKalibrasiStatus();
      jadwalUlang();
      return;
    }

    m = mBaru;
    c = cBaru;

    EEPROM.put(ADDR_M, m);
    EEPROM.put(ADDR_C, c);
    EEPROM.commit();

    kalibrasiState = KAL_SELESAI;
    kalibrasiPesan = "Kalibrasi berhasil dan tersimpan ke EEPROM";
    modeKalibrasi = false;

    Serial.println("KALIBRASI BERHASIL");
    Serial.print("m = "); Serial.println(m, 6);
    Serial.print("c = "); Serial.println(c, 6);

    lcdPesan("Kalibrasi OK", "m:" + String(m, 2), 3000);   // tanpa delay(3000)

    publishKalibrasiStatus();

    jadwalUlang();
    return;
  }
}

void updateKalibrasi() {

  if (
    kalibrasiState == KAL_IDLE ||
    kalibrasiState == KAL_SELESAI ||
    kalibrasiState == KAL_GAGAL
  ) {
    return;
  }

  // Timeout: kalau web terputus / lupa konfirmasi, pompa tidak boleh mati selamanya
  if (millis() - kalibrasiWaktuAktivitas >= KAL_TIMEOUT) {
    batalKalibrasi("Kalibrasi dibatalkan otomatis (timeout 10 menit)");
    return;
  }

  // ================= SAMPLING =================
  if (kalibrasiState == KAL_SAMPLING_PH4 || kalibrasiState == KAL_SAMPLING_PH7) {

    if (millis() - kalibrasiWaktuSample >= kalibrasiIntervalSample) {

      kalibrasiWaktuSample = millis();

      int nilai = readADC();
      kalibrasiDataADC[kalibrasiIndex] = nilai;
      kalibrasiIndex++;

      int minADC = kalibrasiDataADC[0];
      int maxADC = kalibrasiDataADC[0];
      long total = 0;

      for (int i = 0; i < kalibrasiIndex; i++) {
        if (kalibrasiDataADC[i] < minADC) minADC = kalibrasiDataADC[i];
        if (kalibrasiDataADC[i] > maxADC) maxADC = kalibrasiDataADC[i];
        total += kalibrasiDataADC[i];
      }

      kalibrasiMinADC = minADC;
      kalibrasiMaxADC = maxADC;
      kalibrasiRataADC = total / kalibrasiIndex;
      kalibrasiSelisihADC = maxADC - minADC;

      float targetPH = (kalibrasiState == KAL_SAMPLING_PH4) ? 4.00 : 7.00;

      kalibrasiPesan =
        "Mengambil sampel " + String(kalibrasiIndex) + "/" +
        String(KAL_JUMLAH_DATA) + " (buffer pH " + String(targetPH, 2) + ")";

      lcdBaris(1, "ADC:" + String(nilai));

      if (kalibrasiIndex >= KAL_JUMLAH_DATA) {

        if (kalibrasiSelisihADC <= KAL_BATAS_STABIL) {

          kalibrasiState =
            (kalibrasiState == KAL_SAMPLING_PH4) ? KAL_STABIL_PH4 : KAL_STABIL_PH7;

          kalibrasiPesan =
            "ADC stabil (rata-rata " + String(kalibrasiRataADC) +
            "). Menunggu konfirmasi dari web.";

          lcdPesan("ADC Stabil", "Konfirmasi web");

        } else {

          kalibrasiGagalStabil++;

          if (kalibrasiGagalStabil >= KAL_MAX_ULANG) {

            kalibrasiState = KAL_GAGAL;
            kalibrasiPesan =
              "ADC tidak stabil setelah " + String(KAL_MAX_ULANG) +
              " percobaan. Periksa sensor / larutan buffer.";
            modeKalibrasi = false;

            lcdPesan("Kalibrasi Gagal", "ADC tdk stabil", 3000);
            jadwalUlang();

          } else {

            kalibrasiIndex = 0;

            kalibrasiPesan =
              "Belum stabil (selisih " + String(kalibrasiSelisihADC) +
              " ADC). Mengambil ulang...";
          }
        }
      }

      publishKalibrasiStatus();
    }

    return;
  }

  // ================= MENUNGGU KONFIRMASI =================
  if (millis() - kalibrasiWaktuStatusPublish >= kalibrasiIntervalStatusPublish) {
    kalibrasiWaktuStatusPublish = millis();
    publishKalibrasiStatus();
  }
}

String namaKalibrasiState(KalibrasiState state) {

  switch (state) {
    case KAL_IDLE:          return "IDLE";
    case KAL_SAMPLING_PH4:  return "SAMPLING_PH4";
    case KAL_STABIL_PH4:    return "STABIL_PH4";
    case KAL_SAMPLING_PH7:  return "SAMPLING_PH7";
    case KAL_STABIL_PH7:    return "STABIL_PH7";
    case KAL_SELESAI:       return "SELESAI";
    case KAL_GAGAL:         return "GAGAL";
    default:                return "IDLE";
  }
}

void publishKalibrasiStatus() {

  if (!client.connected()) return;

  int titik =
    (kalibrasiState == KAL_SAMPLING_PH4 || kalibrasiState == KAL_STABIL_PH4) ? 1 : 2;

  float targetPH = (titik == 1) ? 4.00 : 7.00;

  bool stabil =
    (kalibrasiState == KAL_STABIL_PH4 || kalibrasiState == KAL_STABIL_PH7);

  char payload[640];

  int n = snprintf(
    payload,
    sizeof(payload),

    "{"
      "\"state\":\"%s\","
      "\"titik\":%d,"
      "\"target_ph\":%.2f,"
      "\"adc\":%d,"
      "\"adc_min\":%d,"
      "\"adc_max\":%d,"
      "\"selisih\":%d,"
      "\"progress\":%d,"
      "\"total_sample\":%d,"
      "\"stabil\":%s,"
      "\"pesan\":\"%s\","
      "\"m\":%.6f,"
      "\"c\":%.6f,"
      "\"adc_ph4\":%.0f,"
      "\"adc_ph7\":%.0f,"
      "\"voltage_ph4\":%.4f,"
      "\"voltage_ph7\":%.4f"
    "}",

    namaKalibrasiState(kalibrasiState).c_str(),
    titik,
    targetPH,
    kalibrasiRataADC,
    kalibrasiMinADC,
    kalibrasiMaxADC,
    kalibrasiSelisihADC,
    kalibrasiIndex,
    KAL_JUMLAH_DATA,
    stabil ? "true" : "false",
    kalibrasiPesan.c_str(),
    m,
    c,
    adcPH4,
    adcPH7,
    voltagePH4,
    voltagePH7
  );

  // Jangan kirim JSON terpotong
  if (n <= 0 || n >= (int)sizeof(payload)) {
    Serial.println("Payload status kalibrasi terlalu panjang, tidak dikirim");
    return;
  }

  client.publish(topicKalibrasiStatus, payload, false);
}

// =====================================================
// ================= UPDATE WAKTU ======================
// =====================================================
void updateWaktu() {

  if (millis() - waktuUpdateWaktu < 1000) return;
  waktuUpdateWaktu = millis();

  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, 0)) {
    return;   // waktu belum tersinkron
  }

  char bufferTanggal[20];
  char bufferJam[20];

  strftime(bufferTanggal, sizeof(bufferTanggal), "%d-%m-%Y", &timeinfo);
  strftime(bufferJam, sizeof(bufferJam), "%H:%M", &timeinfo);

  tanggal = String(bufferTanggal);
  jam = String(bufferJam);
}

// =====================================================
// ====== SIKLUS KONTROL (SENSOR + POMPA, TANPA WIFI) ==
// =====================================================

// Sensor bermasalah: data ditandai tidak valid, pompa AUTO dimatikan (aman)
void sensorError(const char* baris1, const char* baris2) {

  Serial.print("Sensor error: ");
  Serial.print(baris1);
  Serial.print(" ");
  Serial.println(baris2);

  dataValid = false;

  if (!modeManual) {
    arahAuto = ARAH_NONE;     // hentikan dosis berdenyut
    dutyAuto = 0;
    matikanSemuaPompa();
  }

  if (!lcdTertahan()) lcdPesan(baris1, baris2);
}

void siklusKontrol() {

  if (modeKalibrasi) {
    return;
  }

  // ---------- SUHU (non-blocking) ----------
  // Ambil hasil konversi dari request siklus sebelumnya, lalu minta konversi baru.
  float suhu = sensors.getTempCByIndex(0);
  sensors.requestTemperatures();

  // -127 = sensor lepas, 85.0 = nilai default saat power-on / belum konversi
  if (suhu == DEVICE_DISCONNECTED_C || suhu == 85.0 || suhu < -50 || suhu > 100) {
    sensorError("Sensor suhu", "tidak terbaca");
    return;
  }

  // ---------- pH ----------
  int adc = readADC();

  // ADC mentok 0 / 4095 -> kabel sensor/pin bermasalah
  if (adc <= 20 || adc >= 4090) {
    sensorError("Sensor pH", "tidak terbaca");
    return;
  }

  float voltage = adc * (3.3 / 4095.0);

  // pH = m * V + c   (tanpa koreksi suhu)
  float ph = m * voltage + c;

  // pH di luar 0-14 pasti salah baca / kalibrasi rusak
  if (ph < 0.0 || ph > 14.0) {
    sensorError("pH di luar", "rentang 0-14");
    return;
  }

  ph = round(ph * 100.0) / 100.0;
  suhu = round(suhu * 100.0) / 100.0;

  // ---------- KLASIFIKASI DISKRET ----------
  int status = metodeDiskret(suhu, ph);
  int duty = getPWMDiskret(status);

  // AUTO: pompa hanya bekerja jika pH di luar netral. Jika pH netral tapi suhu
  // buruk (status CUKUP), pompa tidak menyala -> duty yang dilaporkan harus 0.
  if (!modeManual && klasifikasiPH(ph) == NETRAL) {
    duty = 0;
  }

  // ---------- ESTIMASI WAKTU ----------
  // Pompa hanya bisa memperbaiki pH. Jika pH sudah netral (mis. status CUKUP
  // karena suhu), tidak ada yang bisa dikoreksi pompa -> estimasi 0.
  if (status == BAIK || klasifikasiPH(ph) == NETRAL) {
    estimasiMenit = 0.0;
  } else {
    estimasiMenit = A_ESTIMASI - (B_ESTIMASI * status);
  }

  // ---------- POMPA ----------
  if (modeManual) {

    terapkanPompaManual();

    int pwmManualAktif = 0;

    if (pompaAsamManual) {
      pwmManualAktif = max(pwmManualAktif, persenKePWM(pwmAsamManualPersen));
    }

    if (pompaBasaManual) {
      pwmManualAktif = max(pwmManualAktif, persenKePWM(pwmBasaManualPersen));
    }

    duty = pwmManualAktif;
  }
  else {

    // Hanya menetapkan target; eksekusi pompa (berdenyut) oleh updatePompaAuto()
    arahAuto = (ph < 6.5) ? ARAH_BASA : (ph > 7.5) ? ARAH_ASAM : ARAH_NONE;
    jarakPhAuto = (ph < 6.5) ? (6.5 - ph) : (ph > 7.5) ? (ph - 7.5) : 0.0;
    dutyAuto = (duty > 0) ? max(duty, PWM_MIN_POMPA) : 0;   // pompa DC butuh duty minimum
  }

  int pwmPersen = round((duty / 255.0) * 100);

  // ---------- SIMPAN UNTUK DIPUBLISH ----------
  gSuhu = suhu;
  gPh = ph;
  gAdc = adc;
  gVoltage = voltage;
  gStatus = status;
  gDuty = duty;
  gPwmPersen = pwmPersen;
  dataValid = true;

  // ---------- SERIAL ----------
  Serial.println("==============================");
  Serial.print("Waktu         : "); Serial.print(tanggal); Serial.print(" "); Serial.println(jam);
  Serial.print("pH            : "); Serial.print(ph, 2); Serial.print("  ADC: "); Serial.println(adc);
  Serial.print("Tegangan      : "); Serial.print(voltage, 4); Serial.println(" V");
  Serial.print("Suhu          : "); Serial.print(suhu, 2); Serial.println(" C");
  Serial.print("Kondisi       : "); Serial.println(getKondisiDiskret(status));
  Serial.print("PWM           : "); Serial.print(duty); Serial.print(" ("); Serial.print(pwmPersen); Serial.println("%)");
  Serial.print("Mode          : "); Serial.println(modeManual ? "MANUAL" : "AUTO");
  Serial.print("WiFi / MQTT   : ");
  Serial.print(WiFi.status() == WL_CONNECTED ? "ON" : "OFF");
  Serial.print(" / ");
  Serial.println(client.connected() ? "ON" : "OFF");

  // ---------- LCD (tanpa clear -> tidak berkedip) ----------
  if (!lcdTertahan()) {

    char indikator;
    if (WiFi.status() == WL_CONNECTED && client.connected()) indikator = 'W';
    else if (WiFi.status() == WL_CONNECTED) indikator = 'w';
    else indikator = 'x';

    String l0 = "PH  :" + String(ph, 1);
    while (l0.length() < 15) l0 += ' ';
    l0 += indikator;

    lcdBaris(0, l0);
    lcdBaris(1, "SUHU:" + String(suhu, 1));
  }
}

// =====================================================
// ================= KIRIM DATA MQTT ===================
// =====================================================
void kirimMQTT() {

  if (modeKalibrasi) return;
  if (!dataValid) return;

  String kondisi = getKondisiDiskret(gStatus);
  String kecepatan = getKecepatanDiskret(gStatus);

  bool pompaAsam = digitalRead(POMPA_PIN_ASAM) == LOW;
  bool pompaBasa = digitalRead(POMPA_PIN_BASA) == LOW;

  char payload[700];

  int n = snprintf(
    payload,
    sizeof(payload),

    "{"
      "\"tanggal\":\"%s\","
      "\"jam\":\"%s\","
      "\"suhu\":%.2f,"
      "\"ph\":%.2f,"
      "\"status\":\"%s\","
      "\"pwm\":%d,"
      "\"pwm_persen\":%d,"
      "\"kecepatan\":\"%s\","
      "\"pompa_asam\":\"%s\","
      "\"pompa_basa\":\"%s\","
      "\"mode\":\"%s\","
      "\"estimasi_menit\":%.1f,"
      "\"kategori_suhu\":\"%s\","
      "\"kategori_ph\":\"%s\","
      "\"pwm_asam_manual\":%d,"
      "\"pwm_basa_manual\":%d"
    "}",

    tanggal.c_str(),
    jam.c_str(),
    gSuhu,
    gPh,
    kondisi.c_str(),
    gDuty,
    gPwmPersen,
    kecepatan.c_str(),
    pompaAsam ? "ON" : "OFF",
    pompaBasa ? "ON" : "OFF",
    modeManual ? "MANUAL" : "AUTO",
    estimasiMenit,
    namaKategoriSuhu(klasifikasiSuhu(gSuhu)).c_str(),
    namaKategoriPH(klasifikasiPH(gPh)).c_str(),
    pwmAsamManualPersen,
    pwmBasaManualPersen
  );

  if (n <= 0 || n >= (int)sizeof(payload)) {
    Serial.println("Payload data terlalu panjang, tidak dikirim");
    return;
  }

  if (client.publish(topicData, payload, false)) {

    Serial.println("MQTT Publish berhasil");
    Serial.print("Payload : ");
    Serial.println(payload);

  } else {

    Serial.println("MQTT Publish gagal");
  }
}

// =====================================================
// ================= SETUP =============================
// =====================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  pinMode(PH_PIN, INPUT);

  EEPROM.begin(32);
  loadCalibration();

  // DS18B20: satu konversi blocking di awal, setelah itu non-blocking
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdPesan("Starting...", "");

  // Relay pompa (aktif LOW) -> pastikan mati
  // Set level HIGH dulu baru jadikan OUTPUT, supaya relay tidak sempat tertarik LOW
  digitalWrite(POMPA_PIN_ASAM, HIGH);
  digitalWrite(POMPA_PIN_BASA, HIGH);
  pinMode(POMPA_PIN_ASAM, OUTPUT);
  pinMode(POMPA_PIN_BASA, OUTPUT);

  // PWM MOSFET
  ledcAttach(PWM_ASAM, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_BASA, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_ASAM, 0);
  ledcWrite(PWM_BASA, 0);

  // ================= WIFI =================
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  lcdPesan("Connecting WiFi", "");

  unsigned long mulaiWiFi = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - mulaiWiFi < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  waktuRetryWiFi = millis();

  if (WiFi.status() == WL_CONNECTED) {

    wifiSebelumnya = true;

    Serial.println("WiFi Connected: " + WiFi.SSID());

    lcdPesan(WiFi.SSID(), WiFi.localIP().toString());
    delay(2000);   // aman di setup()
  }
  else {

    wifiSebelumnya = false;

    Serial.println("WiFi belum terhubung");

    lcdPesan("WiFi Terputus", "Connecting...");
  }

  // Waktu (NTP) WIB (UTC+7), dengan server cadangan
  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");

  // MQTT
#ifdef PAKAI_CA_CERT
  espClient.setCACert(MQTT_CA_CERT);     // verifikasi sertifikat server (disarankan)
#else
  Serial.println("PERINGATAN: TLS tanpa verifikasi sertifikat (ca_cert.h belum ada)");
  espClient.setInsecure();
#endif
  espClient.setTimeout(5);            // detik: batasi blocking saat TLS connect
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(1024);
  client.setSocketTimeout(5);         // detik
  client.setKeepAlive(30);
  client.setCallback(callback);

  // Siklus kontrol pertama langsung jalan setelah setup
  lastKontrol = millis() - intervalKontrol;
}

// =====================================================
// ================= LOOP ==============================
// =====================================================

void loop() {

  // ---------- WIFI ----------
  cekWiFi();

  // ---------- MQTT ----------
  if (WiFi.status() == WL_CONNECTED) {

    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();
  }

  updateWaktu();

  // ---------- KALIBRASI (non-blocking, via MQTT) ----------
  updateKalibrasi();

  // ---------- DOSIS POMPA AUTO (berdenyut, non-blocking) ----------
  updatePompaAuto();

  // ---------- KONTROL POMPA (selalu jalan, tanpa syarat WiFi/MQTT) ----------
  if (millis() - lastKontrol >= intervalKontrol) {
    lastKontrol = millis();
    siklusKontrol();
  }

  // ---------- PUBLISH DATA (hanya jika online) ----------
  if (millis() - lastSend >= interval) {

    lastSend = millis();

    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      kirimMQTT();
    }
  }
}
