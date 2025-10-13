#include <Adafruit_BMP3XX.h>
#include <SPI.h>

// Pin SPI BMP390L
#define BMP_CS   38
#define BMP_SCK  36
#define BMP_MISO 37
#define BMP_MOSI 35

Adafruit_BMP3XX bmp;

// ===== KOEFISIEN FIR (15 tap) =====
// Kaiser
const float h_kaiser[15] = {
  1.33050603e-04, 2.59169437e-03, 1.23348346e-02, 3.55903430e-02,
  7.46290763e-02, 1.22424923e-01, 1.62880237e-01, 1.78831681e-01,
  1.62880237e-01, 1.22424923e-01, 7.46290763e-02, 3.55903430e-02,
  1.23348346e-02, 2.59169437e-03, 1.33050603e-04
};
// Hann
const float h_hann[15] = {
  0.0, 0.00500292, 0.02175076, 0.04984266, 0.08472268,
  0.11876977, 0.14358016, 0.15266208, 0.14358016, 0.11876977,
  0.08472268, 0.04984266, 0.02175076, 0.00500292, 0.0
};
// Hamming
const float h_hamming[15] = {
  0.00636831, 0.01184721, 0.02732023, 0.05240370, 0.08314837,
  0.11297744, 0.13464889, 0.14257169, 0.13464889, 0.11297744,
  0.08314837, 0.05240370, 0.02732023, 0.01184721, 0.00636831
};
// Blackman
const float h_blackman[15] = {
 -1.38767721e-18, 2.29880885e-03, 1.22599608e-02, 3.55965151e-02,
  7.46613077e-02, 1.22542423e-01, 1.63096753e-01, 1.79088462e-01,
  1.63096753e-01, 1.22542423e-01, 7.46613077e-02, 3.55965151e-02,
  1.22599608e-02, 2.29880885e-03, -1.38767721e-18
};
// Boxcar
const float h_boxcar[15] = {
  0.0452267, 0.05360995, 0.06130428, 0.0680308, 0.07354218,
  0.07763356, 0.08015167, 0.08100173, 0.08015167, 0.07763356,
  0.07354218, 0.0680308, 0.06130428, 0.05360995, 0.0452267
};

// ===== Buffer FIR =====
// kita butuh buffer utk Temp, Press, Alt, masing2 filter
float bufT_kaiser[15], bufT_hann[15], bufT_hamming[15], bufT_blackman[15], bufT_boxcar[15];
float bufP_kaiser[15], bufP_hann[15], bufP_hamming[15], bufP_blackman[15], bufP_boxcar[15];
float bufA_kaiser[15], bufA_hann[15], bufA_hamming[15], bufA_blackman[15], bufA_boxcar[15];
int idxT_kaiser=0, idxT_hann=0, idxT_hamming=0, idxT_blackman=0, idxT_boxcar=0;
int idxP_kaiser=0, idxP_hann=0, idxP_hamming=0, idxP_blackman=0, idxP_boxcar=0;
int idxA_kaiser=0, idxA_hann=0, idxA_hamming=0, idxA_blackman=0, idxA_boxcar=0;

// Fungsi FIR generik
float firProcess(float x, const float *h, float *buf, int &idx, int NTAP=15) {
  buf[idx] = x;
  float y = 0.0f;
  int i = idx;
  for (int k = 0; k < NTAP; k++) {
    y += h[k] * buf[i];
    if (--i < 0) i = NTAP - 1;
  }
  if (++idx >= NTAP) idx = 0;
  return y;
}

// Fungsi konversi tekanan -> altitude
float pressureToAltitude(float pPa, float P0 = 101325.0f) {
  return 44330.0f * (1.0f - powf(pPa / P0, 0.190294957f));
}

// Tekanan referensi
float P0 = 101325.0;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println("BMP390L not found, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE); // pakai FIR eksternal
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Kalibrasi P0
  float sumP = 0;
  for (int i = 0; i < 100; i++) {
    if (bmp.performReading()) sumP += bmp.pressure;
    delay(20);
  }
  P0 = sumP / 100.0;
  Serial.print("Calibrated P0 = "); Serial.println(P0);

  Serial.println("===============================================================================================================");
  Serial.println(" Temp(C) | Press(hPa) | RawAlt | Kaiser(T,P,A) | Hann(T,P,A) | Hamming(T,P,A) | Blackman(T,P,A) | Boxcar(T,P,A)");
  Serial.println("===============================================================================================================");
}

void loop() {
  if (!bmp.performReading()) return;

  float T = bmp.temperature;       // °C
  float P = bmp.pressure;          // Pa
  float A = pressureToAltitude(P, P0);

  // FIR Filtering
  float T_kaiser = firProcess(T, h_kaiser, bufT_kaiser, idxT_kaiser);
  float P_kaiser = firProcess(P, h_kaiser, bufP_kaiser, idxP_kaiser);
  float A_kaiser = firProcess(A, h_kaiser, bufA_kaiser, idxA_kaiser);

  float T_hann = firProcess(T, h_hann, bufT_hann, idxT_hann);
  float P_hann = firProcess(P, h_hann, bufP_hann, idxP_hann);
  float A_hann = firProcess(A, h_hann, bufA_hann, idxA_hann);

  float T_hamming = firProcess(T, h_hamming, bufT_hamming, idxT_hamming);
  float P_hamming = firProcess(P, h_hamming, bufP_hamming, idxP_hamming);
  float A_hamming = firProcess(A, h_hamming, bufA_hamming, idxA_hamming);

  float T_blackman = firProcess(T, h_blackman, bufT_blackman, idxT_blackman);
  float P_blackman = firProcess(P, h_blackman, bufP_blackman, idxP_blackman);
  float A_blackman = firProcess(A, h_blackman, bufA_blackman, idxA_blackman);

  float T_boxcar = firProcess(T, h_boxcar, bufT_boxcar, idxT_boxcar);
  float P_boxcar = firProcess(P, h_boxcar, bufP_boxcar, idxP_boxcar);
  float A_boxcar = firProcess(A, h_boxcar, bufA_boxcar, idxA_boxcar);

  // Print hasil ke Serial Monitor
  Serial.printf(" %7.2f | %10.2f | %6.2f | K:(%5.2f,%8.2f,%6.2f) | Hn:(%5.2f,%8.2f,%6.2f) | Hm:(%5.2f,%8.2f,%6.2f) | Bk:(%5.2f,%8.2f,%6.2f) | Bx:(%5.2f,%8.2f,%6.2f)\n",
                T, P/100.0, A,
                T_kaiser, P_kaiser/100.0, A_kaiser,
                T_hann, P_hann/100.0, A_hann,
                T_hamming, P_hamming/100.0, A_hamming,
                T_blackman, P_blackman/100.0, A_blackman,
                T_boxcar, P_boxcar/100.0, A_boxcar);

  delay(20); // ~50 Hz
}
