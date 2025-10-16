#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>  // Library untuk BMP390L pressure sensor

// ===== KALMAN FILTER 2x2 =====
class KalmanFilter2x2 {
private:
  // State vector [altitude, velocity]
  float x[2];
  
  // State transition matrix F (2x2)
  float F[2][2];
  
  // Covariance matrix P (2x2)
  float P[2][2];
  
  // Process noise covariance Q (2x2)
  float Q[2][2];
  
  // Measurement matrix H (1x2)
  float H[2];
  
  // Measurement noise covariance R (scalar)
  float R;
  
  // Time step
  float dt;
  
  // Pressure altitude conversion parameters
  float sea_level_pressure;
  float temperature_lapse_rate;
  float gravity;
  float gas_constant;
  
public:
  KalmanFilter2x2(float dt = 0.1) {
    this->dt = dt;
    
    // Initialize state vector
    x[0] = 0.0;  // altitude
    x[1] = 0.0;  // velocity
    
    // Initialize state transition matrix
    // [1    dt]
    // [0     1]
    F[0][0] = 1.0;  F[0][1] = dt;
    F[1][0] = 0.0;  F[1][1] = 1.0;
    
    // Initialize covariance matrix
    P[0][0] = 1000.0;  P[0][1] = 0.0;
    P[1][0] = 0.0;     P[1][1] = 1000.0;
    
    // Initialize process noise
    Q[0][0] = 0.01;  Q[0][1] = 0.0;
    Q[1][0] = 0.0;   Q[1][1] = 0.01;
    
    // Initialize measurement matrix
    H[0] = 1.0;  // measure altitude
    H[1] = 0.0;  // don't measure velocity directly
    
    // Initialize measurement noise
    R = 10.0;
    
    // Atmospheric constants
    sea_level_pressure = 101325.0;
    temperature_lapse_rate = 0.0065;
    gravity = 9.80665;
    gas_constant = 287.05;
  }
  
  // Set measurement noise (standar deviasi)
  void setMeasurementNoise(float noise_std) {
    R = noise_std * noise_std;
  }
  
  // Set process noise
  void setProcessNoise(float alt_noise, float vel_noise) {
    Q[0][0] = alt_noise * alt_noise;
    Q[1][1] = vel_noise * vel_noise;
  }
  
  // Set sea level pressure (Pa)
  void setSeaLevelPressure(float pressure_pa) {
    sea_level_pressure = pressure_pa;
  }
  
  // Convert pressure and temperature to altitude (barometric formula)
  float pressureToAltitude(float pressure_pa, float temp_celsius) {
    float temp_kelvin = temp_celsius + 273.15;
    
    // Hypsometric equation
    float ratio = pressure_pa / sea_level_pressure;
    float exponent = (gas_constant * temperature_lapse_rate) / gravity;
    float altitude = (temp_kelvin / temperature_lapse_rate) * 
                     (1.0 - pow(ratio, 1.0 / 5.255));
    
    return altitude;
  }
  
  // Prediction step
  void predict() {
    // Temporary variables for matrix operations
    float x_new[2];
    float P_new[2][2];
    
    // x = F * x
    x_new[0] = F[0][0] * x[0] + F[0][1] * x[1];
    x_new[1] = F[1][0] * x[0] + F[1][1] * x[1];
    
    x[0] = x_new[0];
    x[1] = x_new[1];
    
    // P = F * P * F^T + Q
    // First: temp = F * P
    float temp[2][2];
    temp[0][0] = F[0][0] * P[0][0] + F[0][1] * P[1][0];
    temp[0][1] = F[0][0] * P[0][1] + F[0][1] * P[1][1];
    temp[1][0] = F[1][0] * P[0][0] + F[1][1] * P[1][0];
    temp[1][1] = F[1][0] * P[0][1] + F[1][1] * P[1][1];
    
    // Then: P_new = temp * F^T + Q
    P_new[0][0] = temp[0][0] * F[0][0] + temp[0][1] * F[1][0] + Q[0][0];
    P_new[0][1] = temp[0][0] * F[0][1] + temp[0][1] * F[1][1] + Q[0][1];
    P_new[1][0] = temp[1][0] * F[0][0] + temp[1][1] * F[1][0] + Q[1][0];
    P_new[1][1] = temp[1][0] * F[0][1] + temp[1][1] * F[1][1] + Q[1][1];
    
    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];
  }
  
  // Update step
  void update(float pressure_pa, float temp_celsius) {
    // Measure altitude from pressure and temperature
    float z = pressureToAltitude(pressure_pa, temp_celsius);
    
    // Innovation: y = z - H * x
    float y = z - (H[0] * x[0] + H[1] * x[1]);
    
    // Innovation covariance: S = H * P * H^T + R
    float S = H[0] * (P[0][0] * H[0] + P[0][1] * H[1]) + 
              H[1] * (P[1][0] * H[0] + P[1][1] * H[1]) + R;
    
    // Kalman gain: K = P * H^T / S
    float K[2];
    K[0] = (P[0][0] * H[0] + P[0][1] * H[1]) / S;
    K[1] = (P[1][0] * H[0] + P[1][1] * H[1]) / S;
    
    // Update state: x = x + K * y
    x[0] += K[0] * y;
    x[1] += K[1] * y;
    
    // Update covariance: P = (I - K * H) * P
    float P_new[2][2];
    P_new[0][0] = (1.0 - K[0] * H[0]) * P[0][0] - K[0] * H[1] * P[1][0];
    P_new[0][1] = (1.0 - K[0] * H[0]) * P[0][1] - K[0] * H[1] * P[1][1];
    P_new[1][0] = -K[1] * H[0] * P[0][0] + (1.0 - K[1] * H[1]) * P[1][0];
    P_new[1][1] = -K[1] * H[0] * P[0][1] + (1.0 - K[1] * H[1]) * P[1][1];
    
    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];
  }
  
  // Get altitude estimate
  float getAltitude() {
    return x[0];
  }
  
  // Get velocity estimate
  float getVelocity() {
    return x[1];
  }
  
  // Get covariance for diagnostics
  float getCovariance(int i, int j) {
    return P[i][j];
  }
  
  // Initialize state (reset filter)
  void initState(float alt, float vel) {
    x[0] = alt;
    x[1] = vel;
  }
};

// ===== SENSOR INTERFACE =====
Adafruit_BMP3XX bmp;
KalmanFilter2x2 kf(0.1);  // dt = 0.1 second (10 Hz)

// Timing variables
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100;  // 100 ms (10 Hz)

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Initialize BMP390L sensor
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP390L sensor, check wiring!");
    while (1) {}
  }
  
  Serial.println("BMP390L initialized successfully!");
  
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  // Configure Kalman Filter
  kf.setSeaLevelPressure(101325.0);  // Standard sea level pressure
  kf.setMeasurementNoise(5.0);        // Sensor noise ~5 meters
  kf.setProcessNoise(0.1, 0.5);       // Process noise
  
  // Initialize filter state
  if (bmp.performReading()) {
    float init_pressure = bmp.pressure;
    float init_temp = bmp.temperature;
    float init_altitude = kf.pressureToAltitude(init_pressure, init_temp);
    kf.initState(init_altitude, 0.0);
  }
  
  Serial.println("\n===== Kalman Filter - Altitude & Velocity (BMP390L) =====");
  Serial.println("Time(ms)\tPress(Pa)\tTemp(C)\t\tAlt_meas(m)\tAlt_est(m)\tVel_est(m/s)");
  
  lastUpdateTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update at fixed interval
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;
    
    // Read sensor data
    if (bmp.performReading()) {
      float pressure = bmp.pressure;
      float temperature = bmp.temperature;
      
      // Calculate measured altitude
      float altitude_measured = kf.pressureToAltitude(pressure, temperature);
      
      // Kalman Filter prediction and update
      kf.predict();
      kf.update(pressure, temperature);
      
      // Get estimates
      float altitude_estimated = kf.getAltitude();
      float velocity_estimated = kf.getVelocity();
      
      // Print results
      Serial.print(currentTime);
      Serial.print("\t");
      Serial.print(pressure);
      Serial.print("\t");
      Serial.print(temperature, 1);
      Serial.print("\t\t");
      Serial.print(altitude_measured, 2);
      Serial.print("\t\t");
      Serial.print(altitude_estimated, 2);
      Serial.print("\t\t");
      Serial.println(velocity_estimated, 3);
    } else {
      Serial.println("BMP390L reading failed!");
    }
  }
}