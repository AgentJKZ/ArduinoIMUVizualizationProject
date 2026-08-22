// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {           // "!"" is a logical NOT operator
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  configuration();
  Serial.println("");
  delay(100);
}

float Gx_bias = 0;
float Gy_bias = 0;
float Gz_bias = 0;

void configuration() {
  Serial.println("Beginning Calibration, DO NOT MOVE IMU");
  delay(3000);
  for (int i=0; i<1000; i++) {
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp);
    
    Gx_bias += degrees(g.gyro.x);
    Gy_bias += degrees(g.gyro.y);
    Gz_bias += degrees(g.gyro.z);
    delay(2);
  }

  Gx_bias = Gx_bias/1000;
  Gy_bias = Gy_bias/1000;
  Gz_bias = Gz_bias/1000;
  Serial.println("Calibration Complete!");
  //delay(1000);

}

float roll_filter = 0;
float pitch_filter = 0;
float rollG_deg = 0;
float pitchG_deg = 0;                 float yawG_deg = 0;
unsigned long lastTime = 0;
bool filterInitialized = false;

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Modifying angular velocity to be deg/s
  float Gx_deg = g.gyro.x * 180 / PI;
  float Gy_deg = degrees(g.gyro.y);
  float Gz_deg = degrees(g.gyro.z);  //This is the yaw rate
  

  Gx_deg = Gx_deg - Gx_bias;
  Gy_deg = Gy_deg - Gy_bias;
  Gz_deg = Gz_deg - Gz_bias;

  // Adding Roll and Pitch from Accel
  float Ax = a.acceleration.x;
  float Ay = a.acceleration.y;
  float Az = a.acceleration.z;
  float rollA  = atan2(Ay,Az);
  float pitchA = atan2(-Ax, sqrt(Ay*Ay + Az*Az));
  float rollA_deg = degrees(rollA);
  float pitchA_deg = degrees(pitchA);

  // Initialize Filter
  if (filterInitialized == false)
  {
    roll_filter = rollA_deg;
    pitch_filter = pitchA_deg;
    rollG_deg = rollA_deg;
    pitchG_deg = pitchA_deg;

    yawG_deg = 0;               //Assuming 0 since we can't measure

    lastTime = micros();
    filterInitialized = true;
    return;                             //Initialize everything, grab the current time, and exit the loop to start it again.
  }

  //Arduino Timer
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  float Hertz = 1/dt;           // Sample Rate
  

  // Adding Roll, Pitch, and Yaw from Gyro
  rollG_deg += Gx_deg*dt;
  pitchG_deg += Gy_deg*dt;
  yawG_deg += Gz_deg*dt;


  // Adding Complementary Filter
  float rollprediction = roll_filter + Gx_deg*dt;
  float pitchprediction = pitch_filter + Gy_deg*dt;
   roll_filter = 0.98*rollprediction + 0.02*rollA_deg;
   pitch_filter = 0.98*pitchprediction + 0.02*pitchA_deg;

  Serial.print(roll_filter);          //ALL POSITION VALUES IN DEGREES
  Serial.print(",");
  Serial.print(pitch_filter);
  Serial.print(",");
  Serial.print(rollA_deg);
  Serial.print(",");
  Serial.print(pitchA_deg);
  Serial.print(",");
  Serial.print(rollG_deg);
  Serial.print(",");
  Serial.print(pitchG_deg);
  Serial.print(",");
  Serial.print(yawG_deg);
  Serial.print(",");
  Serial.print(Gx_deg);
  Serial.print(",");
  Serial.print(Gy_deg);
  Serial.print(",");
  Serial.print(Gz_deg);
  Serial.print(",");
  Serial.print(dt,6);                   // IN SEC
  Serial.print(",");
  Serial.print(Hertz,6);                // IN SEC^-1
  Serial.print(",");
  Serial.println(temp.temperature);     // IN DEG CELSIUS


  //delay(500);
}