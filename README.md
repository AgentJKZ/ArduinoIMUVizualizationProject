# Arduino_IMU_Vizualization_Project
Attitude Visualization Project utilizing Arduino R3 Uno, MPU6050, Matlab, and Arduino IDE.
Developed a real-time inertial measurement system to estimate roll, pitch, and yaw estimate, characterize sensor behavior, and visualize aircraft attitude in MATLAB.
The MPU6050 has a built in accelerometer and gyroscope, but is lacking a magnetometer, and can therefore only accurately measure roll and pitch. The best that can be achieved for yaw is an estimate. 
The Arduino IDE processes data from the R3 and sensor over a 115200-baud serial connection. It is self calibrating to remove gyro bias at initialization.
A complementary filter is used for fusing Accelerometer and Gyro sensor data:
  Gyroscope integration provides responsive short-term attitude changes but is susceptible to      accumulated drift. Accelerometer-derived attitude provides a long-term gravitational reference   but is more susceptible to transient disturbances. A complementary filter combines the two       measurements.
Sensor Data is sent to MATLAB where it is processed and turned into a 2D Attitude Indicator and 3D plane model (attached .obj file) that reacts to external stimuli on the IMU. 
