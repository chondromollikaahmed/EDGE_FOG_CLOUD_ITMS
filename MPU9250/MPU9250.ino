/***************************************************************************
* Example sketch for the MPU9250_WE library
*
* This sketch shows how to get acceleration, gyroscocope, magnetometer and 
* temperature data from the MPU9250. It might be a bit confusing with all the 
* settings. Therefore you will also find sketches for the individual data.
* 
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
* 
***************************************************************************/

#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68


#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// On ESP32: GPIO-21(SDA), GPIO-22(SCL)
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* There are several ways to create your MPU9250 object:
 * MPU9250_WE myMPU9250 = MPU9250_WE()              -> uses Wire / I2C Address = 0x68
 * MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR)  -> uses Wire / MPU9250_ADDR
 * MPU9250_WE myMPU9250 = MPU9250_WE(&wire2)        -> uses the TwoWire object wire2 / MPU9250_ADDR
 * MPU9250_WE myMPU9250 = MPU9250_WE(&wire2, MPU9250_ADDR) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

void setup() {

  
  Serial.begin(115200);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();

  delay(1000);


 display.clearDisplay();
 display.setTextColor(WHITE);
 display.setTextSize(1.5);
 display.setCursor(0,5);
 display.print("MPU Angles");
 display.display();
 delay(3000);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  
  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  Use either autoOffset or setAccOffsets, not both.
   */
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  The gyroscope data is not zero, even if you don't move the MPU9250. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //myMPU9250.setGyrOffsets(45.0, 145.0, -105.0);

  /*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you 
   *  need to select the bandwdith, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
   *  but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
   *  MPU9250_BW_WO_DLPF_3600 
   *  MPU9250_BW_WO_DLPF_8800
   */
  myMPU9250.enableGyrDLPF();
  //myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
  
  /*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level. 
   *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
   *  
   *  DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
   *    0         250            0.97             8
   *    1         184            2.9              1
   *    2          92            3.9              1
   *    3          41            5.9              1
   *    4          20            9.9              1
   *    5          10           17.85             1
   *    6           5           33.48             1
   *    7        3600            0.17             8
   *    
   *    You achieve lowest noise using level 6  
   */
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

  /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
   *  Sample rate = Internal sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
   *  Divider is a number 0...255
   */
  myMPU9250.setSampleRateDivider(5);

  /*  MPU9250_GYRO_RANGE_250       250 degrees per second (default)
   *  MPU9250_GYRO_RANGE_500       500 degrees per second
   *  MPU9250_GYRO_RANGE_1000     1000 degrees per second
   *  MPU9250_GYRO_RANGE_2000     2000 degrees per second
   */
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

  /*  MPU9250_ACC_RANGE_2G      2 g   (default)
   *  MPU9250_ACC_RANGE_4G      4 g
   *  MPU9250_ACC_RANGE_8G      8 g   
   *  MPU9250_ACC_RANGE_16G    16 g
   */
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  /*  Enable/disable the digital low pass filter for the accelerometer 
   *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
   */
  myMPU9250.enableAccDLPF(true);

  /*  Digital low pass filter (DLPF) for the accelerometer, if enabled 
   *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
   *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
   *     0           460               1.94           1
   *     1           184               5.80           1
   *     2            92               7.80           1
   *     3            41              11.80           1
   *     4            20              19.80           1
   *     5            10              35.70           1
   *     6             5              66.96           1
   *     7           460               1.94           1
   */
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  /* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
   * By default all axes are enabled. Parameters are:  
   * MPU9250_ENABLE_XYZ  //all axes are enabled (default)
   * MPU9250_ENABLE_XY0  // X, Y enabled, Z disabled
   * MPU9250_ENABLE_X0Z   
   * MPU9250_ENABLE_X00
   * MPU9250_ENABLE_0YZ
   * MPU9250_ENABLE_0Y0
   * MPU9250_ENABLE_00Z
   * MPU9250_ENABLE_000  // all axes disabled
   */
  //myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
  //myMPU9250.enableGyrAxes(MPU9250_ENABLE_XYZ);
  
  /*
   * AK8963_PWR_DOWN       
   * AK8963_CONT_MODE_8HZ         default
   * AK8963_CONT_MODE_100HZ
   * AK8963_FUSE_ROM_ACC_MODE 
   */
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
}

const float g = 9.81; // Acceleration due to gravity in m/s^2

// Complementary filter parameters
const float alpha = 0.9; // Weight of gyro angle estimate
const float dt = 0.01; // Time step in seconds

// Initial orientation estimates
float roll = 0.0; // Roll angle in radians
float pitch = 0.0; // Pitch angle in radians

void loop() {
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  float acc_x=gValue.x;
  Serial.print("   ");
  Serial.print(gValue.y);
  float  acc_y=gValue.y;
  Serial.print("   ");
  Serial.println(gValue.z);
  float  acc_z=gValue.z-1;
  Serial.print("Resultant g: ");
  Serial.println(resultantG);


  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
 float  gx =gyr.x;
  Serial.print("   ");
  Serial.print(gyr.y);
  float   gy =gyr.y;
  Serial.print("   ");
  Serial.println(gyr.z);
  float   gz =gyr.z;


      // Compute roll and pitch angles using accelerometer and gyroscope readings
  float accel_roll = atan2(-acc_y, acc_z);
  float accel_pitch = atan2(acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
  float gyro_roll = roll + gx * dt / 131.0;
  float gyro_pitch = pitch + gy * dt / 131.0;

  // Combine accelerometer and gyroscope estimates using a complementary filter
  roll = alpha * gyro_roll + (1 - alpha) * accel_roll;
  pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch;

  // Calculate the acceleration in the direction of motion
  float cos_theta = cos(roll);
  float sin_theta = sin(roll);
  float cos_phi = cos(pitch);
  float sin_phi = sin(pitch);
  float acceleration_x = acc_x * cos_theta + acc_y * sin_phi * sin_theta + acc_z * cos_phi * sin_theta;
  float acceleration_y = acc_y * cos_phi - acc_z * sin_phi;
  float acceleration_z = -acc_x * sin_theta + acc_y * sin_phi * cos_theta + acc_z * cos_phi * cos_theta;
  float acceleration = g * cos_theta * sqrt(acceleration_x * acceleration_x + acceleration_z * acceleration_z);

  // Integration
  static float speed = 0.0;
  static float last_time = millis() / 1000.0;

  float current_time = millis() / 1000.0;
  float time_diff = current_time - last_time;
 
  speed+ = acceleration * time_diff;
  last_time = current_time;

  Serial.println("Acceleration (m/s^2): " + String(acceleration, 3));
  Serial.println("Speed (m/s): " + String(speed, 3));



  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");
  display.clearDisplay();
display.setTextSize(1.5);
display.setCursor(0,0);
  display.print("X: ");
  display.println(acc_z);


 display.setTextSize(1.5);
display.setCursor(0,20);
  display.print("Y: ");
  display.println(acc_y);


display.setTextSize(1.5);
display.setCursor(0,40);
  display.print("z: ");
display.println(acc_z);


display.setTextSize(1.5);
display.setCursor(60,20);
  display.print("S: ");
display.println(String(speed, 5));
display.setTextSize(1);
display.setCursor(80,30);
display.println("ms-1");


display.setTextSize(1.5);
display.setCursor(60,40);
display.print("A: ");
display.println(String(acceleration, 5));
display.setTextSize(1);
display.setCursor(80,50);
display.println("ms-2");


display.display();
  delay(1000);
}
