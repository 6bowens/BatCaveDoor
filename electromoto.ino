#include <Ubidots.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <math.h>
#define PARTICLE_KEEPALIVE 20
#define d2r (PI / 180.0)
#define PI  3.1415926535
#define TOKEN "INSERT HERE" //ubidots

FuelGauge fuel;
Ubidots ubidots(TOKEN);
SYSTEM_MODE(SEMI_AUTOMATIC); //this prevents autocloud connection
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY)); // retain variables in memory

//Configurable Variables
int sleepDur = 60*60*1; //how long to sleep with no movement (s)
int startDur = 10; // time to delay startup for GPS to wakeup
int noGPSDur = 60; // Timeout to go to sleep on no GPS

int moveThres = 1; //movement threshold (m)
int moveNum = 60; //num of consecutive non movements before deepsleep (needs to be longer than a traffic light)

int cloudDur = 5; //Interval to publish to the cloud (s)
int distPubThresh = 1; //Don't publish on nap if haven't moved more than this (m)

int imuMG = 6; // how many mg to trigger movement on
int imuMS = 1; //over what time period

// MPU vars
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
float accelMax = 0;
double accelVec;

// GPS Vars
TinyGPSPlus gps;
char szInfo[64];
double latPrev, lonPrev, latNow, lonNow;
int distance, temp, altNow;
double delta;
int noMov = 0;
int noGPS = 0;
int counter = 0;
retained double latPub; 
retained double lonPub;
bool isValidGPS;
bool sleep = false;
float bat;

// State Vars
char publishData[] = "";

void setup(){
    
    //Define Cloud Vars
    // Particle.variable("lat", &latNow);
    // Particle.variable("lon", &lonNow);
    // Particle.variable("alt", &altNow);
    Particle.keepAlive(PARTICLE_KEEPALIVE);
      
    // Connect to usb serial
    Serial.begin(115200);
    delay(startDur*1000);  
    Serial.println(""); //clear that bitch
    Serial.println("Connection Established");

    //Initialize IMU
    Wire.begin();  
    accelgyro.initialize();  
    Serial.println(accelgyro.testConnection() ? "Initialized IMU..." : "MPU6050 connection failed");
    accelgyro.setMotionDetectionThreshold(imuMG); //mg
    accelgyro.setMotionDetectionDuration(imuMS);  //ms
    accelgyro.setFullScaleAccelRange(0); //+-4g=1, 2g=0
    accelgyro.setDHPFMode(1); // set high pass to 5hz
    accelgyro.setInterruptMode(0); //0 for commonly off
    accelgyro.setIntMotionEnabled(1); //Set MPU to interrupt on motion
    accelgyro.setStandbyXGyroEnabled(1);
    accelgyro.setStandbyYGyroEnabled(1);  //turn off gyro
    accelgyro.setStandbyZGyroEnabled(1);
    accelgyro.setWakeFrequency(2); // wake frequency 5hz
    
    //Initialize GPS
    Serial1.begin(9600);  Serial.println("Initialized GPS...");
    Serial.println(""); //cleanup serial console
    
    ubidots.setMethod(TYPE_UDP); //save data using UDP
    bat = fuel.getSoC(); //Get Battery level
    if (bat < 20) NapTime();

}

void loop(){
    
    // Get IMU Data and compute the ggggs
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accelVec = sqrt((ax*ax)+(ay*ay)+(az*az))/16384;
    if (accelVec > accelMax) accelMax = accelVec;
    
    GetGPS(); //poll GPS

    //Store GPS for next iteration
    latPrev = latNow;
    lonPrev = lonNow;
    
    // Check for movement
    if (latNow == 0){
        noMov = 0;
        noGPS++;
    }else{
        delta = haversine_km(latNow, lonNow, latPrev, lonPrev) * 1000;
        if(delta < moveThres){ 
            noMov++;
            noGPS = 0;
        }else{
            noMov = 0;
            noGPS = 0;
        }
    }
    
    //Serial Prints
    SerialPrints();
    
    //Publish Data to the Cloud
    if ( ((millis()-counter) >= (1000*cloudDur)) ){
        CloudPrints();
        counter = millis();
    }
    
    if (noMov >= moveNum) NapTime();// if no movement go to sleep
    if (noGPS >= noGPSDur) NapTime(); // if no GPS go to sleep
    
    delay(1000);
}


void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial1.write(MSG[i]);
  }
}

double GetGPS(){ //Pull in GPS data

    isValidGPS = false;
    for (unsigned long start = millis(); millis() - start < 1000;){
        // Check GPS data is available
        while (Serial1.available()){
            char c = Serial1.read();
            // parse GPS data
            if (gps.encode(c))
                isValidGPS = true;
        }
    }
    
    // get that new data
    latNow = gps.location.lat();
    lonNow = gps.location.lng(); 
    altNow = gps.altitude.meters();
    
    // while(Serial1.available())             // Make sure device serial is working
    // Serial.write((char)Serial1.read()); 
    
    //Check for incorrect GPS
    if (abs(latNow)>90 || abs(lonNow)>180){
        latNow = 0;
        lonNow = 0;
    }
  
}

int NapTime(){
    uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
    sendUBX(GPSoff, sizeof(GPSoff) / sizeof(uint8_t)); // Put the GPS in sleep mode
    sleep = true; //allow Cloud print to skip check
    CloudPrints(); //if haven't moved, don't pub
    Serial.print("Going to sleep for "); Serial.print(sleepDur/60/60); Serial.println(" hr(s)...");
    delay(1000);
    System.sleep(SLEEP_MODE_DEEP, sleepDur);
}

int SerialPrints(){
    Serial.print("Consecutive Non-movements: "); Serial.print(noMov,1); Serial.print("/"); Serial.println(moveNum);
    Serial.print("Consecutive No GPS Measurements: "); Serial.print(noGPS,1); Serial.print("/"); Serial.println(noGPSDur);
    Serial.print("GPS Location: "); Serial.print(latNow,6); Serial.print(", "); Serial.print(lonNow,6); Serial.print(", "); Serial.println(altNow,1);
    Serial.print("Last Pub GPS Location: "); Serial.print(latPub,6); Serial.print(", "); Serial.print(lonPub,6); Serial.println(", ");
    Serial.print("Distance from last publish:  "); Serial.print(delta,6); Serial.println(" m");
    Serial.println("");
    delay(100); //avoid overloading serial
    return 0;
}

int CloudPrints(){
    
    if (sleep == false) { //skip check if going to sleep

        if (delta <= distPubThresh){
            Serial.println(""); Serial.println("GPS Position hasn't moved since last publish"); Serial.println("");
            return 0; // no publishing if it hasn't moved.
        }

    }
    
    latPub = latNow; lonPub = lonNow; //store for next iteration
    bat = fuel.getSoC(); //Get Battery level
    Serial.print("Bat: "); Serial.print(bat,2); Serial.println(" %");
    
    //Check for connection
    if (Particle.connected() == false){ 
        Particle.connect(); 
        Serial.println("Connecting to the Cloud");
        waitFor(Particle.connected, 30000);
        delay(1000);
    }
    
    //Publish to ubidots
    Serial.println("Publishing to Cloud"); Serial.println("");
    sprintf(publishData, "lat=%.6f$lng=%.6f",latNow,lonNow);
    ubidots.add("Battery", bat, publishData); 
    ubidots.sendAll();
    return 0;
}

//calculate haversine distance for linear distance
double haversine_km(double lat1, double long1, double lat2, double long2){
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;
    return d;
}