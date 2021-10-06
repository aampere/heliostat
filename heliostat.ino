#include <RTClib.h>
#include <Stepper.h>
#include <IRremote.h>
#define ARDBUFFER 16
#include <stdarg.h>

struct Vector {
  float x;
  float y;
  float z;
};

//in pins
const int calibratepin = A0;
const int azplus = A1;
const int azminus = A2;
const int alplus = A3;
const int alminus = 10;
const int sdapin = A4; //SDA
const int sclpin = A5; //SCL
const int irpin = 2;

//out pins
const int m11 = 2; //azimuth motor
const int m12 = 3;
const int m13 = 4;
const int m14 = 5;
const int m21 = 6; //altitude motor
const int m22 = 7;
const int m23 = 8;
const int m24 = 9;

const float pi = 3.14159265359;
const int stepsperrev = 200;//2048;
int azstep = 0;
int alstep = 0;
int stepinc = 1;
int motorspeed = 1 ;
int slowstepdelay = 50; //ms delay used in slowstep method

//Big motor below:
Stepper azmotor(stepsperrev, 4,5,6,7);
Stepper almotor(stepsperrev, 8,9,10,11);

IRrecv irrecv(irpin);
decode_results results;
unsigned long key_value = 0;
const unsigned long calibratecode = 0xFF906F;//EQ
const unsigned long azpluscode = 0xFF02FD;//>>
const unsigned long azminuscode = 0xFF22DD;//<<
const unsigned long alpluscode = 0xFFA857;//+
const unsigned long alminuscode = 0xFFE01F;//-
const unsigned long updatecode = 0xFFC23D; //play/pause button

int mirrorupdateinterval = 15000; //1 minute
int updatetimer = 0;

Vector sunpos = {0,0,1};//It is the code's responsibility to NORMALIZE these any time they change! Then at all other times they can safely be assumed to be unit vectors.
Vector mirrorpos = {0,0,1};
Vector target = {0,0,1};
double latitude = 30.2672;
double longitude = -97.7431;
RTC_PCF8523 rtc;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);
  azmotor.setSpeed(motorspeed);
  almotor.setSpeed(motorspeed);
  pinMode(calibratepin, INPUT);
  pinMode(azplus, INPUT);
  pinMode(azminus, INPUT);
  pinMode(alplus, INPUT);
  pinMode(azminus, INPUT);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

void loop() {
  unsigned long buttoncode=0;
  if(irrecv.decode(&results)) {
    if(results.value == 0XFFFFFFFF){
      results.value = key_value;
    }
    buttoncode = results.value;
    Serial.println(results.value, HEX);
    key_value = results.value;
    irrecv.resume();
  } else {
    buttoncode = 0;
  }
  //moving the mirror with a control button is how you indicate the target vector.
  //You can also reposition the mirror to the calibration position, then press the calibrate button.
  if(buttoncode==azpluscode) {
    azstep = azstep + stepinc;
    azmotor.step(stepinc);
    //slowStep(azmotor,stepinc,slowstepdelay);
    mirrorpos = calculateMirrorPosFromSteps(azstep, alstep);
    sunpos = calculateSunPos(latitude, longitude);
    target = calculateReflectionRay(sunpos, mirrorpos);
    printInfo();
  } else if(buttoncode==azminuscode) {
    azstep = azstep - stepinc;
    azmotor.step(-stepinc);
    //slowStep(azmotor,-stepinc,slowstepdelay);
    mirrorpos = calculateMirrorPosFromSteps(azstep, alstep);
    sunpos = calculateSunPos(latitude, longitude);
    target = calculateReflectionRay(sunpos, mirrorpos);
    printInfo();
  } else if(buttoncode==alpluscode) {
    alstep = alstep + stepinc;
    //almotor.step(stepinc);
    slowStep(almotor,stepinc,slowstepdelay);
    mirrorpos = calculateMirrorPosFromSteps(azstep, alstep);
    sunpos = calculateSunPos(latitude, longitude);
    target = calculateReflectionRay(sunpos, mirrorpos);
    printInfo();
  } else if(buttoncode==alminuscode) {
    alstep = alstep - stepinc;
    //almotor.step(-stepinc);
    slowStep(almotor,-stepinc,slowstepdelay);
    mirrorpos = calculateMirrorPosFromSteps(azstep, alstep);
    sunpos = calculateSunPos(latitude, longitude);
    target = calculateReflectionRay(sunpos, mirrorpos);
    printInfo();
  } else if(buttoncode==calibratecode) {
    //Set the "zero" steps for both motors
    azstep = 0;
    alstep = 0;
    mirrorpos = calculateMirrorPosFromSteps(azstep, alstep);
    sunpos = calculateSunPos(latitude, longitude);
    target = calculateReflectionRay(sunpos, mirrorpos);
    printF("azstep ", azstep);
    printF("alstep ", alstep);
    printF("mirrorpos ", mirrorpos);
    printF("sunpos ", sunpos);
    printF("target ", target);
    delay(200);
  }
  delay(1);
  ++updatetimer;
  //possibly, if the user has recently moved the mirror, postpone update...
  if((false && updatetimer>mirrorupdateinterval) || buttoncode==updatecode) {
    updatetimer = 0;
    Vector oldsunpos = sunpos;
    Vector oldmirrorpos = mirrorpos;
    sunpos = calculateSunPos(latitude, longitude);
    mirrorpos = angleBisector(sunpos, target);
    int oldazstep = azstep;
    int oldalstep = alstep;
    stepsFromMirrorPos(); //This sets azstep and alstep
    //azmotor.step(azstep-oldazstep);
    slowStep(azmotor,azstep-oldazstep,slowstepdelay);
    //almotor.step(alstep-oldalstep);
    slowStep(almotor,alstep-oldalstep,slowstepdelay);
    Serial.println("Updating...");
    printInfo();
    Serial.println("");
  }
}

void printInfo() {
  printF("azstep ", azstep);
  printF("alstep ", alstep);
  printF("azs ", (float)azstep/stepsperrev*360.0);
  printF("als ", (float)alstep/stepsperrev*360.0);
  printF("mirrorpos ", mirrorpos);
}

float nDays(const int& y, const int& m, const int& d) {
  const float febdays = (y%4==0 && y%400!=0)?29:28;
  const float dom[12] = {31,febdays,31,30,31,30,31,31,30,31,30,31};
  float nd = 0;
  int i=1;
  while(i!=m) {
    nd = nd + dom[i-1];
    ++i;
  }
  nd += d;
  return nd;
}
float currentJulianDateTime() {
  //get time from rtc
  float y = 0;//utcdt.Year;
  float d = 0;//nDays(utcdt);
  float h = 0;//utcdt.Hour + utcdt.Minute/60 + utcdt.Second/3600;
  float delta = y-1949;
  float leap = floor(delta/4);
  float jdt = 2432916.5 + delta*365 + leap + d + h/24;
  return jdt;
}

Vector calculateSunPos(const float& lat, const float& lon) {
  //first calculate julian time. We do this here because we need the h term in the sunpos calculation
  //get time from rtc
  DateTime utcdt = rtc.now() + TimeSpan(0,5,0,0); //add 5 hours to get UTC time
  float y = utcdt.year();
  float d = nDays(utcdt.year(),utcdt.month(),utcdt.day());
  float h = (float)utcdt.hour() + (float)utcdt.minute()/60.0 + (float)utcdt.second()/3600.0;
  float delta = y-1949;
  float leap = floor(delta/4);
  float jdt = 2432916.5 + delta*365 + leap + d + h/24; //doesn't work wiht single precision
  //calculate sun position in cartesian coords, where (x, y, z) corresponds to E, N, Up
  //Almanac algorithm. Note that the terms altitude and elevation are interchangible. The algorithm uses "el" whereas I prefer "al" in the rest of the program.
  //Calculate right ascention and declination
  float n = -18628.5 + delta*365 + leap + d + h/24.0;
  float L = fmod(280.460 + 0.9856474*n, 360); // degrees
  float g = fmod(357.528 + 0.9856003*n, 360); // degrees
  float l = fmod(L + 1.915*sin(g/180*pi) + 0.020*sin(2*g/180*pi), 360); // degrees
  float ep = 23.439 - 0.0000004*n; // degrees
  float ra = atan2(cos(ep/180*pi)*sin(l/180*pi),cos(l/180*pi)); // radians
  if(ra<0)
     ra = ra + 2*pi; 
  float dec = asin(sin(ep/180*pi)*sin(l/180*pi)); //radians

  //Greenwich mean sidereal time
  float gmst = fmod(6.697375 + 0.0657098242*n + h, 24);

  //Local mean sidereal time
  float eastlong = fmod(lon, 360);
  float lmst = fmod(gmst + eastlong/15, 24);
  float lmstrad = lmst*15/180*pi;

  //Hour angle
  float ha = lmstrad - ra; // calculations in radians
  if(ha<-pi)
      ha = ha + 2*pi;
  else if(ha>pi)
      ha = ha - 2*pi;

  //Calculate elevation ("altitude") and azimuth
  //recall dec ra ha are all radians.
  //supply latitude and convert to radians
  float latrad = lat/180*pi;
  float el = asin(sin(dec)*sin(latrad) + cos(dec)*cos(latrad)*cos(ha));
  float az = asin(-cos(dec)*sin(ha)/cos(el));
  float elc = asin(sin(dec)/sin(latrad));
  if(el>=elc)
     az = pi-az;
  if(el<elc && ha>0)
     az = 2*pi + az;

  //Refraction
  float eldeg = el/pi*180;
  float refrac = 3.51823*(.1594+.0196*eldeg+.00002*pow(eldeg,2))/(1.+.505*eldeg+.0845*pow(eldeg,2)); //degrees, this is about 0.5 degrees at el=0, and <0.2deg for el>4
  float correctedel = el + refrac/180*pi; //refraction-corrected elevation in radians
  
  //Serial.print("Az,el: "+toStr(az/pi*180)+" "+toStr(el/pi*180)+"\n");
  Vector sp = {sin(az)*cos(el), cos(az)*cos(el), sin(el)}; //Note that the refraction corrected elevation is not used currently.
  return sp;
}

Vector calculateMirrorPosFromSteps(const int& azs, const int& als) {
  float az = (0.25+(float)azs/stepsperrev)*2*pi;
  float al = (0.25-(float)als/stepsperrev)*2*pi;
  Vector tar = {sin(az)*cos(al), cos(az)*cos(al), sin(al)};
  return tar;
}

Vector calculateReflectionRay(const Vector& sun, const Vector& mirror) {
  float costheta = sun.x*mirror.x + sun.y*mirror.y + sun.z*mirror.z;
  Vector proj = {costheta*mirror.x, costheta*mirror.y, costheta*mirror.z};
  Vector refl = {-sun.x+2*proj.x, -sun.y+2*proj.y, -sun.z+2*proj.z};
  return refl;
}

Vector angleBisector(const Vector& v1, const Vector& v2) {
  //returns the unit vector that bisects v1 and v2
  Vector bisector = {v1.x+v2.x, v1.y+v2.y, v1.z+v2.z};
  float mag = sqrt(bisector.x*bisector.x+bisector.y*bisector.y+bisector.z*bisector.z);
  bisector.x = bisector.x/mag;
  bisector.y = bisector.y/mag;
  bisector.z = bisector.z/mag;
  return bisector;
}

void stepsFromMirrorPos() {
  if(mirrorpos.x==0 && mirrorpos.y==0) {
    //Mirror is pointed up, any azimuth is suitable, so just keep the current azimuth
    azstep = azstep;
  } else {
    float az = atan2(mirrorpos.x, mirrorpos.y);
    azstep = (az/(2*pi)-0.25)*stepsperrev;
  }
  float al = asin(mirrorpos.z);
  float alfromup = 0.5*pi-al; //radians away from the up vector. This coresponds with alstep=0 for up
  alstep = alfromup/(2*pi)*stepsperrev;
}

String toStr(double f) {
  char str[7];
  dtostrf(f,7,3,str);
  //std::string str = st
  //snprintf(str,sizeof(str),"%.2f",f);
  return String(str);
}
void printF(String s, float f) {
  Serial.print(s);
  Serial.print(f,4);
  Serial.print("\n");
}
void printF(String s, Vector f) {
  Serial.print(s);
  Serial.print("(");
  Serial.print(f.x,4);
  Serial.print(", ");
  Serial.print(f.y,4);
  Serial.print(", ");
  Serial.print(f.z,4);
  Serial.print(")");
  Serial.print("\n");
}
String toStr(int i) {
  return String(i);
}
void slowStep(Stepper& m, int steps, int msdelay) {
  int sign = steps>0?1:-1;
  for(int i=0; i!=abs(steps); ++i) {
    m.step(sign);
    delay(msdelay);
  }
}
