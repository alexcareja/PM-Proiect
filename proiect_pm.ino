
#include <LCD5110_Graph.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define Hall_Sensor A0
#define Hall_Sensor_D 2
#define GPS_RX 3
#define GPS_TX 4
#define _GND 6

LCD5110 myGLCD(8,9,10,12,11);
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
SoftwareSerial hallSerial(Hall_Sensor, Hall_Sensor_D);
TinyGPSPlus gps;

extern unsigned char TinyFont[];
extern unsigned char SmallFont[];
float v = 0.0;
unsigned long d = 0;
int m1 = 0, m2 = 1;
unsigned long last_spin = 0;

ISR(INT0_vect)
{
  m1 = digitalRead(Hall_Sensor_D);
  if (m1 == 0 && m2 == 1) {
    unsigned long new_spin = millis();
    if (last_spin != 0) {
      v = 8.3692 / ((float) (new_spin - last_spin) / 1000.0);
      d += 2325; // millimeters
    }
    last_spin = new_spin;
  }
  m2 = m1;
}

void setup_interrupts() {
  // buton : PD2 / INT0
  cli();

  // input
  DDRD &= ~(1 << PD2);
  // input pullup
  PORTD |= (1 << PD2);

  // configurare intreruperi externe
  EICRA |= (1 << ISC01);    // set INT0 to trigger on falling edge (1-0)

  // activare intrerupere externa
  EIMSK |= (1 << INT0);     // Turns on INT0

  sei();
}

void print_speed() {
  char v_string[10];
  unsigned long new_time = millis();
  
  if (new_time - last_spin > 2000) {
    v = 0.0;
  }

  int v_int = (int) v, v_len = 3;

  while (v_int > 10) {
    v_int /= 10;
    v_len++;
  }
  
  dtostrf(v, v_len, 1, v_string) ;
  strcat(v_string, " km/h");
  myGLCD.print(v_string, CENTER, 8);
}

void print_distance() {
  char d_string[10];
  
  if (d > 1000000) {
    float df = d / 1000000.0;
    int d_len = 4;
    for (unsigned long i = 10000000; d > i; i *= 10) {
      d_len++;
    }
    dtostrf(df, d_len, 2, d_string) ;
    strcat(d_string, " km");
  } else {
    itoa(d / 1000, d_string, 10);
    strcat(d_string, " m");
  }
  myGLCD.print(d_string, CENTER, 17);
}

void print_time_elapsed() {
  char et[10];
  unsigned long t = millis() / 1000;
  int s, m, h;

  s = t % 60;
  m = (t % 3600) / 60;
  if (t < 3600) {
    sprintf(et, "%02d:%02d", m, s);
  } else {
    h = t / 3600;
    sprintf(et, "%02d:%02d:%02d", h, m, s);
  }
  myGLCD.print(et, CENTER, 26);
}

void print_kcal() {
  char kcal_string[10];
  unsigned long kcal = d / 24000;

  sprintf(kcal_string, "%d kcal", kcal);
  myGLCD.print(kcal_string, LEFT, 0);
}

void print_time() {
  char time[8];

  if (gps.time.isValid()) {
    sprintf(time, "%02d:%02d", (gps.time.hour() + 3) % 24, gps.time.minute());
    myGLCD.print(time, RIGHT, 0);
  } else {
    myGLCD.print("--:--", RIGHT, 0);
  }
}

void print_coord() {
  float lat, lng;
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
    myGLCD.printNumF(lat, 6, 0, 36, '.', 9, ' ');
    myGLCD.printNumF(lng, 6, 0, 42, '.', 9, ' ');
    
  } else {
    myGLCD.print("no gps", LEFT, 36);
    myGLCD.print("signal", LEFT, 42);
  }
  
}

void print_avg_speed() {
  unsigned long t = millis();
  float avg_v = 3.6 * (d * 1.0 / t);
  char v_string[10];
  int v_int = (int) avg_v, v_len = 3;

  while (v_int > 10) {
    v_int /= 10;
    v_len++;
  }
  
  dtostrf(avg_v, v_len, 1, v_string) ;
  strcat(v_string, " km/h");
  myGLCD.print(v_string, RIGHT, 42);
  
}

void print_altitude() {
  unsigned long alt;
  char alt_string[10];
  if (gps.altitude.isValid()) {
    alt = (unsigned long) gps.altitude.meters();
    sprintf(alt_string, "alt: %dm", alt);
    myGLCD.print(alt_string, RIGHT, 36);
  }
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

void setup() {
  myGLCD.InitLCD();
  randomSeed(analogRead(7));
  Serial.begin(9600);
  setup_interrupts();
  hallSerial.begin(9600);
  gpsSerial.begin(9600);
  digitalWrite (_GND, LOW); // ground set to 0V
  pinMode (_GND, OUTPUT);
}

void loop() {

  // Clear screen
  myGLCD.clrScr();
  
  myGLCD.setFont(SmallFont);

  print_distance();

  print_speed();

  print_time_elapsed();
  
  myGLCD.setFont(TinyFont);

  print_time();

  print_kcal();

  print_coord();
  
  print_avg_speed();

  print_altitude();
  
  myGLCD.update();

  int Val2 = digitalRead(Hall_Sensor_D);
  Serial.print("v: ");
  Serial.print(v);
  Serial.print("km/h\n");
  smartDelay(250);
}
