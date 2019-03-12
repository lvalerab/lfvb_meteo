String VERSION_FIRM="1.0.2";

#include <Wire.h>
#include <Adafruit_Sensor.h>

/******************************************
 * LCD
 ******************************************/

#include <LiquidCrystal_I2C.h>

 LiquidCrystal_I2C lcd(0x3F,20,4);
 bool SalidaSerial=true;

 void lcdInit() {
  lcd.init();
  lcd.backlight();
  lcd.clear();  
  lcdPrint("Meteo GPS/GPRS",0,0);  
  lcdPrint("VER "+VERSION_FIRM,7,1);
  lcdPrint("By LFVB Software",0,2);
  lcdPrint("GPS GPRS LUZ BMP DHT",0,3);
  lcd.scrollDisplayLeft();
  delay(5000);  
 }

  void lcdPrint(String texto, int x, int y) {
    lcd.setCursor(x,y);
    lcd.print(texto);
    if(SalidaSerial) {
      Serial.println("("+String(x)+","+String(y)+") "+texto);
    };
  }

/*******************************
 * Medicion % de luz
 *******************************/

#define LUZPIN A0

bool PlotLuz=true;
 
void LuzInfo() {
  int luz=analogRead(LUZPIN);
  int valor=map(luz,0,1024,0,100);
  lcd.clear();
  lcdPrint("LUZ",0,0);
  lcdPrint("% "+String(valor),0,2);
  lcdPrint("R: "+String(luz),0,3);
  if(PlotLuz) {
    Serial.println(valor);
  }
  delay(2000);  
}

/*******************************+
 * PARA EL GPS
 *******************************/

#include <FuGPS.h>

FuGPS gps(Serial2);
bool gpsAlive = false;

void InitGps() {
  Serial2.begin(9600);
  //gps.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_DEFAULT);
  //gps.sendCommand(FUGPS_PMTK_CMD_COLD_START);
  //gps.sendCommand(FUGPS_PMTK_SET_NMEA_BAUDRATE_9600);
  //gps.sendCommand(FUGPS_PMTK_CMD_FULL_COLD_START);  
  gps.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);          
}

void GpsInfo() {
  // Valid NMEA message
  lcd.clear();
  lcdPrint("LECTURA GPS",0,0);
    if (gps.read())
    {
        // We don't know, which message was came first (GGA or RMC).
        // Thats why some fields may be empty.
        lcd.clear();
        
        gpsAlive = true;

        /*Serial.print("Quality: ");
        Serial.println(gps.Quality);

        Serial.print("Satellites: ");
        Serial.println(gps.Satellites);*/
        lcdPrint("GPS CALIDAD",0,0);
        lcdPrint("Q: "+String(gps.Quality)+" S: "+String(gps.Satellites),0,1);
        lcdPrint("AC: "+String(gps.Accuracy)+" AL: "+String(gps.Altitude),0,2);
        delay(2000);
        
        if (gps.hasFix() == true)
        {
            // Data from GGA message
            /*Serial.print("Accuracy (HDOP): ");
            Serial.println(gps.Accuracy);*/
            lcdPrint("GPS VELOCIDAD",0,0);
            lcdPrint("SP: "+String(gps.Speed)+" DIR: "+String(gps.Course),0,3);
            

            /*Serial.print("Altitude (above sea level): ");
            Serial.println(gps.Altitude);*/
            delay(2000);
            lcd.clear();
            // Data from GGA or RMC
            /*Serial.print("Location (decimal degrees): ");
            Serial.println("https://www.google.com/maps/search/?api=1&query=" + String(gps.Latitude, 6) + "," + String(gps.Longitude, 6));*/
            lcdPrint("POSICION GPS",0,0);
            lcdPrint("TM: "+String(gps.Hours)+":"+String(gps.Minutes)+":"+String(gps.Seconds),0,1);
            lcdPrint("LAT: "+String(gps.Latitude,6),0,2);
            lcdPrint("LON: "+String(gps.Longitude,6),0,3);
            delay(2000);
        }
      delay(2000);
      BMPInfo();
      delay(2000);
      DhtInfo();
      delay(2000);
      LuzInfo();
    } else {
      lcdPrint("SIN DATOS GPS",0,1);
      delay(2000);
      BMPInfo();
      delay(2000);
      DhtInfo();
      delay(2000);
      LuzInfo();
    }

    // Default is 10 seconds
    if (gps.isAlive() == false)
    {
        if (gpsAlive == true)
        {
            gpsAlive = false;
            /*Serial.println("GPS module not responding with valid data.");
            Serial.println("Check wiring or restart.");*/
            lcd.clear();
            lcdPrint("GPS",0,0);
            lcdPrint("GPS no responde",0,2);
            delay(2000);
        }
    }
     
}

/**********************************************
 * PARA EL DHT11
 */
#include <DHT.h>
#include <DHT_U.h>
 
#define DHTPIN 3
#define DHTTYPE DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);

int8_t notice;

uint32_t InitDHT() {
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  return sensor.min_delay / 1000;
}

void DhtInfo() {
  //InitDHT();
  delay(InitDHT());
  lcd.clear();
  lcdPrint("DHT",0,0);
  sensors_event_t event;
  //Temperatura
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    lcdPrint("Error Temperatura",0,2);
  }
  else {
    lcdPrint("T: "+String(event.temperature,2)+" C",0,2);    
  }
  //Humedad
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    lcdPrint("Error Humedad",0,3);
  }
  else {
    lcdPrint("H: "+String(event.relative_humidity,2)+" %",0,3);
  }
}
/***********************************************
 * PARA EL SENSOR BMP280
 */


#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
bool BMPInicializado=false;


void InitBMP() {
  lcd.clear();
  lcdPrint("Inicializando BMP",0,1);
  if(!bmp.begin()) {
    /*Serial.println("No se ha podido establecer contacto con el sensor BMP280 "+String(BMP280_ADDRESS));*/
    
    lcdPrint("No inicializado BMP", 0, 2);    
    delay(1000);
  } else {
    lcdPrint("BMP Correcto", 0, 2);    
    BMPInicializado=true;
  }
}

void BMPInfo() {
  lcd.clear();
  lcdPrint("BME180 status", 0, 0);
  if(!BMPInicializado) {
    InitBMP();
  } else {
    lcdPrint("Temperatura: "+String(bmp.readTemperature(),6)+" *C", 0, 1);
    float mmHg=(((float)bmp.readPressure()+0)/101325)*760;
    float mmHgnm=(((float)bmp.readSealevelPressure()+0)/101325)*760;
    Serial.println(mmHg);
    Serial.println(mmHgnm);
    
    lcdPrint("P: "+String(mmHg,2)+" mmHg",0,2);
    lcdPrint("Pnm: "+String(mmHgnm,2)+" mmHg",0,3);
    delay(1000);
    lcdPrint("Altitud: "+String(bmp.readAltitude(),2)+" m",0,3);
    //lcdPrint("Presion (N.M) "+String(bmp.readSealevelPressure(),6)+" Pa",0,3);
    
    /*Serial.println("Temperatura: "+String(bmp.readTemperature(),6)+" *C");
    Serial.println("Presion: "+String(bmp.readPressure(),6)+" Pa");
    Serial.println("Altitud: "+String(bmp.readAltitude(1013.25),6)+" m");  */  
    delay(2000);
  }
}

/****************************************************
 * Moudlo SIM800L
 */
#include <Adafruit_FONA.h>
char* CONFIG_SMS_DEST="+34645190094";

Adafruit_FONA telf=Adafruit_FONA(10);

bool HayTelf=false;
int numMen=0;

void InitTelf() {
  lcd.clear();
  lcdPrint("TELFONIA",0,0);
  if(telf.begin(Serial3)) {
    HayTelf=true;
    lcdPrint("GSM OK",0,2);
  } else {
    lcdPrint("GSM NOK",0,2);
  }
  delay(2000);
}

void EnviarSMS(String texto) {
  lcdPrint("TELFONIA",0,0);
  if(numMen>=3) {
    if(HayTelf) {
      char* aux;
      texto.toCharArray(aux,texto.length()-1);
      if(telf.sendSMS(CONFIG_SMS_DEST,aux)) {
        lcdPrint("SMS ENVIADO",0,2);
        numMen++;
      } else {
        lcdPrint("SMS NO ENVIADO",0,2);
      }
    } else {
      lcdPrint("SMS NO ENVIADO",0,2);
      lcdPrint("NO TLF",0,3);
    }
  } else {
    lcdPrint("SMS NO ENVIADO",0,2);
    lcdPrint("M.N.M",0,3);
  }
  delay(2000);
}

/**************************************************+
 * Cuerpo del programa
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  InitGps();
  lcdInit();
  InitBMP();
  //InitDHT();
  //InitTelf();
  //EnviarSMS("Iniciado sistema");
  delay(1000);
}

void loop() {  
  GpsInfo();
  //DhtInfo();
}
