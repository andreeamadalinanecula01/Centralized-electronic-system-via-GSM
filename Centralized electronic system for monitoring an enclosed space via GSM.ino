#include <SoftwareSerial.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>


#define LED_GALBEN 6
#define LED_VERDE 7
#define RELEU 12
#define ALARMA_BUZZ A1
#define DHT11_tem_umid 8
#define VENTILATIE_IN1 10
#define VENTILATIE_IN2 11
#define LED_PRAF 5
#define PIN_PRAF A0

#define TX_GSM_RX_MC 3
#define RX_GSM_TX_MC 2

uint8_t flag_ventilatie = 0;
uint8_t ventilatie_pornita = 0;
uint8_t flag_releu = 0;
uint8_t bec_aprins = 0;
uint8_t flag_dht11 = 0;
uint8_t flag_light = 0;
uint8_t flag_praf = 0;
uint8_t primit = 0;
uint8_t contor = 0;
uint8_t alarma_temperatura = 0;
uint8_t alarma_umiditate = 0;

float densitate;
String temperatura, umiditate, int_lum_tsl;
byte temperature = 0;
byte humidity = 0;

SoftwareSerial mySerial(3, 2);  // creez seriala software sa comunic cu sim800

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);  // initializare senzor  TSL2561

SimpleDHT11 dht11(DHT11_tem_umid);


LiquidCrystal_I2C lcd(0x27, 16, 2);

//__________________________________________________________________________
// _________________________________SETUP____________________________________
//__________________________________________________________________________
void setup() {
  // configurare pini microcontroler
  pinMode(LED_GALBEN, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_PRAF, OUTPUT);
  pinMode(ALARMA_BUZZ, OUTPUT);
  pinMode(RELEU, OUTPUT);
  digitalWrite(RELEU, HIGH);  // configurare stare pini
  pinMode(VENTILATIE_IN1, OUTPUT);
  pinMode(VENTILATIE_IN2, OUTPUT);
  pinMode(DHT11_tem_umid, INPUT);

  // initializare senzori
  initializare_TSL2561();  // initializare senzor Intensitate luminoasa


  initializare_lcd();  // initializare lcd
  Serial.begin(9600);  // configurare baud rate seriala
  init_gsm();          // configurare modul gsm
}
//__________________________________________________________________________
// _________________________________LOOP____________________________________
//__________________________________________________________________________

void loop() {

  read_sms();  // citire mesaj primit prin gsm
 //updateSerial();
  if (primit == 1) {
    comanda_sms();
  } else {
    display_lcd(contor);
    if (contor == 1) {
      contor = 0;
    } else {
      contor++;
    }
  }

}  // end loop


//__________________________________________________________________________
// _________________________________functii_________________________________
//__________________________________________________________________________

//______________________________________________________________________
// FUNCTIE PENTRU CITIREA TEMPERATURII
void read_dht11() {

  dht11.read(&temperature, &humidity, NULL);

  temperatura = String(int(temperature));
  umiditate = String(int(humidity));
  //Serial.print(temperatura);
  //Serial.print(" *C, ");
  // Serial.print(umiditate);
  //Serial.println(" H");
}
//________________________________________________________________________
// functie initializare senzor TSL2561
void initializare_TSL2561() {
  tsl.enableAutoRange(true);  //auto-gain
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
}

//________________________________________________________________________
// functie citirea senzorului de intensitate luminoasa TSL2561
void read_TSL2561() {
  sensors_event_t intensitate;
  tsl.getEvent(&intensitate);
  int int_luminoasa = intensitate.light;
  int_lum_tsl = String(int_luminoasa);
  // Serial.print(int_lum_tsl);
  // Serial.println(" lux");
}
//______________________________________________________________________
// senzor intensitate praf

//~~~~~~GP2Y1014AU0F functie citire~~~~~
void read_senzor_pm2_5() {
  digitalWrite(LED_PRAF, LOW);
  delayMicroseconds(280);

  float Praf_VO = analogRead(PIN_PRAF);
  delayMicroseconds(40);

  digitalWrite(LED_PRAF, HIGH);
  delayMicroseconds(9600);  // sleep time

  float Praf_VO_Calc = Praf_VO * (5.0 / 1024.0);
  densitate = 0.17 * Praf_VO_Calc - 0.1;
  if (densitate < 0) {
    densitate = 0.00;
  }
  // Serial.print(densitate);  // mg
  // Serial.println(" mg");
}

//________________________________________________________
// initializare lcd
void initializare_lcd() {
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(" Initializare ");
  lcd.setCursor(3, 1);
  lcd.print(" sistem... ");
  delay(2000);
}

//________________________________________________________
// initializare Funcite pentru initializarea GSM

void init_gsm() {
  mySerial.begin(9600);
  Serial.println("Initializare GSM...");
  delay(2000);

  mySerial.println("AT");  
  updateSerial();

  mySerial.println("AT+CMGF=1");  // Configurarea modului TEXT
  updateSerial();
  mySerial.println("AT+CNMI=1,2,0,0,0");  // Se decide cum ar trebui gestionate mesajele SMS noi
  updateSerial();
}
//______________________________________________________________________
// functie pentru citirea serialei
void updateSerial() {
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read());  // Se redirecționează ce a fost primit prin Serial către portul Serial Software
  }
  while (mySerial.available()) {
    Serial.write(mySerial.read());  // Se redirecționează ce a fost primit prin Serial Software către portul Serial.
  }
}
//______________________________________________________________________

//______________________________________________________________________
// FUNCTIE PENTRU A TRIMITE MESAJ SMS
void send_sms(String mesaj_sms) {
  mySerial.println("AT");  
  updateSerial();

  mySerial.println("AT+CMGF=1");  // Configurarea modului TEXT
  updateSerial();
  mySerial.println("AT+CMGS=\"+40767539664\"");  
  updateSerial();
  mySerial.print(mesaj_sms);  
  updateSerial();
  mySerial.write(26);
}
//___________________________________________________________________________
// FUNCTIE PENTRU CITIREA MESAJULUI SMS
void read_sms() {
  primit = 0;
  while (mySerial.available()) {
    String sms_primit = mySerial.readString();
   // Serial.println(sms_primit); //afisare string primit de la gsm
    int index = sms_primit.indexOf("\"2");
    String comanda = sms_primit.substring(index + 24, index + 28);
    Serial.print("comanda:");
    Serial.println(comanda);

    // ----------VENTILATOR-----------
    if (comanda == "VEON") {
      flag_ventilatie = 1;
      Serial.println("VENTILATOR ON");
    }
    if (comanda == "VOFF") {
      flag_ventilatie = 0;
      Serial.println("VENTILATOR OFF");
    }
    // -------------actionare bulb?--------------
    if (comanda == "BUON") {
      flag_releu = 1;
      Serial.println("BEC ON");
    }
    if (comanda == "BOFF") {
      flag_releu = 0;
      Serial.println("BEC OFF");
    }
    // -----------se doreste citirea senzorului de temperatura si umiditate?----------
    if (comanda == "RDHT") {
      flag_dht11 = 1;
      Serial.println("READ DHT");
    }
    // --------SE DORESTE CITIREA NIVELULUI DE ILUMINARE? ---------
    if (comanda == "RLUM") {
      flag_light = 1;
      Serial.println("READ ILUMINARE");
    }

    // --------SE DORESTE CITIREA NIVELULUI DE ILUMINARE? ---------
    if (comanda == "RPRF") {
      flag_praf = 1;
      Serial.println("READ DENSITATE PRAF");
    }

    primit = 1;
  }
}

//_____________________________________________________________________
//_______________________________________________________________
// Se executa comanda primita prin sms
void comanda_sms() {
  // actionare RELEU
  if (flag_releu == 1) {
    if (bec_aprins == 0) {
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Se aprinde");
      lcd.setCursor(7, 1);
      lcd.print("becul...");
      delay(500);
      digitalWrite(RELEU, LOW);  // aprind bec ul
      bec_aprins = 1;
      delay(2000);
    }
  } else {
    bec_aprins = 0;
    digitalWrite(RELEU, HIGH);  // sting bec-ul
  }
  //--------------------------------
  // verificare ventilatie(on/off)
  if (flag_ventilatie == 1) {
    if (ventilatie_pornita == 0) {
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Se porneste");
      lcd.setCursor(3, 1);
      lcd.print("ventilatia...");
      delay(500);
      // pornesc ventilatia
      digitalWrite(VENTILATIE_IN1, LOW);
      digitalWrite(VENTILATIE_IN2, HIGH);
      ventilatie_pornita = 1;
      delay(2000);
    }
  } else {
    ventilatie_pornita = 0;  // ventilatia oprita
    // opresc ventilatia
    digitalWrite(VENTILATIE_IN1, HIGH);
    digitalWrite(VENTILATIE_IN2, HIGH);
  }
  //--------------------------------
  // se solicita informatii privind temperatura si umiditatea?
  if (flag_dht11 == 1) {
    digitalWrite(LED_GALBEN, HIGH);
    read_dht11();                                                      // se actualizeaza informatiile privind temperatua si umiditatea
    String dht11_info = temperatura + "*C" + " , " + umiditate + "%";  // se configureaza mesajul ce urmeaza a fi transmis
    send_sms(dht11_info);                                              // se trimite mesajul prin gsm
    flag_dht11 = 0;                                                    // se reseteaza flag-ul
    digitalWrite(LED_GALBEN, LOW);
  }

  //--------------------------------
  // se solicita informatii privind intensitatea luminoasa
  if (flag_light == 1) {
    digitalWrite(LED_GALBEN, HIGH);
    read_TSL2561();                           // se actualizeaza informatiile privind intensitatea luminoasa
    String light_info = int_lum_tsl + " lx";  // se configureaza mesajul ce urmeaza a fi transmis
    send_sms(light_info);                     // se trimite mesajul prin gsm
    flag_light = 0;                           // se reseteaza flag-ul
    digitalWrite(LED_GALBEN, LOW);
  }
  //--------------------------------
  // se solicita informatii privind nivelul densitatii de praf
  if (flag_praf == 1) {
    digitalWrite(LED_GALBEN, HIGH);
    read_senzor_pm2_5();                                // se actualizeaza informatiile privind intensitatea luminoasa
    String densitate_info = String(densitate) + " mg";  // se configureaza mesajul ce urmeaza a fi transmis
    send_sms(densitate_info);                           // se trimite mesajul prin gsm
    flag_praf = 0;                                      // se reseteaza flag-ul
    digitalWrite(LED_GALBEN, LOW);
  }
}

//__________________________________________________________________________________________________
// functie pentru afisarea pe LCD

//_____________________________________________________________________
// functie pentru afisarea datelor pe lcd
void display_lcd(uint8_t cnt) {
  if (cnt == 0) {
    // citire dht11
    read_dht11(); // actualizez valorile pentru temp si umid
    if ((int(temperature) > 38) || (int(humidity) > 70)) {
      digitalWrite(ALARMA_BUZZ, HIGH);  // PORNESC ALARMA
      delay(400);
      digitalWrite(ALARMA_BUZZ, LOW);  // PORNESC ALARMA
      digitalWrite(LED_VERDE, LOW);    // sting led ul verde
                                       // daca este temperatura depasita, trimit mesaj catre utilizator
      if (int(temperature) > 38) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp. ridicata!");
        lcd.setCursor(3, 1);
        lcd.print(temperature);
        lcd.print("(grade)");

        if (alarma_temperatura == 0) {
          send_sms("Prag temperatura depasit!");
          alarma_temperatura = 1;
        }

        delay(1000);
      }
      // daca este umiditatea depasita, trimit mesaj catre utilizator
      if (int(humidity) > 70) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Umid. Ridicata!");
        lcd.setCursor(6, 1);
        lcd.print(umiditate);
        lcd.print(" %");

        if (alarma_umiditate == 0) {
          send_sms("Prag umiditate depasit!");
          alarma_umiditate = 1;
        }
        delay(1000);
      }

    } else {
      read_dht11();// actualizez valorile pentru temp si umid
      delay(100);
      alarma_umiditate = 0;
      alarma_temperatura = 0;
      digitalWrite(ALARMA_BUZZ, LOW);
      digitalWrite(LED_VERDE, HIGH);
      // temperatura si umiditate
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperatura);
      lcd.print("(grade)");
      lcd.setCursor(0, 1);
      lcd.print("Umiditate: ");
      lcd.print(umiditate);
      lcd.print(" %");
      delay(2900);
    }
  }

  if (cnt == 1) {
    read_senzor_pm2_5();
    read_TSL2561();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("D_Praf: ");
    lcd.print(densitate);
    lcd.print(" mg");
    lcd.setCursor(0, 1);
    lcd.print("Int: ");
    lcd.print(int_lum_tsl);
    lcd.print(" lx");

    delay(3000);
  }
}
