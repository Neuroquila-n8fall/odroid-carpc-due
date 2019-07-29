#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Keyboard.h>
#include <Mouse.h>

//Opto 2 - Zündung Aktiv
const int PIN_IGNITION_INPUT = 6;
//Eingang Odroid Power Button. Wenn dieser HIGH ist, ist der Odroid aktiv.
const int PIN_ODROID_POWER_INPUT = 7;
//Ausgang Odroid Power Button
const int PIN_ODROID_POWER_BUTTON = 8;
//Power Button vom Display
const int PIN_ODROID_DISPLAY_POWER_BUTTON = 9;
//I2C Receiver Addresse
const int WIRE_ADDRESS = 0x01;
//CAN CS Pin
const int SPI_CS_PIN = 10;
//Pin für Display Helligkeit
const int PIN_VU7A_BRIGHTNESS = 5;
//Pin für Debug Switch
const int PIN_DEBUG = 53;

/*

CAN Wiring:

<CPU is here>

Reset SCK   MISO
.     .     .       
.     .     .       
GND   MOSI  5V      

*/

//2 Sekunden für Aufwecken
const int ODROID_BOOT_HOLD_DELAY = 2100;
//5 Sekunden zum Herunterfahren
const int ODROID_SHUTDOWN_HOLD_DELAY = 5100;

/*
      Tastenbefehle für Odroid Settings
      Momentan ist es notwendig diese Tasten in der /vendor/usr/keylayout/Generic.kl abzuändern um die gewünschte Funktion zu besitzen
*/
const byte MUSIC_NEXT_KEY = KEY_F1;
const byte MUSIC_PREV_KEY = KEY_F2;
const byte MUSIC_FASTFORWARD_KEY = KEY_F3;
const byte MUSIC_REWIND_KEY = KEY_F4;
const byte VOICE_BUTTON = KEY_F7;
const byte PICKUP_BUTTON = KEY_F8;

int odroidRunning = LOW; //Ergebnis vom Odroid GPIO #1. LOW = aus, HIGH = an

int lastIgnitionState = HIGH; //Hält den letzten Zündungsstatus

const int CYCLE_DELAY = 500; //Verzögerung in ms pro Schleifendurchlauf

unsigned long previousOdroidActionTime = 0;  //Vorherige Zeitmessung für Odroid Steuerung
unsigned long previousMainTaskTime = 0;      //Vorherige Zeitmessung für allgemeinen Timer
unsigned long previousIgnitionCheckTime = 0; //Vorherige Zeitmessung für Zündungsstatus
unsigned long previousOdroidPauseTime = 0;   //Vorherige Zeitmessung für Odroid Sleepmodus

bool odroidStartRequested = false;    //Start von Odroid angefordert
bool odroidShutdownRequested = false; //Stop von Odroid angefordert
bool odroidPauseRequested = false;    //Sleep oder Wakeup angefordert

bool startup = true; //Steuerung ist gerade angelaufen.

bool debugMode = false; //Debugmodus aktiv?

int ignitionOn = HIGH; //Zündung - HIGH = Aus, LOW = An

const int ODROID_STANDBY_HOLD_DELAY = 100;          //Button Press für Display und Sleep.
const unsigned long WAKEUP_WAIT_DELAY = 10000;      //10 Sekunden Wartezeit für Aufwecken
const unsigned long STARTUP_WAIT_DELAY = 60000;     //Wartezeit für Start
const unsigned long SHUTDOWN_WAIT_DELAY = 60000;    //Wartezeit für Herunterfahren
unsigned long startPowerTransitionMillis = 0;       //Counter für den Aufweck- und Herunterfahrprozess
const unsigned long ODROID_STANDBY_DELAY = 5000;    //Wartzeit für Sleepfunktion

const int serialBaud = 115200;

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

int lastBrightness = 0; //zuletzt errechneter Helligkeitswert für Display.

unsigned long lastMflPress = 0; //Debounce für MFL Knopfdruck
bool MflButtonNextHold = false; //Ob der MFL Knopf "Next" gehalten wird
bool MflButtonPrevHold = false; //Ob der MFL Knopf "Prev" gehalten wird


//Stunden
int hours = 0;
//Minuten
int minutes = 0;
//Sekunden
int seconds = 0;
//Tag
int days = 0;
//Monat
int month = 0;
//Jahr
int year = 0;
//Langer Zeitstempel
char timeStamp[20] = "00:00:00 00.00.0000";
//Uhrzeit als Text
char timeString[9] = "00:00:00";
//Datum als Text
char dateString[11] = "00.00.0000";

//Zeit seit dem letzten Empfangen einer Uhrzeitnachricht über CAN
unsigned long previousCanDateTime = 0;

//Initialstatus der eingebauten LED
int ledState = LOW;

//Zeitstempel für Sekundentimer
unsigned long previousOneSecondTick = 0;

//Das muss eventuell nach dem Wakeup gesendet werden, damit der Controller anfängt Drehungspositionen zu schicken. Kann gut sein, dass das bereits automatisch passiert.
unsigned char IDRIVE_CTRL_WAKEUP[8] = {0x1D, 0xE1, 0x00, 0xF0, 0xFF, 0x7F, 0xDE, 0x00};

//Mögliche Aktionen
enum PendingAction
{
  ODROID_START,
  ODROID_STOP,
  ODROID_STANDBY,
  NONE
};
//Beinhaltet die aktuelle Aktion, welche ausgeführt wird oder werden soll.
//Sofern diese nicht NONE ist, können keine weiteren Aktionen ausgeführt werden.
//Dies soll doppelte Ausführungen von Start & Stop während der Hoch- und Herunterfahrphase des PCs verhindern
PendingAction pendingAction = NONE;

//iDrive Stuff
enum iDriveRotationDirection
{
  ROTATION_RIGHT,
  ROTATION_LEFT,
  UNCHANGED
};

//iDrive Knopf gedreht?
bool iDriveRotChanged = false;
//Zuletzt gesicherte Drehung des Knopfes um Richtung zu bestimmen
int iDriveRotLast = 0;
//Drehungs-counter
int iDriveRotCountLast = 0;
//Drehrichtung
iDriveRotationDirection iDriveRotDir = UNCHANGED;

//Status der Zündung abrufen und entsprechende Aktionen auslösen
void checkIgnitionState();
//Start bzw. Aufwecken
void startOdroid();
//Sofortiges geordnetes Herunterfahren
void stopOdroid();
//Odroid in Sleep versetzen und Display ausschalten
void pauseOdroid();
//Ein- und Ausgänge prüfen
void checkPins();
//CAN Nachrichten prüfen
void checkCan();
//Zeitstempel bauen
void buildtimeStamp();
//CAN Nachrichten auf der Konsole ausgeben
void printCanMsg(int canId, unsigned char *buffer, int len);
//CAN Output als CSV
void printCanMsgCsv(int canId, unsigned char *buffer, int len);
//Mausrad simulieren, je nachdem in welche Richtung der iDrive Knopf gedreht wurde.
void scrollScreen();
//Uhrzeit pflegen. Ist ausschließlich dazu da die Uhrzeit voran schreiten zu lassen, wenn der Canbus inaktiv ist und keine Zeit vom Auto kommt.
//Die RTC Library kommt leider nicht in Frage da mein DUE board wohl keinen Kristall für RTC hat und daher der MCU einfriert beim initialisieren.
void timeKeeper();

void setup()
{
  digitalWrite(LED_BUILTIN, LOW);
  Wire.begin(WIRE_ADDRESS); //I2C-Init
  Serial.begin(115200);
  Keyboard.begin();
  Mouse.begin();

  buildtimeStamp();

  pinMode(PIN_IGNITION_INPUT, INPUT_PULLUP);        //Zündungs-Pin
  pinMode(PIN_ODROID_POWER_BUTTON, OUTPUT);         //Opto 2 - Odroid Power Button
  pinMode(PIN_ODROID_POWER_INPUT, INPUT_PULLUP);    //Odroid VOUT Pin als Rückmeldung ob der PC eingeschaltet ist
  pinMode(PIN_ODROID_DISPLAY_POWER_BUTTON, OUTPUT); //Opto 3 - Display Power Button
  pinMode(LED_BUILTIN,OUTPUT);                      //LED
  pinMode(PIN_DEBUG, INPUT_PULLUP);                 //Debug Switch Pin
  pinMode(PIN_VU7A_BRIGHTNESS, OUTPUT); //Display Helligkeitssteuerung

  //So lange versuchen das modul zu initialisieren, bis es klappt.
  while (CAN_OK != CAN.begin(CAN_100KBPS))
  {
    Serial.println("[setup] CAN BUS Shield init fail");
    Serial.println("[setup] Init CAN BUS Shield again");
    delay(1000);
  }
  Serial.println("[setup] CAN BUS Shield init ok!");

  //Display von ganz dunkel nach ganz Hell stellen. Quasi als Test
  for (int i = 0; i <= 255; i++)
  {
    analogWrite(PIN_VU7A_BRIGHTNESS, i);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  //Aktuelle Zeit
  unsigned long currentMillis = millis();
  if (startup)
  {
    //Wenn die Steuerung mit aktiver Zündung startet oder resettet, sollte der Odroid starten
    //Aber nur wenn der Odroid AUS ist aber die Zündung an
    //Wenn der Odroid im Sleep ist, ist odroidRunning ebenfalls Low. Auf diese Art wird er bei Zündung direkt aufgeweckt.
    if (odroidRunning == LOW && ignitionOn == LOW && odroidStartRequested == false)
    {
      Serial.println("[STARTUP] PC aus, Zündung an --> Starte Pc...");
      startOdroid();
    }
    //Start beendet.
    startup = false;
  }

  //Start angefordert
  if (odroidStartRequested)
  {
    if (currentMillis - previousOdroidActionTime >= ODROID_BOOT_HOLD_DELAY)
    {
      odroidStartRequested = false;
      //Zeit merken
      previousOdroidActionTime = currentMillis;
      //Ausgang freigeben
      digitalWrite(PIN_ODROID_POWER_BUTTON, LOW);
      Serial.println("[odroidStartRequested] Start erfolgt.");
      //Aktuellen Counter merken. Ausführung weiterer Aktionen wird pausiert, bis abgewartet wurde.
      startPowerTransitionMillis = millis();
    }
  }
  //Stop angefordert
  if (odroidShutdownRequested)
  {
    if (currentMillis - previousOdroidActionTime >= ODROID_SHUTDOWN_HOLD_DELAY)
    {
      odroidShutdownRequested = false;
      //Zeit merken
      previousOdroidActionTime = currentMillis;
      //Ausgang freigeben
      digitalWrite(PIN_ODROID_POWER_BUTTON, LOW);
      Serial.println("[odroidShutdownRequested] Stop erfolgt.");
      //Aktuellen Counter merken. Ausführung weiterer Aktionen wird pausiert, bis abgewartet wurde.
      startPowerTransitionMillis = millis();
    }
  }

  if (odroidPauseRequested)
  {
    if (currentMillis - previousOdroidPauseTime >= ODROID_STANDBY_DELAY)
    {
      odroidPauseRequested = false;
      Serial.println("[odroidPauseRequested] Standby erfolgt.");
      //Aktuellen Counter merken. Ausführung weiterer Aktionen wird pausiert, bis abgewartet wurde.
      startPowerTransitionMillis = millis();
    }
  }

  //CAN Nachrichten verarbeiten
  checkCan();

  //1-Second timer
  if (currentMillis - previousOneSecondTick >= 1000)
  {
    //Heartbeat
    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
    previousOneSecondTick = currentMillis;
    //Aktualisieren.
    timeKeeper();
    //Zeitstempel Variablen füllen
    buildtimeStamp();
  }

  bool anyPendingActions = odroidStartRequested || odroidShutdownRequested || odroidPauseRequested;

  //Allgemeine Funktionen. Nur ausführen, wenn Zyklus erreicht wurde und keine ausstehenden Aktionen laufen, die ein zeitkritisches Verändern der Ausgänge beinhalten.
  if (currentMillis - previousMainTaskTime >= CYCLE_DELAY && !anyPendingActions)
  {
    //Zündung überprüfen
    checkIgnitionState();
    //Ein- und Ausgänge überprüfen
    checkPins();

    //Zeit merken
    previousMainTaskTime = currentMillis;
    //Sperrung freigeben, wenn Timeout abgelaufen ist
    switch (pendingAction)
    {
    case ODROID_STOP:
      if (currentMillis - startPowerTransitionMillis >= SHUTDOWN_WAIT_DELAY)
      {
        Serial.println("[LOOP] Shutdown Wartezeit abgelaufen");
        pendingAction = NONE;
      }
      break;
    case ODROID_START:
      if (currentMillis - startPowerTransitionMillis >= STARTUP_WAIT_DELAY)
      {
        Serial.println("[LOOP] Start Wartezeit abgelaufen");
        pendingAction = NONE;
      }
      break;
    case ODROID_STANDBY:
      if (currentMillis - startPowerTransitionMillis >= ODROID_STANDBY_DELAY)
      {
        Serial.println("[LOOP] Stand-by Wartezeit abgelaufen");
        pendingAction = NONE;
      }
      break;

    default:
      //Keine Aktion aktiv.
      break;
    }
  }
}

void checkCan()
{
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long currentMillis = millis();

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);

    unsigned int canId = CAN.getCanId();

    //Alle CAN Nachrichten ausgeben, wenn debug aktiv.
    if(debugMode)
    {
      printCanMsgCsv(canId,buf,len);
    }
    
    switch (canId)
    {
    //MFL Knöpfe
    case 0x1D6:
    {
      //Kein Knopf gedrückt (alle 100ms)
      if (buf[0] == 0xC0 && buf[1] == 0x0C)
      {
        //Knöpfe zurücksetzen
        MflButtonNextHold = false;
        MflButtonPrevHold = false;
        Keyboard.release(MUSIC_NEXT_KEY);
        Keyboard.release(MUSIC_PREV_KEY);
        Keyboard.release(MUSIC_FASTFORWARD_KEY);
        Keyboard.release(MUSIC_REWIND_KEY);
      }
      //Next
      if (buf[0] == 0xE0 && buf[1] == 0x0C)
      {
        /*
          Sowohl beim Drücken als auch beim Loslassen wird eine Nachricht geschickt.
          Wenn der Knopf das erste Mal gedrückt wird, wird keine Aktion ausgeführt.
          Die Zeit des Drückens wird gemessen. Erst wenn das Signal erneut innerhalb einer Sekunde kommt (Knopf losgelassen) wird reagiert.
          TODO: Es muss geprüft werden ob der Knopf immer wieder gesendet wird, wenn er gehalten wird.
        */
        //Der Knopf wurde innerhalb einer Sekunde losgelassen
        if (currentMillis - lastMflPress < 1000)
        {

          Serial.print("NEXT\n");
          Keyboard.press(MUSIC_NEXT_KEY);
          delay(200);
          Keyboard.releaseAll();
        }
        else
        {
          //Knopf wird gehalten
          Keyboard.press(MUSIC_FASTFORWARD_KEY);
          MflButtonNextHold = true;
        }
        lastMflPress = currentMillis;
      }
      //Prev
      if (buf[0] == 0xD0 && buf[1] == 0x0C)
      {
        //Der Knopf wurde innerhalb einer Sekunde losgelassen
        if (currentMillis - lastMflPress < 1000)
        {

          Serial.print("PREV\n");
          Keyboard.press(MUSIC_PREV_KEY);
          delay(200);
          Keyboard.releaseAll();
        }
        else
        {
          //Knopf wird gehalten
          Keyboard.press(MUSIC_FASTFORWARD_KEY);
          MflButtonPrevHold = true;
        }
        lastMflPress = currentMillis;
        lastMflPress = currentMillis;
      }
      //Pickup Button
      if (buf[0] == 0xC1 && buf[1] == 0x0C)
      {
      }
      //Voice Button
      if (buf[0] == 0xC0 && buf[1] == 0x0D)
      {
      }
      break;
    }
    //CAS: Schlüssel & Zündung
    case 0x130:
    {
      //Wakeup signal
      if (buf[0] == 0x45)
      {
        //TODO: Prüfen ob ideses Signal vom CI bereits gesendet wird!
        //Controller vorgaukeln, dass das CIC da ist.
        //CAN.sendMsgBuf(0x273,0,8,IDRIVE_CTRL_WAKEUP);
      }
      break;
    }
      //CIC
    case 0x273:
    {

      Serial.print("CIC\t");
      printCanMsg(canId, buf, len);
    }
    //CAS: Schlüssel Buttons
    case 0x23A:

      //Öffnen:     00CF01FF
      if (buf[0] == 0x00 && buf[1] == 0x30 && buf[2] == 0x01 && buf[3] == 0x60)
      {

        Serial.print("START\n");
        startOdroid();
      }
      //Schließen:  00DF40FF
      if (buf[0] == 0x00 && buf[1] == 0x30 && buf[2] == 0x04 && buf[3] == 0x60)
      {

        Serial.print("STOP\n");
        stopOdroid();
      }
      //Kofferraum: Wird nur gesendet bei langem Druck auf die Taste.

      break;
    //Licht-, Solar- und Innenraumtemperatursensoren
    case 0x32E:
    {
      /*  
          Lichtsensor auf Byte 0: Startet bei 0, in Praller Sonne wurde 73 zuletzt gemeldet.
          In der Dämmerung tauchen werte niedriger als 2 auf. Selbst das Parken am helligsten Tag unter einem Baum wirft Werte um 2 aus.
          Das bedeutet, dass bei Lichteifnall von der Seite das Display abdunkelt und es unleserlich wird. Das kann sehr gut anhand der Armaturenbeleuchtung beobachtet werden.
          Da es sich aber bei der Armaturenbeleuchtung um ein invertiertes Dot-Matrix LCD Display handelt, ist dies sogar unter direkter Sonneneinstrahlung perfekt lesbar.
      */
      int lightValue = buf[0];

      //Display auf volle Helligkeit einstellen. Das ist unser Basiswert
      int val = 255;

      //Bei wenig Licht abdimmen
      if (lightValue < 3)
      {
        val = 150;
      }

      //Deaktiviert. Ist zu dunkel im Schatten...
      /*       //Wenn keine Lichtdaten vorhanden sind oder es wirklich stockfinster ist...
      if (lightValue == 0)
      {
        val = 50;
      } */

      //Wenn der Wert unverändert ist, nichts tun.
      if (val == lastBrightness)
      {
        break;
      }

      //Wenn der aktuelle Wert größer als der zuletzt gespeicherte ist, zählen wir vom letzten Wert hoch.
      if (val > lastBrightness)
      {
        for (int i = lastBrightness; i <= val; i++)
        {
          analogWrite(PIN_VU7A_BRIGHTNESS, i);
          delay(10);
        }
      }

      //Wenn der aktuelle Wert kleiner als der zuletzt gespeicherte ist, dann schrittweise die Helligkeit vom letzten bekannten Wert absenken
      if (val < lastBrightness)
      {
        for (int i = lastBrightness; i >= val; i--)
        {
          analogWrite(PIN_VU7A_BRIGHTNESS, i);
          delay(10);
        }
      }
      //letzten Wert zum Vergleich speichern
      lastBrightness = val;

      //Ausgabe auf Konsole
      Serial.print("Helligkeit (Roh, Steuerwert):");
      Serial.print(String(lightValue, DEC));
      Serial.print('\t');
      Serial.print(val);
      Serial.println();
      break;
    }
    //Steuerung für Helligkeit der Armaturenbeleuchtung
    case 0x202:
    {
      //254 = AUS
      //Bereich: 0-253
      //Ab und zu wird 254 einfach so geschickt...
      Serial.print("Beleuchtung (Roh, Ctrl):");
      int dimRawVal = buf[0];
      int dimBrightness = map(dimRawVal,0,253,0,100);
      if(buf[0] == 254)
      {
        Serial.println("AUS = 254");
        break;
      }
      Serial.print(buf[0]);
      Serial.print(',');
      Serial.println(dimBrightness);
    }
    //Rückspiegel und dessen Lichtsensorik
    case 0x286:
    {
    }
    //iDrive Controller

    //iDrive Controller: Drehung
    case 0x264:
    {
      //Byte 0 beinhaltet den counter
      //Byte 1 zählt je nach Drehrichtung von 0 nach 254 hoch
      //   Ich vermute aktuell mal, dass wenn 254 erreicht wurde der Wert wieder auf 0 zurückspringt

      //Grundsätzliche bestimmung ob der Knopf überhaupt mal gedreht wurde
      iDriveRotChanged = iDriveRotCountLast != buf[0];

      //Etwas hat sich an der Drehrichtung geändert
      if (iDriveRotLast != buf[1])
      {
        iDriveRotChanged = true;
        //Sonderfälle bearbeiten
        //Knopf war am logischen Anschlag und springt jetzt wieder auf 0 zurück.
        if (iDriveRotLast == 254 && buf[1] == 0)
        {
          iDriveRotDir = ROTATION_RIGHT;
          iDriveRotLast = buf[1];
          break;
        }
        //Umgekehrte Drehrichtung
        if (iDriveRotLast == 0 && buf[1] == 254)
        {
          iDriveRotDir = ROTATION_LEFT;
          iDriveRotLast = buf[1];
          break;
        }
        //Wenn der zuletzt gespeicherte Wert größer als der neue ist, dann wurde der Knopf links gedreht
        //Umgekehrt wurde er Rechts gedreht
        if (iDriveRotLast > buf[1])
        {
          iDriveRotDir = ROTATION_LEFT;
        }
        else
        {
          iDriveRotDir = ROTATION_RIGHT;
        }
        //Wert speichern
        iDriveRotLast = buf[1];
      }
      else
      {
        iDriveRotChanged = false;
      }
      break;
    }
    //Knöpfe und Joystick
    case 0x267:
    {
      if (buf[4] != 0xDD)
      {
        switch (buf[5])
        {
        //Joystick oder Menüknopf
        case 0x01:
          switch (buf[3])
          {
          //Kurz gedrückt
          case 0x01:
            Keyboard.press(KEY_RETURN);
            break;
          //Lang gedrückt
          case 0x02:
            Keyboard.press(KEY_LEFT_ALT);
            Keyboard.press(KEY_TAB);
            break;
          //Losgelassen
          case 0x00:
            Keyboard.releaseAll();
            break;
          }
          break;
          //BACK Button
        case 0x02:
          switch (buf[3])
          {
          //Kurz gedrückt
          case 0x01:
            Keyboard.press(KEY_LEFT_CTRL);
            Keyboard.press(KEY_BACKSPACE);
            break;
          //Lang gedrückt
          case 0x02:
            break;
          //Losgelassen
          case 0x00:
            Keyboard.releaseAll();
            break;
          }
          break;
          //OPTION Button
        case 0x04:
          switch (buf[3])
          {
          //Kurz gedrückt
          case 0x01:
            break;
          //Lang gedrückt
          case 0x02:
            break;
          //Losgelassen
          case 0x00:
            Keyboard.releaseAll();
            break;
          }
          break;
          //RADIO Button
        case 0x08:
          switch (buf[3])
          {
          //Kurz gedrückt
          case 0x01:
            break;
          //Lang gedrückt
          case 0x02:
            break;
          //Losgelassen
          case 0x00:
            Keyboard.releaseAll();
            break;
          }
          break;
          //CD Button
        case 0x10:
          switch (buf[3])
          {
          //Kurz gedrückt
          case 0x01:
            break;
          //Lang gedrückt
          case 0x02:
            break;
          //Losgelassen
          case 0x00:
            Keyboard.releaseAll();
            break;
          }
          break;
          //NAV Button
        case 0x20:
          switch (buf[3])
          {
          //Kurz gedrückt
          case 0x01:
            break;
          //Lang gedrückt
          case 0x02:
            break;
          //Losgelassen
          case 0x00:
            Keyboard.releaseAll();
            break;
          }
          break;
          //TEL Button
        case 0x40:
          switch (buf[3])
          {
          //Kurz gedrückt
          case 0x01:
            break;
          //Lang gedrückt
          case 0x02:
            break;
          //Losgelassen
          case 0x00:
            Keyboard.releaseAll();
            break;
          }
          break;
        default:
          break;
        }
      }
      else
      {
        switch (buf[3])
        {
          //Hoch (kurz)
        case 0x11:
          Keyboard.press(KEY_UP_ARROW);
          break;
          //Hoch (lang)
        case 0x12:
          break;
        //Rechts (kurz)
        case 0x21:
          Keyboard.press(KEY_RIGHT_ARROW);
          delay(100);
          Keyboard.releaseAll();
          break;
        //Rechts (lang)
        case 0x22:
          break;
        //Runter (kurz)
        case 0x41:
          Keyboard.press(KEY_DOWN_ARROW);
          delay(100);
          Keyboard.releaseAll();
          break;
        //Runter (lang)
        case 0x42:
          break;
        //Links (kurz)
        case 0x81:
          Keyboard.press(KEY_LEFT_ARROW);
          delay(100);
          Keyboard.releaseAll();
          break;
        //Links (lang)
        case 0x82:
          break;
        default:
          break;
        }
      }

      break;
    }
    //PDC
    case 0x1C2:
    {
      //Byte 0~3 = Hinten
      //Byte 4~7 = Vorne
      //Angaben in cm von 0 - 255
      //Heck:
      int backOuterLeft = buf[0];
      int backInnerLeft = buf[1];
      int backInnerRight = buf[2];
      int backOuterRight = buf[3];
      //Front:
      int frontOuterLeft = buf[4];
      int frontInnerLeft = buf[5];
      int frontInnerRight = buf[6];
      int frontOuterRight = buf[7];

      break;
    }
    //Rückwärtsgang
    case 0x3B0:
    {
      if (buf[0] == 0xFD)
      {
        //Rückwärtsgang NICHT aktiv
      }
      if (buf[0] == 0xFE)
      {
        //Rückwärtsgang AKTIV
      }
      break;
    }
    //Batteriespannung und Status
    case 0x3B4:
    {
      //(((Byte[1]-240 )*256)+Byte[0])/68
      float batteryVoltage = (((buf[1] - 240) * 256) + buf[0]) / 68;

      if (buf[3] == 0x00)
      {
        Serial.print("Engine RUNNING");
      }
      if (buf[3] == 0x09)
      {
        Serial.print("Engine OFF");
      }
      break;
    }
    //Uhrzeit
    case 0x2F8:
    {
      //Merken, wann das letzte mal diese Nachricht empfangen wurde.
      previousCanDateTime = millis();

      //0: Stunden
      hours = buf[0];
      //1: Minuten
      minutes = buf[1];
      //2: Sekunden
      seconds = buf[2];
      //3: Tage
      days = buf[3];
      //4: die ersten 4 bits stellen den Monat dar.
      month = buf[4] >> 4;
      //6 & 5: Jahr
      // #6 nach links shiften und 5 addieren
      year = (buf[6] << 8) + buf[5];

      buildtimeStamp();

      break;
    }
    default:
      break;
    }
  }

  //Mausbewegungen zum Scrollen umsetzen.
  scrollScreen();
}

void checkPins()
{
  //Status des Zündungspins abrufen
  ignitionOn = digitalRead(PIN_IGNITION_INPUT);
  //Staus Odroid Vcc pin
  odroidRunning = !digitalRead(PIN_ODROID_POWER_INPUT);

  //Status Debug-Pin
  debugMode = !digitalRead(PIN_DEBUG);

  //Prüfe alle Faktoren für Start, Stopp oder Pause des Odroid.
  checkIgnitionState();
}

void checkIgnitionState()
{

  //Wenn der Status der Zündung sich verändert hat.
  if (ignitionOn != lastIgnitionState)
  {
    
  }
  //Letzten Status merken.
  lastIgnitionState = ignitionOn;
}

void startOdroid()
{
  Serial.print("[STARTODROID] Odroid Status:");
  Serial.println(odroidRunning == LOW ? "AUS" : "AN");
  //Mehrfachen Aufruf verhindern - auch wenn der PC bereits läuft
  if (odroidStartRequested || odroidRunning || pendingAction != NONE)
  {
    return;
  }

  //Starten
  odroidStartRequested = true;
  pendingAction = ODROID_START;
  digitalWrite(PIN_ODROID_POWER_BUTTON, HIGH);
  Serial.println("[startOdroid] Start angefordert.");
  previousOdroidActionTime = millis();
}

void pauseOdroid()
{
  //Wenn der PC aus ist, dann brauchen wir auch nicht Pause drücken.... Wir gehen auch einfach mal davon aus, dass er aus ist.
  //Wenn auch noch eine Stand-By Anforderung ausstehend ist, könnte ein erneutes Drücken den start wieder auslösen.
  if (odroidPauseRequested || !odroidRunning || pendingAction != NONE)
  {
    return;
  }
  pendingAction = ODROID_STANDBY;
  digitalWrite(PIN_ODROID_DISPLAY_POWER_BUTTON, HIGH);
  digitalWrite(PIN_ODROID_POWER_BUTTON, HIGH);
  Serial.println("[pauseOdroid] Stand-By angefordert");
  //Kurze Verzögerung - kurzer Tastendruck für display und Odroid
  delay(ODROID_STANDBY_HOLD_DELAY);
  digitalWrite(PIN_ODROID_DISPLAY_POWER_BUTTON, LOW);
  digitalWrite(PIN_ODROID_POWER_BUTTON, LOW);
  odroidPauseRequested = true;
  previousOdroidPauseTime = millis();
}

void stopOdroid()
{
  Serial.print("[STOPODROID] Odroid Status:");
  Serial.println(odroidRunning == LOW ? "AUS" : "AN");
  //Mehrfachen Aufruf verhindern - auch wenn der PC bereits aus ist. Das würde diesen nämlich einschalten.
  if (odroidShutdownRequested || !odroidRunning || pendingAction != NONE)
  {
    return;
  }

  //Herunterfahren
  odroidShutdownRequested = true;
  pendingAction = ODROID_STOP;

  digitalWrite(PIN_ODROID_POWER_BUTTON, HIGH);
  Serial.println("[stopOdroid] Herunterfahren angefordert");

  previousOdroidActionTime = millis();
}

void buildtimeStamp()
{
  sprintf(timeStamp, "%02d:%02d:%02d %02d.%02d.%4d", hours, minutes, seconds, days, month, year);
  sprintf(timeString, "%02d:%02d:%02d", hours, minutes, seconds);
  sprintf(dateString, "%02d.%02d.%4d", days, month, year);
}

void printCanMsg(int canId, unsigned char *buffer, int len)
{
  //OUTPUT:
  //ABC FF  FF  FF  FF  FF  FF  FF  FF
  Serial.print(canId, HEX);
  Serial.println('\t');
  for (int i = 0; i < len; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print("\t");
  }
  Serial.println();
}

void printCanMsgCsv(int canId, unsigned char *buffer, int len)
{
  //OUTPUT:
  //ABC FF  FF  FF  FF  FF  FF  FF  FF
  Serial.print(canId, HEX);
  Serial.println(';');
  for (int i = 0; i < len; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print(";");
  }
  Serial.println();
}

void scrollScreen()
{
  if (iDriveRotDir == ROTATION_RIGHT)
  {
    Mouse.move(0, 0, 1);
  }
  if (iDriveRotDir == ROTATION_LEFT)
  {
    Mouse.move(0, 0, -1);
  }
  if (iDriveRotDir == UNCHANGED)
  {
    Mouse.move(0, 0, 0);
  }
}

void timeKeeper()
{
  unsigned long currentMillis = millis();
  //Nur, wenn die letzte Aktualisierung über CAN mehr als eine Sekunde zurückliegt
  if (currentMillis - previousCanDateTime > 1000)
  {
    if (hours == 23 && minutes == 59 && seconds == 59)
    {
      hours = 0;
      minutes = 0;
      seconds = 0;
    }
    if (minutes == 59 && seconds == 59)
    {
      hours++;
      minutes = 0;
      seconds = 0;
    }
    if (seconds == 59)
    {
      minutes++;
      seconds = 0;
      buildtimeStamp();
      //Tick abgeschlossen
      return;
    }
    buildtimeStamp();
    //Sekunden erhöhen
    seconds++;
  }
}