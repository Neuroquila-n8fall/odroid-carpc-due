#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Keyboard.h>
#include <Mouse.h>
/*
  Projektbezogene Header
*/
#include <k-can.h>

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

const int ODROID_STANDBY_HOLD_DELAY = 100;       //Button Press für Display und Sleep.
const unsigned long WAKEUP_WAIT_DELAY = 10000;   //10 Sekunden Wartezeit für Aufwecken
const unsigned long STARTUP_WAIT_DELAY = 60000;  //Wartezeit für Start
const unsigned long SHUTDOWN_WAIT_DELAY = 60000; //Wartezeit für Herunterfahren
unsigned long startPowerTransitionMillis = 0;    //Counter für den Aufweck- und Herunterfahrprozess
const unsigned long ODROID_STANDBY_DELAY = 5000; //Wartzeit für Sleepfunktion

//Geschwindigkeit der Seriellen Schnittstelle "Serial"
const int serialBaud = 115200;

//CAN Modul initialisieren
MCP_CAN CAN(SPI_CS_PIN);

//zuletzt errechneter Helligkeitswert für Display.
int lastBrightness = 0;

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

//Initialstatus der eingebauten LED
int ledState = LOW;

//Zeitstempel für Sekundentimer
unsigned long previousOneSecondTick = 0;

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

//Hier wird gespeichert, ob nach dem Ausführen einer Aktion eine weitere folgt
//Beispiel: Das Auto wird aufgesperrt und innerhalb des Startup-Intervalls wieder zugesperrt. Der PC würde nun eingeschaltet bleiben.
//Hier würde nun gespeichert werden, dass der PC wieder heruntergefahren werden soll, sobald der Timer abgelaufen ist.
PendingAction queuedAction = NONE;

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
//Taste drücken und sofort wieder freigeben
void sendKey(uint8_t keycode);
//Interaktion mit serieller Konsole
void readConsole();

void setup()
{
  digitalWrite(LED_BUILTIN, LOW);
  Wire.begin(WIRE_ADDRESS); //I2C-Init
  Serial.begin(serialBaud);
  //Tastatur- und Mausemulation aktivieren
  Keyboard.begin();
  Mouse.begin();

  //Zeitstempel einrichten
  buildtimeStamp();

  pinMode(PIN_IGNITION_INPUT, INPUT_PULLUP);        //Zündungs-Pin
  pinMode(PIN_ODROID_POWER_BUTTON, OUTPUT);         //Opto 2 - Odroid Power Button
  pinMode(PIN_ODROID_POWER_INPUT, INPUT_PULLUP);    //Odroid VOUT Pin als Rückmeldung ob der PC eingeschaltet ist
  pinMode(PIN_ODROID_DISPLAY_POWER_BUTTON, OUTPUT); //Opto 3 - Display Power Button
  pinMode(LED_BUILTIN, OUTPUT);                     //LED
  pinMode(PIN_DEBUG, INPUT_PULLUP);                 //Debug Switch Pin
  pinMode(PIN_VU7A_BRIGHTNESS, OUTPUT);             //Display Helligkeitssteuerung

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
        //Wurde der Start vorgemerkt, ausführen und zurücksetzen
        if (queuedAction == ODROID_START)
        {
          startOdroid();
          queuedAction = NONE;
        }
      }
      break;
    case ODROID_START:
      if (currentMillis - startPowerTransitionMillis >= STARTUP_WAIT_DELAY)
      {
        Serial.println("[LOOP] Start Wartezeit abgelaufen");
        pendingAction = NONE;
        //Wurde Stopp vorgemerkt, ausführen und zurücksetzen
        if (queuedAction == ODROID_STOP)
        {
          stopOdroid();
          queuedAction = NONE;
        }
        
        //Maus in die Mitte des Bildschirms bringen
        Mouse.move(960,540,0);
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

      //Sicherheitshalber zurücksetzen
      queuedAction = NONE;
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
    previousCanMsgTimestamp = currentMillis;
    canbusEnabled = true;
    CAN.readMsgBuf(&len, buf);

    unsigned int canId = CAN.getCanId();

    //Alle CAN Nachrichten ausgeben, wenn debug aktiv.
    if (debugMode)
    {
      printCanMsgCsv(canId, buf, len);
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
          Serial.println("[checkCan] Music NEXT");
          Keyboard.press(MUSIC_NEXT_KEY);
          delay(200);
          Keyboard.releaseAll();
        }
        else
        {
          //Knopf wird gehalten
          Keyboard.press(MUSIC_FASTFORWARD_KEY);
          MflButtonNextHold = true;
          Serial.println("[checkCan] Music FASTFORWARD");
        }
        lastMflPress = currentMillis;
      }
      //Prev
      if (buf[0] == 0xD0 && buf[1] == 0x0C)
      {
        //Der Knopf wurde innerhalb einer Sekunde losgelassen
        if (currentMillis - lastMflPress < 1000)
        {
          Serial.println("[checkCan] Music PREV");
          Keyboard.press(MUSIC_PREV_KEY);
          delay(200);
          Keyboard.releaseAll();
        }
        else
        {
          //Knopf wird gehalten
          Keyboard.press(MUSIC_REWIND_KEY);
          MflButtonPrevHold = true;
          Serial.println("[checkCan] Music REWIND");
        }
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
      //Wakeup Signal vom CAS --> Alle Steuergeräte aufwecken
      //Wird alle 100ms geschickt
      if (buf[0] == 0x45)
      {
      }
      //Wenn der Schlüssel im Fach ist, ist der Wert größer 0x0
      if (buf[0] > 0)
      {
        if (!iDriveInitSuccess)
        {
          CAN.sendMsgBuf(IDRIVE_CTRL_INIT_ADDR, 0, 8, IDRIVE_CTRL_INIT);
        }
      }
      break;
    }
    //CIC
    case 0x273:
    {
      Serial.print("CIC\t");
      printCanMsg(canId, buf, len);
      break;
    }
    case IDRIVE_CTRL_INIT_RESPONSE_ADDR:
    {
      Serial.println("[checkCan] iDrive Controller ist aktiv");
      iDriveInitSuccess = true;
      break;
    }
    case IDRIVE_CTRL_STATUS_ADDR:
    {
      printCanMsg(canId, buf, len);
      Serial.print("[checkCan] iDrive Controller Statusmeldung: ");
      if (buf[4] == 6)
      {
        Serial.println("Controller ist nicht initialisiert.");
        //Controller meldet er sei nicht initialisiert: Nachricht zum Initialisieren senden.
        CAN.sendMsgBuf(IDRIVE_CTRL_INIT_ADDR, 0, 8, IDRIVE_CTRL_INIT);
        iDriveInitSuccess = false;
      }
      else
      {
        Serial.println("Controller ist bereit.");
        iDriveInitSuccess = true;
      }

      break;
    }
    case IDRIVE_CTRL_KEEPALIVE_ADDR:
    {
      Serial.print("[checkCan] 0x501 (Keepalive): ");
      printCanMsg(canId, buf, len);
      break;
    }
    //CAS: Schlüssel Buttons
    case 0x23A:
      //Debounce: Befehle werden erst wieder verarbeitet, wenn der Timeout abgelaufen ist.
      if (currentMillis - previousCasMessageTimestamp > CAS_DEBOUNCE_TIMEOUT)
      {
        previousCasMessageTimestamp = currentMillis;
        //Öffnen:     00CF01FF
        if (buf[0] == 0x00 && buf[1] == 0x30 && buf[2] == 0x01 && buf[3] == 0x60)
        {
          //Prüfen, ob der PC noch im Begriff ist herunter zu fahren
          if (pendingAction == ODROID_STOP)
          {
            queuedAction = ODROID_START;
            Serial.println("[checkCan] PC wird nach dem Herunterfahren wieder gestartet.");
          }
          startOdroid();
          //Controller initialisieren.
          CAN.sendMsgBuf(IDRIVE_CTRL_INIT_ADDR, 0, 8, IDRIVE_CTRL_INIT);
          previousIdriveInitTimestamp = currentMillis;
          //Zur Kontrolle die Instrumentenbeleuchtung einschalten.
          CAN.sendMsgBuf(DASHBOARD_LIGHTING_ADDR, 0, 8, DASHBOARD_LIGHTING_ON);
        }
        //Schließen:  00DF40FF
        if (buf[0] == 0x00 && buf[1] == 0x30 && buf[2] == 0x04 && buf[3] == 0x60)
        {
          //Prüfen, ob der PC noch im Begriff ist hochzufahren
          if (pendingAction == ODROID_START)
          {
            queuedAction = ODROID_STOP;
            Serial.println("[checkCan] PC wird nach dem Starten wieder heruntergefahren.");
          }
          stopOdroid();
        }
        //Kofferraum: Wird nur gesendet bei langem Druck auf die Taste.
      }
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
      //Ab und zu wird 254 einfach so geschickt, wenn 0 vorher aktiv war...warum auch immer
      Serial.print("Beleuchtung (Roh, Ctrl):");
      int dimRawVal = buf[0];
      int dimBrightness = map(dimRawVal, 0, 253, 0, 100);
      if (buf[0] == 254)
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
      break;
    }

    //iDrive Controller

    //iDrive Controller: Drehung
    case 0x264:
    {
      //Byte 2 beinhaltet den counter
      //Byte 3 Counter Geschwindigkeit der Drehrichtung:
      //        Startet bei 0 bei Drehung im Uhrzeigersinn, wird von 0xFE heruntergezählt bei entgegengesetzter Richtung.
      //Byte 4 0x80 für Drehung im Uhrzeigersinn
      //       0x7F für Drehung gegen den Uhrzeigersinn
      //        Alle anderen Werte: Keine Drehung

      //Code von IAmOrion
      //  https://github.com/IAmOrion/BMW-iDrive-BLE-HID

      //Das Syste arbeitet nach LittleEndian. Byte 4 und 3 repräsentieren die Drehung und somit auch Drehrichtung.
      /*
      Beispiel:
      E1      FD      AA      FE      7F      1E
      E1      FD      AB      FD      7F      1E
      E1      FD      AC      FE      7F      1E
      E1      FD      AD      FF      7F      1E
      E1      FD      AE      1       80      1E
      E1      FD      AF      2       80      1E
      E1      FD      B0      3       80      1E
      */
      //Es wird also von 80FF nach 7F00 heruntergezählt und umgekehrt.

      byte rotarystepa = buf[3];
      byte rotarystepb = buf[4];
      //unsigned int newpos = (((unsigned int)rotarystepa) + ((unsigned int)rotarystepb) * 0x100);
      //Bitshift Endianness: 0xFF, 0x7F -> 7FFF
      unsigned int newpos = (rotarystepb << 8) + rotarystepa;

      //Initialstellung des Encoders feststellen
      if (!(RotaryInitPositionSet))
      {
        switch (rotarystepb)
        {
        case 0x7F:
          rotaryposition = (newpos + 1);
          break;
        case 0x80:
          rotaryposition = (newpos - 1);
          break;
        default:
          rotaryposition = newpos;
          break;
        }
        RotaryInitPositionSet = true;
      }

      //Da auch die Drehgeschwindigkeit durch byte 3 mit einbezogen wird, sollte diese auch ausgeführt werden.
      //Hier wird einfach das Delta zwischen alter und neuer Position ausgeführt.
      while (rotaryposition < newpos)
      {
        if (!iDriveInitSuccess)
        {
          iDriveRotDir = ROTATION_RIGHT;
          //Scrollbewegung ausführen
          scrollScreen();
        }
        rotaryposition++;
      }
      while (rotaryposition > newpos)
      {
        if (!iDriveInitSuccess)
        {
          iDriveRotDir = ROTATION_LEFT;
          //Scrollbewegung ausführen
          scrollScreen();
        }
        rotaryposition--;
      }

      break;
    }
      //Knöpfe und Joystick
    case 0x267:
    {
      Serial.print("[checkCan] iDrive Knöpfe:");
      printCanMsg(canId, buf, len);
      //Dieser Wert erhöht sich, wenn eine Taste gedrückt wurde.
      int buttonCounter = buf[2];

      //Status der Taste (kurz, lang, losgelassen) oder Joystick-Richtung
      int buttonPressType = buf[3];

      //Eingabetyp: Button oder Joystick
      int inputType = buf[4];

      //Knopf
      int buttonType = buf[5];

      //Entprellung der Knöpfe: Bei jedem Tastendruck wird eine Laufnummer auf byte 2 gesendet. Solange diese sich nicht verändert, wird der Knopf gehalten.
      //Zur Sicherheit wird dabei gleichzeitig die ID des Knopfes selbst abgeglichen.
      if ((buttonCounter != previousIdriveButtonPressCounter || lastKnownIdriveButtonPressType != buttonPressType) && previousIdriveButtonTimestamp - currentMillis >= 500)
      {
        //Fallunterscheidung nach Art des Knopfdrucks:
        // Kurzer Druck = 1 (Wird dauerhaft gesendet)
        // Gehalten = 2 (Wird nach ca 2 Sekunden halten gesendet)
        // Losgelassen = 0 (wird immer nach dem Loslassen gesendet)
        switch (buttonPressType)
        {
        //Kurzer Knopfdruck registriert
        case 0x01:
        {
          iDriveBtnPress = BUTTON_SHORT_PRESS;
          break;
        }
        //Lang
        case 0x02:
        {
          iDriveBtnPress = BUTTON_LONG_PRESS;
          break;
        }
        }
      }

      //Egal wie der vorherige Status war wird beim Senden von "0" die Taste als losgelassen betrachtet.
      if (buttonPressType == 0x00)
      {
        iDriveBtnPress = BUTTON_RELEASE;
      }

      //Zeitstempel des letzten Knopfdrucks merken.
      previousIdriveButtonTimestamp = currentMillis;
      //Zuletzt empfangenen Zähler merken.
      previousIdriveButtonPressCounter = buttonCounter;
      //Zuletzt empfangene Bedienungsart merken.
      lastKnownIdriveButtonPressType = buttonPressType;

      //Aussortieren, ob der Knopf in eine Richtung gedrückt wurde oder ob ein Funktionsknopf gedrückt wurde.
      if (inputType != IDRIVE_JOYSTICK)
      {
        //Knöpfe entsprechend nach Typ behandeln
        switch (buttonType)
        {
        //Joystick oder Menüknopf
        case IDRIVE_BUTTON_CENTER_MENU:
          switch (iDriveBtnPress)
          {
          //Kurz gedrückt
          case BUTTON_SHORT_PRESS:
            Keyboard.press(KEY_RETURN);
            break;
          //Lang gedrückt
          case BUTTON_LONG_PRESS:
            //Zuletzt geöffnete Apps anzeigen
            Keyboard.press(KEY_LEFT_ALT);
            Keyboard.press(KEY_TAB);
            Keyboard.release(KEY_TAB);
            break;
          //Losgelassen
          case BUTTON_RELEASE:
            Keyboard.releaseAll();
            break;
          }
          break;
          //BACK Button
        case IDRIVE_BUTTON_BACK:
          switch (iDriveBtnPress)
          {
          //Kurz gedrückt
          case BUTTON_SHORT_PRESS:
            //Zurück
            Keyboard.press(KEY_ESC);
            Keyboard.release(KEY_ESC);
            break;
          //Lang gedrückt
          case BUTTON_LONG_PRESS:
            break;
          //Losgelassen
          case BUTTON_RELEASE:
            Keyboard.releaseAll();
            break;
          }
          break;
          //OPTION Button
        case IDRIVE_BUTTON_OPTION:
          switch (iDriveBtnPress)
          {
          //Kurz gedrückt
          case BUTTON_SHORT_PRESS:
          {
            //Menü aufrufen
            Keyboard.press(KEY_LEFT_CTRL);
            Keyboard.press(KEY_ESC);
            Keyboard.release(KEY_LEFT_CTRL);
            Keyboard.release(KEY_ESC);
            break;
          }
          //Lang gedrückt
          case BUTTON_LONG_PRESS:
          {
            break;
          }
          //Losgelassen
          case BUTTON_RELEASE:
          {
            Keyboard.releaseAll();
            break;
          }
          }
          //RADIO Button
        case IDRIVE_BUTTON_RADIO:
          switch (iDriveBtnPress)
          {
          //Kurz gedrückt
          case BUTTON_SHORT_PRESS:
          {
            break;
          }
          //Lang gedrückt
          case BUTTON_LONG_PRESS:
          {
            break;
          }
          //Losgelassen
          case BUTTON_RELEASE:
          {
            Keyboard.releaseAll();
            break;
          }
          }
          //CD Button
        case IDRIVE_BUTTON_CD:
          switch (iDriveBtnPress)
          {
          //Kurz gedrückt
          case BUTTON_SHORT_PRESS:
          {
            break;
          }
          //Lang gedrückt
          case BUTTON_LONG_PRESS:
          {
            break;
          }
          //Losgelassen
          case BUTTON_RELEASE:
          {
            Keyboard.releaseAll();
            break;
          }
          }
          //NAV Button
        case IDRIVE_BUTTON_NAV:
          switch (iDriveBtnPress)
          {
          //Kurz gedrückt
          case BUTTON_SHORT_PRESS:
          {
            break;
          }
          //Lang gedrückt
          case BUTTON_LONG_PRESS:
          {
            break;
          }
          //Losgelassen
          case BUTTON_RELEASE:
          {
            Keyboard.releaseAll();
            break;
          }
          }
          //TEL Button
        case IDRIVE_BUTTON_TEL:
          switch (iDriveBtnPress)
          {
          //Kurz gedrückt
          case BUTTON_SHORT_PRESS:
          {
            break;
          }
          //Lang gedrückt
          case BUTTON_LONG_PRESS:
          {
            break;
          }
          //Losgelassen
          case BUTTON_RELEASE:
          {
            Keyboard.releaseAll();
            break;
          }
          }
        default:
          break;
        }
        previousIdriveButton = buttonType;
      }
      else
      {
        switch (buttonPressType)
        {
          //Hoch (kurz)
        case 0x11:
          Keyboard.press(KEY_UP_ARROW);
          Keyboard.release(KEY_UP_ARROW);
          break;
          //Hoch (lang)
        case 0x12:
          break;
        //Rechts (kurz)
        case 0x21:
          Keyboard.press(KEY_RIGHT_ARROW);
          Keyboard.release(KEY_RIGHT_ARROW);
          break;
        //Rechts (lang)
        case 0x22:
          break;
        //Runter (kurz)
        case 0x41:
          Keyboard.press(KEY_DOWN_ARROW);
          Keyboard.release(KEY_DOWN_ARROW);
          break;
        //Runter (lang)
        case 0x42:
          break;
        //Links (kurz)
        case 0x81:
          Keyboard.press(KEY_LEFT_ARROW);
          Keyboard.release(KEY_LEFT_ARROW);
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
      /*       //Byte 0~3 = Hinten
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
 */
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

  //Timeout für Canbus.
  if (currentMillis - previousCanMsgTimestamp >= CAN_TIMEOUT && canbusEnabled == true)
  {
    //Canbus wurde heruntergefahren. Es werden keinerlei Nachrichten mehr seit 30 Sekunden ausgetauscht.
    //Der iDrive Controller ist jetzt als deaktiviert zu betrachten und muss neu intialisiert werden
    iDriveInitSuccess = false;
    canbusEnabled = false;
    Serial.println("[checkCan] Keine Nachrichten seit 30 Sekunden. Der Bus wird nun als deaktiviert betrachtet.");
  }
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
  Serial.print('[');
  Serial.print(canId, HEX);
  Serial.print(']');
  Serial.print('\t');
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
  Serial.print(';');
  Serial.print(len);
  Serial.print(';');
  for (int i = 0; i < len; i++)
  {
    Serial.print(buffer[i], HEX);
    //Semikolon nicht beim letzten Eintrag anhängen
    if (i < len - 1)
    {
      Serial.print(';');
    }
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
  if (iDriveRotDir == ROTATION_NONE)
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

void sendKey(uint8_t keycode)
{
  Keyboard.press(keycode);
  Keyboard.release(keycode);
}

void readConsole()
{
  if (Serial.available)
  {
    String command = Serial.readStringUntil('\n');
    //Kommando zum stoppen der Tastatur und Mausdienste. Wichtig zum umprogrammieren des Controllers
    //Sind Tastatur oder Maus aktiv, kann der Due nicht neu beschrieben werden...
    if (command == "hid.stop")
    {
      Keyboard.releaseAll();
      Keyboard.end();
      Mouse.end();
    }
    if (command == "hid.start")
    {
      Keyboard.begin();
      Mouse.begin();
    }
  }
}