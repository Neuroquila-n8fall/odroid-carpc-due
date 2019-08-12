//Zeit seit dem letzten Empfangen einer Uhrzeitnachricht über CAN
unsigned long previousCanDateTime = 0;
//Timeout für CAN-Bus
const int CAN_TIMEOUT = 30000;
//Zeitstempel der zuletzt empfangenen CAN-Nachricht
unsigned long previousCanMsgTimestamp = 0;
//CAS Debounce Timeout
const int CAS_DEBOUNCE_TIMEOUT = 1000;
//Debounce für MFL Knopfdruck
unsigned long lastMflPress = 0; 
//Ob der MFL Knopf "Next" gehalten wird
bool MflButtonNextHold = false; 
//Ob der MFL Knopf "Prev" gehalten wird
bool MflButtonPrevHold = false; 

//    iDrive

//Timer für Keepalive
unsigned long previousIdrivePollTimestamp = 0;
//Keepalive Intervall
const int IDRIVE_KEEPALIVE_INTERVAL = 500;
//Init Timeout
const int IDRIVE_INIT_TIMEOUT = 750;
//Init Timer
unsigned long previousIdriveInitTimestamp = 0;
//Initialisierung erfolgreich
bool iDriveInitSuccess = false;

//CAN Nachrichten
unsigned long previousCasMessageTimestamp = 0;
bool canbusEnabled = false;
//Initialisierung des iDrive Controllers, damit dieser die Drehung übermittelt
//Quelladresse für Init
const int IDRIVE_CTRL_INIT_ADDR = 0x273;
//Nachrichtenadresse erfolgreiche Initialisierung
const int IDRIVE_CTRL_INIT_RESPONSE_ADDR = 0x277;
//Nachricht für Init
unsigned char IDRIVE_CTRL_INIT[8] = {0x1D, 0xE1, 0x0, 0xF0, 0xFF, 0x7F, 0xDE, 0x4};
//Keep-Alive Nachricht, damit der Controller aufgeweckt bleibt
//Quelladresse für Keepalive
const int IDRIVE_CTRL_KEEPALIVE_ADDR = 0x501;
//Nachricht für Keepalive
unsigned char IDRIVE_CTRL_KEEPALIVE[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//Adresse des iDrive Controllers für Buttons
const int IDRIVE_CTRL_BTN_ADDR = 0x267;
//Adresse des iDrive Controllers für Rotation
const int IDRIVE_CTRL_ROT_ADDR = 0x264;
//Adresse für Status des iDrive Controllers. byte[5] = Counter, byte[6] Initialisiert: 1 = true, 6 = false
const int IDRIVE_CTRL_STATUS_ADDR = 0x5E7;
//TRUE, wenn Initialwert für Drehknopf am iDrive bereits errechnet wurde
bool RotaryInitPositionSet = false;
//Zähler für Drehung des Drehknopfes
unsigned int rotaryposition = 0;

//Adresse für Armaturenbeleuchtung
//Nachricht: Länge 2
//Byte 0: Intensität
//Byte 1: 0x0
//Mögliche Werte:
//Dimmwert    byte[0]
//0           0
//1           28
//2           56
//3           84
//4           112
//5           141
//6           169
//7           197
//8           225
//9           253
const unsigned long DASHBOARD_LIGHTING_ADDR = 0x202;
//Nachricht für Licht einschalten
unsigned char DASHBOARD_LIGHTING_ON[2] = {0xFD, 0x00};
//Nachricht für Licht auszuschalten
unsigned char DASHBOARD_LIGHTING_OFF[2] = {0xFE, 0x00};

//iDrive Stuff
enum iDriveRotationDirection
{
  ROTATION_RIGHT,
  ROTATION_LEFT,
  ROTATION_NONE
};

//iDrive Knopf gedreht?
bool iDriveRotChanged = false;
//Zuletzt gesicherte Drehung des Knopfes um Richtung zu bestimmen
int iDriveRotLast = 0;
//Drehungs-counter
int iDriveRotCountLast = 0;
//Drehrichtung
iDriveRotationDirection iDriveRotDir = ROTATION_NONE;

//iDrive Button debounce limit
const int IDRIVE_BUTTON_DEBOUNCE = 1000;
//iDrive Button debounce timer
unsigned long previousIdriveButtonTimestamp = 0;

//        iDrive ENDE