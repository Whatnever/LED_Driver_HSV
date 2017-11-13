struct HSV{
  unsigned int h;
  unsigned int s;
  unsigned int v;
  bool flag;
};
union bytefloat
{
    float f[3];
    byte b[12];
};
union byteint
{
    unsigned int i[3];
    byte b[6];
};

//Const
  // ++ PINS ++
  const int Blue_RX = 10;
  const int Blue_TX = 11;
  const int CycleButton = 8;
  //Number of LED Strips
  const uint8_t numStrips = 5;

//Variables
  //RGB Array
  HSV colorState[numStrips];
  //Timingcycles
  float blinkcycle[numStrips];
  float fadecycle[numStrips];
  unsigned long strobocycle[numStrips];
  //Timekeeping
  unsigned long previousMillis = 0;
  unsigned long currentMillis = 0;
  unsigned long deltaT = 0;
  //Bluetooth read variables
  boolean readFlag = false;
  char commandChar = '#';
  char c = '#';
  bool pause = false;
  bool off = false;

//Librarys
  //Bluetooth Software Serial Port
  SoftwareSerial BluetoothSerial(Blue_RX, Blue_TX); // RX, TX
  //PWM Multiplexer
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
  //extEEPROM Library
  extEEPROM myEEPROM(kbits_256, 1, 64);

//Function Prototypes - (I know these are not necessary for arduino compiler)
  //State Functions Prototypes
  void fade(uint8_t Strip, unsigned long deltaT);
  void blinken(uint8_t Strip, unsigned long deltaT);
  void stay(uint8_t Strip, unsigned long deltaT);
  void strobo(uint8_t Strip, unsigned long deltaT);
  //Miscellaneous functions
  void processBluetooth();
  void processKeys();
  void drawRGBStrip(uint8_t Strip);
  HSV hsv2rgb(HSV hsv);
  //State Function Pointer
  void (*statefunc[numStrips])(uint8_t Strip, unsigned long deltaT);
  //EEPROM functions
  void saveToEEPROM(unsigned int pos);
  void loadFromEEPROM(unsigned int pos);
  //Bluetooth command functions
  void commandA();
  void commandB();
  void commandC();
  void commandD();
  void commandE();
  void commandF();
  void commandG();
  void commandH();
  void commandI();
  void commandJ();
  void commandK();
  void commandL();
  void commandM();
  void commandZ();




