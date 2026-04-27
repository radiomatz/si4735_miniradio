#include "SI4735.h"
#include <EEPROM.h>

#include <Tiny4kOLED.h>

/*
 * Modified from PU2CLR's SI473x Library example for Arduino
 * this Variant is from Matthias, DM2HR
 * my Circuit can be found at https:///github.com/radiomatz/si4735_miniradio
 * Dependencies:
 *  - (modified) SI4735 Library of PU2CLR (included here)
 * - Tiny4kOled
 * - Tiny4koled font 6x8
 */

#define SH1106       // or #undef SH1106, when using a SSD1306 Display
#undef ROTARY_ENABLE // if you want to youse a Rotary like in the original Example
#undef SERIAL_ENABLE // if you want to try out some things on serial line

#ifdef ROTARY_ENABLE
#include "Rotary.h"
#endif

#include "patch_ssb_compressed.h"  // Compressed SSB patch version (saving almost 1KB)

const uint16_t size_content = sizeof ssb_patch_content;  // See ssb_patch_content.h
const uint16_t cmd_0x15_size = sizeof cmd_0x15;          // Array of lines where the 0x15 command occurs in the patch content.

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

// OLED Diaplay constants
#define RST_PIN -1  // Define proper RST_PIN if required.

#define RESET_PIN 12  // of si4735
#define GPIOINTPIN 7  // of si4735
#ifdef ROTARY_ENABLE
// Enconder PINs - if the clockwise and counterclockwise directions are not correct for you, please, invert this settings.
#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 2
#endif

// CMD Buttons
#define BUTTON A0            // cmd switch button
#define BUTTONONESTEPUP 4    // instead of rotary: encoderCount = +1
#define BUTTONONESTEPDOWN 8  // instead of rotary: encoderCount = -1

#define MIN_ELAPSED_TIME 500
#define MIN_ELAPSED_RSSI_TIME 1500

#define DEFAULT_VOLUME 25  // change it for your favorite sound volume

#define FM 0
#define LSB 1
#define USB 2
#define AM 3
#define LW 4
const char *bandModeDesc[] = { "FM ", "LSB", "USB", "AM ", "LW " };


// EEPROM Constants
#define STORE_TIME 10000  // Time of inactivity to make the current receiver status writable (10s / 10000 milliseconds).
#define IDARR 44          // ID of saved Arr in EEPROM
// EEPROM Locations
#define IDNRLOC 0     // Idnr for this Prog
#define DEFECTCELL 1  // sadly, here
#define IDXLOC 2      // BandIdx
#define CMDADR 3      // "cursor" of cmdlist
#define VOLADR 4      // Volume
// some room for more ...
#define SAVEADR 10  // location of BandArray in EEPROM



bool locksave = false;
uint8_t i = 0;

long storeTime = millis();

uint8_t currentMode = FM;
uint8_t seekDirection = 1;

uint8_t amSoftMuteThr = 0;

bool bfoOn = false;
bool avc_en = true;

bool ssbLoaded = false;
bool fmStereo = false;

bool cmdVolume = false;  // if true, the encoder will control the volume.
bool cmdAgcAtt = false;  // if true, the encoder will control the AGC / Attenuation
bool cmdStep = false;    // if true, the encoder will control the step frequency
bool cmdBw = false;      // if true, the encoder will control the bandwidth
bool cmdBand = false;    // if true, the encoder will control the band

long countRSSI = 0;

int currentBFO = 0;

long elapsedRSSI = millis();
long elapsedButton = millis();

// Encoder control variables
volatile int encoderCount = 0;
volatile bool bModeButton = false;

// Some variables to check the SI4735 status
uint16_t currentFrequency;
uint16_t newfreq;
uint16_t previousFrequency;
// uint8_t currentStep = 1;
uint8_t currentBFOStep = 25;

bool updateOled = true;

// Datatype to deal with bandwidth on AM, SSB and FM in numerical order.
// Ordering by bandwidth values.
typedef struct {
  uint8_t idx;       // SI473X device bandwidth index value
  const char *desc;  // bandwidth description
} Bandwidth;

int8_t bwIdxSSB = 4;
Bandwidth bandwidthSSB[] = {
  { 4, "0.5" },  // 0
  { 5, "1.0" },  // 1
  { 0, "1.2" },  // 2
  { 1, "2.2" },  // 3
  { 2, "3.0" },  // 4  - default
  { 3, "4.0" }   // 5
};

int8_t bwIdxAM = 5;
const int maxFilterAM = 6;
Bandwidth bandwidthAM[] = {
  { 4, "1.0" },  // 0
  { 5, "1.8" },  // 1
  { 3, "2.0" },  // 2
  { 6, "2.5" },  // 3
  { 2, "3.0" },  // 4 - default
  { 1, "4.0" },  // 5
  { 0, "6.0" }   // 6
};

int8_t bwIdxFM = 0;
Bandwidth bandwidthFM[] = {
  { 0, "AUT" },  // Automatic - default
  { 1, "110" },  // Force wide (110 kHz) channel filter.
  { 2, " 84" },
  { 3, " 60" },
  { 4, " 40" }
};

// Atenuação and AGC
int8_t agcIdx = 0;
uint8_t disableAgc = 0;
uint8_t agcNdx = 0;

int tabStep[] = {
  1,    // 0
  5,    // 1
  9,    // 2
  10,   // 3
  50,   // 4
  100,  // 5
  500
};

const int lastStep = (sizeof tabStep / sizeof(int)) - 1;
int idxStep = 1;


typedef struct {
  uint8_t bandType;        // Band type (FM, MW or SW)
  uint16_t minimumFreq;    // Minimum frequency of the band
  uint16_t maximumFreq;    // maximum frequency of the band
  uint16_t currentFreq;    // Default frequency or current frequency
  uint8_t currentStepIdx;  // Idex of tabStep:  Defeult frequency step (See tabStep)
  uint8_t bandwidthIdx;    //  Index of the table bandwidthFM, bandwidthAM or bandwidthSSB;
  char banddesc[4];
  int16_t bfo;
  uint8_t mode;
} Band;

Band band[] = {
  { FM_BAND_TYPE, 6400, 10800, 9850, 3, 5, "FM", 0, FM },    // Bayern 3
  { FM_BAND_TYPE, 6400, 10800, 9070, 3, 5, "B1", 0, FM },    // Bayern 1
  { FM_BAND_TYPE, 6400, 10800, 10000, 3, 5, "DLF", 0, FM },  // Deutschlandfunk
  { LW_BAND_TYPE, 149, 520, 225, 2, 5, "LW", 0, AM },
  { MW_BAND_TYPE, 520, 1810, 530, 2, 5, "MW", 0, AM },
  { SW_BAND_TYPE, 1810, 30000, 9670, 0, 5, "DAR", 0, AM },  // Radio DARC, Sun, 11:00 Localtime Germany
  { SW_BAND_TYPE, 1810, 30000, 6180, 0, 5, "DWD", 0, AM },  // DWD Voice Pinneberg
  { SW_BAND_TYPE, 1810, 30000, 1840, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 3573, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 5364, 0, 2, "WSP", -401, USB },
  { SW_BAND_TYPE, 1810, 30000, 7074, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 10136, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 14074, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 18100, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 21074, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 24915, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 28074, 0, 5, "FT8", 0, USB },
  { SW_BAND_TYPE, 1810, 30000, 3855, 0, 3, "DWD", 1050, USB },   // DDH3 DWD WeFax IOC576
  { SW_BAND_TYPE, 1810, 30000, 7880, 0, 3, "DWD", 1225, USB },   // DDK3 DWD WeFax IOC576
  { SW_BAND_TYPE, 1810, 30000, 4583, 0, 1, "DWD", 1280, USB },   // DDK2 DWD Rtty 50Bd 450Hz
  { SW_BAND_TYPE, 1810, 30000, 7646, 0, 1, "DWD", 1427, USB },   // DDH7 DWD Rtty 50Bd 450Hz
  { SW_BAND_TYPE, 1810, 30000, 10100, 0, 1, "DWD", 835, USB },   // DDK9 DWD Rtty 50Bd 450Hz
  { SW_BAND_TYPE, 1810, 30000, 149, 0, 0, "DWD", 2225, USB },    // DDH47 DWD Rtty 50Bd _85Hz_(147.3kHz)
  { SW_BAND_TYPE, 1810, 30000, 11039, 0, 1, "DWD", 1440, USB },  // DDH9 DWD Rtty 50Bd 450Hz
  { SW_BAND_TYPE, 1810, 30000, 14230, 0, 3, "TV1", 600, USB },   // SSTV
  { SW_BAND_TYPE, 1810, 30000, 21340, 0, 3, "TV2", 600, USB },   // SSTV
  { SW_BAND_TYPE, 1810, 30000, 28680, 0, 3, "TV3", 600, USB },   // SSTV
  { SW_BAND_TYPE, 1810, 30000, 27245, 1, 5, "CB", 0, USB },      // SSTV
  { SW_BAND_TYPE, 1810, 30000, 1885, 0, 5, "Swa", 0, LSB },      // Schwaben Rundspruch DARC Dok T05
  { SW_BAND_TYPE, 1810, 30000, 3590, 0, 5, "Obb", -750, USB },   // Oberbayern Rundspruch DARC Dok C digital MFSK-32 Center 750 Hz
  { SW_BAND_TYPE, 1810, 30000, 15350, 1, 5, "Tye", 0, AM },      // Stimme Türkiye
  { SW_BAND_TYPE, 1810, 30000, 11880, 1, 5, "RO", 0, AM },       // Radio Romania
  { SW_BAND_TYPE, 1810, 30000, 9535, 6, 5, "CN1", 0, AM },       // Chinese
  { SW_BAND_TYPE, 1810, 30000, 11735, 6, 5, "CN2", 0, AM }       // Chinese
};

const uint8_t lastBand = (sizeof band / sizeof(Band)) - 1;
uint8_t bandIdx = 1;

uint8_t rssi = 0;
uint8_t stereo = 1;
uint8_t volume = DEFAULT_VOLUME;

// Devices class declarations
#ifdef ROTARY_ENABLE
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);
#endif

SI4735 si4735;

bool key_reset = false;
char key = 0;
char *stationName;
char bufferStatioName[20];
long rdsElapsed = millis();
char oldBuffer[15];


#define CMDTABSIZE 12
enum {
  CMDSTEP,
  CMDFREQ,
  CMDVOL,
  CMDMODE,
  CMDBAND,
  CMDBFOSTEP,
  CMDBFO,
  CMDAGC,
  CMDVC,
  CMDBW,
  CMDSEEK,
  CMDLOCK
} CMDTAB;
const char *CMDTABSTR[CMDTABSIZE] = { "STEP", "FREQ", "VOL", "MODE", "BAND", "BSTEP", "BFO", "AGC", "VC", "BW", "SEEK", "LOCK" };
uint8_t cmd = 0;

// *****************************************************************

void setup() {

#ifdef SERIAL_ENABLE
  Serial.begin(115200);
  delay(500);
  if (Serial.available() && Serial.read() == 'r')
    key_reset = true;
#endif  // SERIAL_ENABLE

    // Encoder pins
#ifdef ROTARY_ENABLE
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
#endif

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUTTONONESTEPUP, INPUT_PULLUP);
  pinMode(BUTTONONESTEPDOWN, INPUT_PULLUP);

  // Splash - Change it for your introduction text.
#ifdef SERIAL_ENABLE
  Serial.println(F("SI4735-D60 by DM2HR"));
  Serial.println(F("All in One Radio"));
  Serial.println(F("Arduino Library -"));
  Serial.println(F("V3.0.7 - By PU2CLR"));

  Serial.println(F("Init OLED now..."));
#endif

#ifdef SH1106
  oled.begin(2, 0, 128, 64, sizeof(tiny4koled_init_128x32r), tiny4koled_init_128x32r);
#else
  oled.begin();
#endif
  oled.clear();
  oled.on();
  oled.setFont(FONT6X8);
  oled.setContrast(255);

  oled.println(F("SI4735-D60 by DM2HR"));
  oled.println(F("All in One Radio"));
  oled.println(F("Arduino Library -"));
  oled.print(F("V3.0.7 - By PU2CLR"));

  delay(500);

  // If you want to reset the eeprom, keep the VOLUME_UP button pressed during statup
  if (digitalRead(BUTTON) == LOW || key_reset) {

    oled.clear();
    oled.print(F("EEPROM RESET"));
    for (int j = 0; j < EEPROM.length(); j++) {
      oled.setCursor(0, 1);
      EEPROM.write(j, 0xff);
      oled.print(j);
    }
    // delay(2000);
    oled.clear();
  }

  // Encoder interrupt
#ifdef ROTARY_ENABLE
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);
#endif  // ROTARY_ENABLE

  // Gets and sets the Si47XX I2C bus address
  int16_t si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);
  if (si4735Addr == 0) {
#ifdef SERIAL_ENABLE
    Serial.println(F("Si473X not found!"));
    Serial.flush();
#else
    oled.clear();
    oled.print(F("Si473X not found!!"));
#endif
    while (1)
      ;
  } else {
#ifdef SERIAL_ENABLE
    Serial.print(F("The Si473X I2C address is 0x"));
    Serial.println(si4735Addr, HEX);
#endif
  }

  si4735.setup(RESET_PIN, MW_BAND_TYPE);  //
  delay(300);

  // Checking the EEPROM content
  if (EEPROM.read(IDNRLOC) == IDARR) {
    readAllReceiverInformation();
  }

  // Set up the radio for the current band (see index table variable bandIdx )
  currentMode = band[bandIdx].mode;
  check_loadpatch();
  useBand();

  currentFrequency = previousFrequency = si4735.getFrequency();
  si4735.setAvcAmMaxGain(90);  // Sets the maximum gain for automatic volume control on AM/SSB mode (from 12 to 90dB)
  delay(50);

#ifdef SERIAL_ENABLE
  Serial.print(F("set Volume to: "));
  Serial.println(volume);
#endif
  si4735.setVolume(volume);
  delay(50);
}

// *****************************************************************

void check_loadpatch() {
  if (currentMode == USB || currentMode == LSB) {
    if (!ssbLoaded)
      loadSSB();
    currentBFO = band[bandIdx].bfo;
    si4735.setSSBBfo(currentBFO);
  } else {
    if (currentMode != FM)
      currentMode = AM;
    ssbLoaded = false;
    bfoOn = false;
    useBand();
  }
}

// *****************************************************************

// ISR's
#ifdef ROTARY_ENABLE
void rotaryEncoder() {
  static uint8_t encoderStatus = encoder.process();
  if (encoderStatus) {
    if (encoderStatus == DIR_CW) {
      encoderCount = 1;
    } else {
      encoderCount = -1;
    }
  }
}
#endif

// *****************************************************************

/* EEPROM Routines */
void saveAllReceiverInformation() {
  writeEpromState();
#ifdef SERIAL_ENABLE
  Serial.println(F("State saved in EEPROM"));
#endif
}

void writeEpromState() {
  if (locksave)
    return;
  locksave = true;

#ifdef SERIAL_ENABLE
  Serial.println(F("write EEPROM"));
#endif
  uint8_t *ptr = (uint8_t *)&band;
  EEPROM.update(IDNRLOC, IDARR);
  EEPROM.update(IDXLOC, bandIdx);
  EEPROM.update(VOLADR, volume);
  EEPROM.update(CMDADR, (uint8_t)cmd);
  for (int i = 0; i < sizeof(band); i++) {
    EEPROM.update(SAVEADR + i, *(ptr + i));
  }
  resetEepromDelay();
  locksave = false;
}

void readAllReceiverInformation() {
  readEpromState();
#ifdef SERIAL_ENABLE
  Serial.println(F("EEPROM State restored"));
#endif
}

void readEpromState() {
#ifdef SERIAL_ENABLE
  Serial.println(F("read EEPROM"));
#endif
  uint8_t *ptr = (uint8_t *)&band;
  for (int i = 0; i < sizeof(band); i++) {
    *(ptr + i) = EEPROM.read(SAVEADR + i);
  }
  EEPROM.get(IDXLOC, bandIdx);
  EEPROM.get(VOLADR, volume);
#ifdef SERIAL_ENABLE
  Serial.print(F("EEPROM volume:"));
  Serial.println(volume);
#endif
  EEPROM.get(CMDADR, cmd);
}

void resetEepromDelay() {
  storeTime = millis();
  previousFrequency = 0;
}

// *****************************************************************

/* utility routines */
void convertToChar(uint16_t value, char *strValue, uint8_t len, uint8_t dot, uint8_t separator) {
  char d;
  for (int i = (len - 1); i >= 0; i--) {
    d = value % 10;
    value = value / 10;
    strValue[i] = d + 48;
  }
  strValue[len] = '\0';
  if (dot > 0) {
    for (int i = len; i >= dot; i--) {
      strValue[i + 1] = strValue[i];
    }
    strValue[dot] = separator;
  }

  if (strValue[0] == '0') {
    strValue[0] = ' ';
    if (strValue[1] == '0')
      strValue[1] = ' ';
  }
}

// *****************************************************************

void showFrequency() {
  char *unit;
  char freqDisplay[10];

  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    convertToChar(currentFrequency, freqDisplay, 5, 3, ',');
    unit = (char *)"MHz";
  } else {
    unit = (char *)"kHz";
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE)
      convertToChar(currentFrequency, freqDisplay, 5, 0, '.');
    else
      convertToChar(currentFrequency, freqDisplay, 5, 2, ',');
  }
#ifdef SERIAL_ENABLE
  Serial.print(F("Freq:"));
  Serial.print(freqDisplay);
  Serial.println(unit);
#endif
}

// *****************************************************************

bool checkStopSeeking() {
  // Checks the touch and encoder
  return (encoderCount != 0 || digitalRead(BUTTON) == LOW);  // returns true if the user rotates the encoder or press the push button
}

// *****************************************************************

void showStatus() {
#ifdef SERIAL_ENABLE
  Serial.println();
  showFrequency();
  showBandDesc();
  showStep();
  showBandwidth();
  showAgcAtt();
  showRSSI();
  showVolume();
  if (currentMode == FM) {
    Serial.print(F("RDS Station:"));
    Serial.println(stationName);
  }
  Serial.println();
#endif
}

// *****************************************************************

void showFrequencySeek(uint16_t freq) {
  currentFrequency = freq;
  showOledFrequency();
}

// *****************************************************************

void showOledFrequency() {
  uint32_t f = currentFrequency;

  oled.setCursor(0, 0);

  oled.print(F("F:"));
  if (currentMode == FM) {
    f *= 10;
  }
  if (f > 1000) {
    oled.print(f / 1000);
    oled.print('.');
  }
  if (f % 1000 < 10)
    oled.print("00");
  else if (f % 1000 < 100)
    oled.print('0');
  oled.print(f % 1000);
  oled.print('/');
  oled.print(tabStep[idxStep]);
}

// *****************************************************************

void showOledStatus() {

  updateOled = false;

  oled.clear();

  showOledFrequency();

  oled.print(F(" "));
  oled.print(band[bandIdx].banddesc);

  oled.print(F(" "));
  oled.print(bandModeDesc[currentMode]);

  if (ssbLoaded) {
    oled.print(F(" BFO:"));
    oled.print(currentBFO);
    oled.print('/');
    oled.print(currentBFOStep);
  }


  if (ssbLoaded)
    oled.print(F(" RSSI:"));
  else
    oled.print(F(" R/SN:"));
  oled.print(rssi);

  if (!ssbLoaded) {
    oled.print('/');
    oled.print(si4735.getCurrentSNR());
  }

  oled.print(F(" V:"));
  oled.print(si4735.getCurrentVolume());

  oled.print(F(" Vc:"));
  oled.print(avc_en ? 1 : 0);

  char *bw = "";
  oled.print(F(" BW:"));
  switch (currentMode) {
    case LSB:
    case USB:
      bw = (char *)bandwidthSSB[bwIdxSSB].desc;
      break;

    case AM:
      bw = bw = (char *)bandwidthAM[bwIdxAM].desc;
      break;

    default:
      bw = (char *)bandwidthFM[bwIdxFM].desc;
      break;
  }
  oled.print(bw);

  oled.print(F(" Agc:"));
  if (disableAgc)
    oled.print(agcNdx);
  else
    oled.print(F("auto"));

  if (currentMode == FM) {
    oled.print(F(" St:"));
    oled.print(stationName);
  }
  showCmdMode();
}

// *****************************************************************

void showCmdMode() {
  oled.setCursor(10 * 6, 3);
  oled.clearToEOL();
  oled.setCursor(10 * 6, 3);
  oled.print(F("*"));
  if (locksave) {
    oled.print(F("LOCKED*"));
  } else {
    oled.print(CMDTABSTR[cmd]);
    i = cmd + 1;
    if (i > CMDTABSIZE - 1)
      i = 0;
    oled.print(F(">"));
    oled.print(CMDTABSTR[i]);
  }
  oled.flush();
}

// *****************************************************************

void showBandDesc() {
  char *bandMode;

  Serial.print(F("Band:"));
  Serial.print(band[bandIdx].banddesc);
  Serial.print(F(" "));
  Serial.println(bandModeDesc[currentMode]);
}

// *****************************************************************

void showRSSI() {
  si4735.getCurrentReceivedSignalQuality();
  rssi = si4735.getCurrentRSSI();
  //  int bars = (rssi / 20.0) + 1;
  Serial.print(F("RSSI:"));
  Serial.print(rssi);
  Serial.print('/');
  Serial.print(si4735.getCurrentSNR());

  if (currentMode == FM) {
    if (si4735.getCurrentPilot()) {
      Serial.print(F(" stereo"));
    }
  }
  Serial.println();
}

// *****************************************************************

void showVolume() {
  Serial.print(F("Volume:"));
  Serial.println(si4735.getCurrentVolume());
}

// *****************************************************************

void showStep() {
  Serial.print(F("Step:"));
  Serial.println(tabStep[idxStep]);
}

// *****************************************************************
/**
   Shows bandwidth on AM,SSB and FM mode
*/
void showBandwidth() {
  char *bw;
  if (currentMode == LSB || currentMode == USB) {
    bw = (char *)bandwidthSSB[bwIdxSSB].desc;
    showBFO();
  } else if (currentMode == AM) {
    bw = (char *)bandwidthAM[bwIdxAM].desc;
  } else {
    bw = (char *)bandwidthFM[bwIdxFM].desc;
  }
  Serial.print(F("BW:"));
  Serial.println(bw);
}

// *****************************************************************
/*
   Shows AGCC and Attenuation
*/
void showAgcAtt() {
  // Show AGC Information
  if (agcIdx == 0) {
    Serial.println(F("AGC"));
  } else {
    Serial.print(F("At"));
    Serial.println(agcNdx);
  }
}

// *****************************************************************
/*
   Shows the BFO current status.
   Must be called only on SSB mode (LSB or USB)
*/
void showBFO() {
  Serial.print(F("BFO: "));
  Serial.print(currentBFO);
  Serial.println(F("Hz"));
}

// *****************************************************************
/*
   Clean the content of the third line (line 2 - remember the first line is 0)
*/
void cleanBfoRdsInfo() {
  *stationName = 0;
}

// *****************************************************************
/*
   Show the Station Name.
*/
void showRDSStation() {
  char *po, *pc;
  int col = 0;

  po = oldBuffer;
  pc = stationName;
  while (*pc) {
    *po = *pc;
    po++;
    pc++;
    col += 10;
  }
}

// *****************************************************************
/*
   Checks the station name is available
*/
void checkRDS() {
  si4735.getRdsStatus();
  if (si4735.getRdsReceived()) {
    if (si4735.getRdsSync() && si4735.getRdsSyncFound() && !si4735.getRdsSyncLost() && !si4735.getGroupLost()) {
      stationName = si4735.getRdsText0A();
      if (stationName != NULL /* && si4735.getEndGroupB() */ && (millis() - rdsElapsed) > 10 ) {
        showRDSStation();
        // si4735.resetEndGroupB();
        rdsElapsed = millis();
      }
    }
  }
}

// *****************************************************************
/*
   Goes to the next band (see Band table)
*/
void bandUp() {
  // save the current frequency for the band
  if (currentFrequency != 0)
    band[bandIdx].currentFreq = currentFrequency;
  band[bandIdx].currentStepIdx = idxStep;  // currentStep;

  if (bandIdx < lastBand) {
    bandIdx++;
  } else {
    bandIdx = 0;
  }
  useBand();
}

// *****************************************************************
/*
   Goes to the previous band (see Band table)
*/
void bandDown() {
  // save the current frequency for the band
  if (currentFrequency != 0)
    band[bandIdx].currentFreq = currentFrequency;
  band[bandIdx].currentStepIdx = idxStep;
  if (bandIdx > 0) {
    bandIdx--;
  } else {
    bandIdx = lastBand;
  }
  useBand();
}

// *****************************************************************
/*
   This function loads the contents of the ssb_patch_content array into the CI (Si4735) and starts the radio on
   SSB mode.
*/
void loadSSB() {
#ifdef SERIAL_ENABLE
  Serial.println(F("  Switching to SSB  "));
#else
  oled.clear();
  oled.print(F("Switch to SSB"));
  oled.flush();
#endif
  // si4735.setI2CFastModeCustom(850000); // It is working. Faster, but I'm not sure if it is safe.
  // si4735.setI2CFastModeCustom(500000);
  si4735.queryLibraryId();  // Is it really necessary here? I will check it.
  si4735.patchPowerUp();
  delay(50);
  si4735.downloadCompressedPatch(ssb_patch_content, size_content, cmd_0x15, cmd_0x15_size);
  si4735.setSSBConfig(bandwidthSSB[bwIdxSSB].idx, 1, 0, 1, 0, 1);
  si4735.setI2CStandardMode();
  ssbLoaded = true;
  cleanBfoRdsInfo();
}

// *****************************************************************
/*
   Switch the radio to current band.
   The bandIdx variable points to the current band.
   This function change to the band referenced by bandIdx (see table band).
*/
void useBand() {
  cleanBfoRdsInfo();
  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    si4735.setTuneFrequencyAntennaCapacitor(0);
    si4735.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabStep[band[bandIdx].currentStepIdx]);
    si4735.setSeekFmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
    si4735.setSeekFmSpacing(1);
    bfoOn = ssbLoaded = false;
    si4735.setRdsConfig(1, 2, 2, 2, 2);
    si4735.setFifoCount(1);
    bwIdxFM = band[bandIdx].bandwidthIdx;
    si4735.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    si4735.setFmBlendStereoThreshold(127);
    si4735.setFmBlendMonoThreshold(127);

  } else {
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE || band[bandIdx].bandType == SW_BAND_TYPE) {
      si4735.setTuneFrequencyAntennaCapacitor(0);
    }
    if (ssbLoaded) {
      si4735.setSSB(149, 30000, band[bandIdx].currentFreq, tabStep[band[bandIdx].currentStepIdx], currentMode);
      si4735.setTuneFrequencyAntennaCapacitor(1);
      si4735.setSSBAutomaticVolumeControl(1);
      si4735.setSsbSoftMuteMaxAttenuation(0);  // Disable Soft Mute for SSB
      bwIdxSSB = band[bandIdx].bandwidthIdx;
      si4735.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
      si4735.setSSBBfo(currentBFO);
    } else {
      si4735.setAM(149, 30000, band[bandIdx].currentFreq, tabStep[band[bandIdx].currentStepIdx]);
      si4735.setAutomaticGainControl(disableAgc, agcNdx);

      si4735.setAmSoftMuteMaxAttenuation(50);  // // Enable Soft Mute for AM
      si4735.setAMSoftMuteSnrThreshold(25);

      bwIdxAM = band[bandIdx].bandwidthIdx;
      si4735.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
      bfoOn = false;
    }
    si4735.setSeekAmLimits(149, 30000);
    si4735.setSeekAmSpacing((tabStep[band[bandIdx].currentStepIdx] > 10) ? 10 : tabStep[band[bandIdx].currentStepIdx]);  // Max 10kHz for spacing
  }
  currentMode = band[bandIdx].mode;
  currentFrequency = band[bandIdx].currentFreq;
  idxStep = band[bandIdx].currentStepIdx;
  si4735.setFrequencyReliable(currentFrequency);
  delay(50);
#ifdef SERIAL_ENABLE
  Serial.print(F("set Volume to: "));
  Serial.println(volume);
#endif
  si4735.setVolume(volume);
  delay(50);
  showStatus();
  showOledStatus();
  resetEepromDelay();
}

// *****************************************************************
/**
   Changes the step frequency value based on encoder rotation
*/
void doStep(int8_t v) {
  idxStep = idxStep + v;
  if (idxStep > lastStep)
    idxStep = 0;
  else if (idxStep < 0)
    idxStep = lastStep;
  si4735.setFrequencyStep(tabStep[idxStep]);
  band[bandIdx].currentStepIdx = idxStep;
  if (currentMode == AM || currentMode == LSB || currentMode == USB)
    si4735.setSeekAmSpacing((tabStep[idxStep] > 10) ? 10 : tabStep[idxStep]);  // Max 10kHz for spacing
  else
    si4735.setSeekFmSpacing(10);
  showStep();
  //  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
}

// *****************************************************************
/**
   Changes the volume based on encoder rotation
*/
void doVolume(uint8_t v) {
  if (v == 1)
    si4735.volumeUp();
  else
    si4735.volumeDown();
  delay(10);
  volume = si4735.getCurrentVolume();
  showVolume();
  //  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
}

// *****************************************************************
/**
   Switches the AGC/Attenuation based on encoder rotation
*/
void doAgcAtt(int8_t v) {

  agcIdx = (v == 1) ? agcIdx + 1 : agcIdx - 1;
  if (agcIdx < 0)
    agcIdx = 37;
  else if (agcIdx > 37)
    agcIdx = 0;

  disableAgc = (agcIdx > 0);  // if true, disable AGC; esle, AGC is enable

  if (agcIdx > 1)
    agcNdx = agcIdx - 1;
  else
    agcNdx = 0;

  // Sets AGC on/off and gain
  si4735.setAutomaticGainControl(disableAgc, agcNdx);
  showAgcAtt();
  //  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
}

// *****************************************************************
/**
   Switches the bandwidth based on encoder rotation
*/
void doBandwidth(uint8_t v) {
  if (currentMode == LSB || currentMode == USB) {
    bwIdxSSB = (v == 1) ? bwIdxSSB + 1 : bwIdxSSB - 1;

    if (bwIdxSSB > 5)
      bwIdxSSB = 0;
    else if (bwIdxSSB < 0)
      bwIdxSSB = 5;

    band[bandIdx].bandwidthIdx = bwIdxSSB;

    si4735.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      si4735.setSSBSidebandCutoffFilter(0);
    else
      si4735.setSSBSidebandCutoffFilter(1);
  } else if (currentMode == AM) {
    bwIdxAM = (v == 1) ? bwIdxAM + 1 : bwIdxAM - 1;

    if (bwIdxAM > maxFilterAM)
      bwIdxAM = 0;
    else if (bwIdxAM < 0)
      bwIdxAM = maxFilterAM;

    band[bandIdx].bandwidthIdx = bwIdxAM;
    si4735.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
  } else {
    bwIdxFM = (v == 1) ? bwIdxFM + 1 : bwIdxFM - 1;
    if (bwIdxFM > 4)
      bwIdxFM = 0;
    else if (bwIdxFM < 0)
      bwIdxFM = 4;

    band[bandIdx].bandwidthIdx = bwIdxFM;
    si4735.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
  }
  showBandwidth();
  //  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
}

// *****************************************************************

void loop() {

#ifdef SERIAL_ENABLE

  if (Serial.available()) {
    key = Serial.read();
    switch (key) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        newfreq = (newfreq * 10) + (key - '0');
        Serial.print(F("\rNew Frequency: "));
        Serial.print(newfreq);
        break;

      case '\b':
        newfreq = newfreq / 10;
        Serial.print(F("New Frequency: "));
        Serial.println(newfreq);
        break;

      case 0x0d:
      case 0x0a:
        Serial.println();
        if (newfreq == 0)
          break;
        si4735.setFrequencyReliable(newfreq);
        currentFrequency = si4735.getFrequency();
        if (currentFrequency != 0)
          band[bandIdx].currentFreq = currentFrequency;
        newfreq = 0;
        Serial.print(F("Switch Frequency: "));
        Serial.println(currentFrequency);
        previousFrequency = 0;
        break;

      case 'v':
        encoderCount = -1;
        cmd = CMDVOL;
        break;

      case 'V':
        encoderCount = 1;
        cmd = CMDVOL;
        break;

      case 'a':
        encoderCount = -1;
        cmd = CMDAGC;
        break;

      case 'A':
        encoderCount = 1;
        cmd = CMDAGC;
        break;

      case 's':
        encoderCount = 1;
        cmd = CMDSTEP;
        break;

      case 'w':
        encoderCount = -1;
        cmd = CMDBW;
        break;

      case 'W':
        encoderCount = 1;
        cmd = CMDBW;
        break;

      case 'm':
        encoderCount = 1;
        cmd = CMDMODE;
        break;

      case 'r':
        Serial.print(F("RDS Station:"));
        Serial.println(stationName);
        break;

      case 'B':
        encoderCount = 1;
        cmd = CMDBFO;
        break;

      case 'b':
        encoderCount = -1;
        cmd = CMDBFO;
        break;

      case '<':
        encoderCount = -1;
        cmd = CMDBAND;
        break;

      case '>':
        encoderCount = 1;
        cmd = CMDBAND;
        break;

      case 'x':
        break;

      case ',':
      case '.':
        if (currentMode == FM || currentMode == AM) {
          cmd = CMDSEEK;
          if (key == '.')
            encoderCount = 1;
          else
            encoderCount = -1;
        }
        break;

      case '-':
        encoderCount = -1;
        cmd = CMDFREQ;
        break;

      case '+':
        encoderCount = 1;
        cmd = CMDFREQ;
        break;
    }
    showStatus();
    showOledStatus();
  }
#endif  // SERIAL_ENABLE

  if (!locksave) {
    // BUTTIONS
    if (digitalRead(BUTTON) == LOW) {
      while (digitalRead(BUTTON) == LOW) delay(1);
      updateOled = true;
      cmd++;
      if (cmd >= CMDTABSIZE)
        cmd = 0;
      showCmdMode();
    }
    if (digitalRead(BUTTONONESTEPUP) == LOW) {
      while (digitalRead(BUTTONONESTEPUP) == LOW) delay(1);
      encoderCount = 1;
    } else if (digitalRead(BUTTONONESTEPDOWN) == LOW) {
      while (digitalRead(BUTTONONESTEPDOWN) == LOW) delay(1);
      encoderCount = -1;
    }
  } else {
    if (digitalRead(BUTTONONESTEPUP) + digitalRead(BUTTONONESTEPDOWN) == LOW) {
      while (digitalRead(BUTTONONESTEPUP) + digitalRead(BUTTONONESTEPDOWN) == LOW)
        delay(1);
      locksave = false;
      showCmdMode();
    }
  }


  // Check if the encoder has moved.
  if (encoderCount != 0) {
    updateOled = true;

    switch (cmd) {
      case CMDFREQ:
        if (encoderCount == 1) {
          si4735.frequencyUp();
          seekDirection = 1;
        }
        if (encoderCount == -1) {
          si4735.frequencyDown();
          seekDirection = 0;
        }
        currentFrequency = si4735.getFrequency();
        if (currentFrequency != 0)
          band[bandIdx].currentFreq = currentFrequency;
        previousFrequency = 0;
        break;

      case CMDSTEP:
        doStep(encoderCount);
        break;

      case CMDVOL:
        doVolume(encoderCount);
        break;

      case CMDBAND:
        if (encoderCount == 1)
          bandUp();
        if (encoderCount == -1)
          bandDown();
        currentMode = band[bandIdx].mode;
        check_loadpatch();
        break;

      case CMDAGC:
        doAgcAtt(encoderCount);
        break;

      case CMDBFO:
        currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
        si4735.setSSBBfo(currentBFO);
        band[bandIdx].bfo = currentBFO;
        previousFrequency = 0;  // Forces eeprom update
        break;

      case CMDBFOSTEP:
        if (currentBFOStep == 1)
          currentBFOStep = 5;
        else if (currentBFOStep == 5)
          currentBFOStep = 10;
        else if (currentBFOStep == 10)
          currentBFOStep = 25;
        else if (currentBFOStep == 25)
          currentBFOStep = 50;
        else if (currentBFOStep == 50)
          currentBFOStep = 100;
        else if (currentBFOStep == 100)
          currentBFOStep = 250;
        else if (currentBFOStep == 250)
          currentBFOStep = 1;
        break;

      case CMDBW:
        doBandwidth(encoderCount);
        break;

      case CMDMODE:
        if (currentMode == AM) {
          // If you were in AM mode, it is necessary to load SSB patch (avery time)
          loadSSB();
          currentMode = LSB;
          currentBFO = band[bandIdx].bfo;
          si4735.setSSBBfo(currentBFO);
        } else if (currentMode == LSB) {
          currentMode = USB;
          currentBFO = band[bandIdx].bfo;
          si4735.setSSBBfo(currentBFO);
        } else if (currentMode == USB) {
          currentMode = AM;
          ssbLoaded = false;
          bfoOn = false;
        }
        if (currentFrequency != 0)
          band[bandIdx].currentFreq = currentFrequency;
        band[bandIdx].currentStepIdx = idxStep;
        band[bandIdx].mode = currentMode;
        useBand();
        break;

      case CMDVC:
        avc_en = avc_en == 0 ? 1 : 0;
        if (ssbLoaded) {
          si4735.setSSBAutomaticVolumeControl(avc_en);
        } else {  // TODO
          // si4735.setAutomaticVolumeControl(avc_en);
        }
        break;

      case CMDSEEK:
        if (currentMode == FM || currentMode == AM) {
          if (encoderCount == 1)
            seekDirection = 1;
          else
            seekDirection = 0;
          /* old method of PU2CLR */
          // Jumps up or down one space
          if (seekDirection)
            si4735.frequencyUp();
          else
            si4735.frequencyDown();
          si4735.seekStationProgress(showFrequencySeek, checkStopSeeking, seekDirection);
          delay(30);
          if (currentMode == FM) {
            float f = round(si4735.getFrequency() / 10.0);
            currentFrequency = (uint16_t)f * 10;  // adjusts band space from 1 (10kHz) to 10 (100 kHz)
            si4735.setFrequencyReliable(currentFrequency);
          } else {
            currentFrequency = si4735.getFrequency();  //
          }
          if (currentFrequency != 0)
            band[bandIdx].currentFreq = currentFrequency;

          previousFrequency = 0;
        }
        break;

      case CMDLOCK:
        locksave = true;
        showCmdMode();
        break;
    }

    if (encoderCount != 0) {
      showStatus();
      updateOled = true;
    }

    encoderCount = 0;
    resetEepromDelay();  // if you moved the encoder, something was changed
    elapsedRSSI = millis();
    countRSSI = 0;
    elapsedButton = millis();
  }

  // Show RSSI status only if this condition has changed
  if ( (millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 9 ) {
    si4735.getCurrentReceivedSignalQuality();
    int aux = si4735.getCurrentRSSI();
    if (rssi != aux) {
      rssi = aux;
      showStatus();
      updateOled = true;
    }


    if (countRSSI++ > 3) {
      //disableCommand(NULL, false, NULL); // disable all command buttons
      countRSSI = 0;
    }
    elapsedRSSI = millis();
  }

  if (currentMode == FM) {
    if (currentFrequency != previousFrequency) {
      cleanBfoRdsInfo();
    } else {
      checkRDS();
    }
  }

  if ( updateOled ) {
    updateOled = false;
    showOledStatus();
  }

  // Show the current frequency only if it has changed
  if ((currentFrequency != previousFrequency && currentFrequency != 0)
      || (millis() - storeTime) > STORE_TIME) {
    band[bandIdx].currentFreq = currentFrequency;
    saveAllReceiverInformation();
    storeTime = millis();
    previousFrequency = currentFrequency;
    // updateOled = true;
  }
  delay(1);
}
