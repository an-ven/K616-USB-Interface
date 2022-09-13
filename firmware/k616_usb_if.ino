/* 
BSD 2-Clause License

Copyright (c) 2022, Andraž Vene
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <EEPROM.h>

// ===== GENERAL PROGRAM SETTINGS ==========
#define ID_STRING         "K616_USB_IF"   // ID string
#define UPDATE_TIMEOUT    250             // Update timeout in [ms]
#define EE_SETTINGS_ADDR  0               // EEPROM settings data structure start adderss

// ===== IO PIN MAPING =====================
// "Gated clock", "Count now" and "Polarity"
#define CN_IN         4     // PD4 (ICP1)
#define GC_IN         12    // PD6 (T1)
#define POL_IN        8     // PB4
// Range switch exponent inputs
#define EXP_1E0_IN    A3    // PF4
#define EXP_2E0_IN    A2    // PF5
#define EXP_4E0_IN    A1    // PF6
#define EXP_8E0_IN    A0    // PF7
#define EXP_1E1_IN    A4    // PF1
#define EXP_POL_IN    A5    // PF0
// Range switch function index inputs
#define FN1_IN        0     // PD2
#define FN2_IN        1     // PD3
// Sensitivity indicator outputs
#define SENS_R1_IN    9     // PB5
#define SENS_R2_IN    10    // PB6
#define SENS_R4_IN    11    // PB7
// Sensitivity selection inputs
#define SENS_R1_OUT   15    // PB1
#define SENS_R2_OUT   16    // PB2
#define SENS_R4_OUT   14    // PB3
#define SENS_MR_OUT   5     // PC6
// Zero check
#define ZCHK_IN       7     // PE6
#define ZCHK_OUT      6     // PD7
// Other outputs
#define HOLD_OUT      2     // PD1
#define LED_OUT       13    // PC7

// Range switch function index enumerators
enum function_e {
  FN_VOLT,
  FN_AMP,
  FN_COULOMB,
  FN_OHM
};

// Unit prefix characters
const char prefix_char[] = {
  'f',  // femto
  'p',  // pico
  'n',  // nano
  'u',  // micro
  'm',  // milli
  ' ',  // none
  'k',  // kilo
  'M',  // mega
  'G',  // giga
  'T'   // tera
};

// Global Variables
volatile uint8_t  count_available = 0;  // New gated clock pulse count available flag 
volatile uint16_t count_latest    = 0;  // Latest gated clock pulse count

static String output_str;               // Output string variable

struct {
  uint8_t remote_zero : 1;  // Remote zero check enabled flag
  uint8_t manual_sens : 1;  // Manual sensitivity flag
  uint8_t disp_hold : 1;    // Display hold flag
} instrument_flags;

struct {
  uint8_t meas_stream : 1;  // Measurement streaming flag
  uint8_t print_once : 1;   // Print single latest measurement flag
  uint8_t human_units : 1;  // Human friendly unit display flag
} interface_flags;


// ===== Setup routine ============================================================================
void setup() {
  // Configure IO pin directions and pullups
  pinMode(CN_IN, INPUT_PULLUP);
  pinMode(GC_IN, INPUT_PULLUP);
  pinMode(POL_IN, INPUT_PULLUP);
  pinMode(EXP_1E0_IN,INPUT_PULLUP);
  pinMode(EXP_2E0_IN,INPUT_PULLUP);
  pinMode(EXP_4E0_IN,INPUT_PULLUP);
  pinMode(EXP_8E0_IN,INPUT_PULLUP);
  pinMode(EXP_1E1_IN,INPUT_PULLUP);
  pinMode(EXP_POL_IN,INPUT_PULLUP);
  pinMode(FN1_IN, INPUT_PULLUP);
  pinMode(FN2_IN, INPUT_PULLUP);
  pinMode(SENS_R1_IN, INPUT_PULLUP);
  pinMode(SENS_R2_IN, INPUT_PULLUP);
  pinMode(SENS_R4_IN, INPUT_PULLUP);
  pinMode(SENS_R1_OUT, OUTPUT);
  pinMode(SENS_R2_OUT, OUTPUT);
  pinMode(SENS_R4_OUT, OUTPUT);
  pinMode(SENS_MR_OUT, OUTPUT);
  pinMode(ZCHK_IN, INPUT_PULLUP);
  pinMode(ZCHK_OUT, OUTPUT);
  pinMode(HOLD_OUT, OUTPUT);
  pinMode(LED_OUT, OUTPUT);  

  // Configure Timer/Counter 1
  // - All waveform outputs disabled (normal port operation) -> COM1x{1:0} = 0
  // - Clock source: falling edge on pin T1(PD6) -> CS1{2:0}=6
  // - Waveform generator mode: Normal -> WGM1{3:0}=0
  // - Configure input capture: falling edge on pin ICP1(PD4) -> ICES1=0
  // - Enable Input Capture Noise Canceler -> ICNC1 -> 1
  TCCR1A = 0<<COM1A1 | 0<<COM1A0 | 0<<COM1B1 | 0<<COM1B0 | 0<<COM1C1 | 0<<COM1C0 | 0<<WGM11 | 0<<WGM10;
  TCCR1B = 1<<ICNC1 | 0<<ICES1 | 0<<WGM13 | 0<<WGM12 |1<<CS12 | 1<<CS11 | 0<<CS10;
  // Enable Timer/Counter 1 Interrupt on input capture -> ICIE1=1
  TIMSK1 = 1<<ICIE1;
  // Clear counter
  TCNT1 = 0;

  // Initialize USB serial comm port
  Serial.begin(115200);
  
  // Load interface settings from EEPROM
  EEPROM.get(EE_SETTINGS_ADDR, interface_flags);

  // Enable interrupts
  interrupts();
}

// ===== Timer/Counter 1 Input capture interrupt service routine ==================================
ISR(TIMER1_CAPT_vect) {
  noInterrupts();         // Disable interrupts

  if (PIND & 1<<PIND4) {  // On rising edge
    TCNT1 = 0;              // Reset Timer/Counter 1 count register to zero
    TCCR1B &= ~(1<<ICES1);  // Set input capture to trigger on falling edge (Clear ICES1 bit)
    TIFR1 &= ~(1<<ICF1);    // Clear Input Capture Flag ICF1

  } else {                // On falling edge
    count_latest = ICR1;    // Save captured pulse count
    TCCR1B |= 1<<ICES1;     // Set input capture to trigger on rising edge (Set ICES1 bit)
    TIFR1 &= ~(1<<ICF1);    // Clear Input Capture Flag ICF1
    count_available = 1;    // Set new pulse count available flag
  }
  interrupts();           // Reenable interrupts
}

// ===== Main program loop ========================================================================
void loop() {

  static long timeout_ts;               // Update timeout timestamp variable
  
  if (count_available) {                // When new pulse count is available
    digitalWrite(13, HIGH);
    timeout_ts = millis()+UPDATE_TIMEOUT; // Save current update timestamp
    count_available = 0;                  // Clear new pulse count available flag 

    if (interface_flags.human_units) {    // if human friendly output ie senbled
      makeHumanString(false);               // Generate human friendly output string
    } else {
      makeOutStr(false);                    // Generate normal compact output string  
    }
    
    if (interface_flags.meas_stream) {    // If continuous output is enabled
      Serial.println(output_str);           // Send output string over USB serial
    }

    digitalWrite(13, LOW);

  } else if (timeout_ts < millis()) {   // If there is no new pulse count available after a timeout period 
    count_latest = 9999;                  // Set current count to something ridiculous
    timeout_ts = millis()+UPDATE_TIMEOUT; // Save current update timestamp
    
    if (interface_flags.human_units) {    // if human friendly output ie senbled
      makeHumanString(true);                // Generate human friendly output string indicating timeout error
    } else {
      makeOutStr(true);                     // Generate normal compact output string indicating timeout error
    }

    if (interface_flags.meas_stream) {    // If continuous output is enabled
      Serial.println(output_str);           // Send output string over USB serial
    }
  }

  if (interface_flags.print_once) {     // If print once flag is set
    Serial.println(output_str);           // Send output string over USB serial
    interface_flags.print_once = 0;       // Clear print_once flag
  }
  
  // Read and process incomming commands
  getCommands();
}


// ===== Generate compact output string ===================================================================
void makeOutStr(uint8_t timeout) {
  output_str = "";  // Start with an empty string

  uint8_t s  = getSensitivity();        // Get sensitivity setting number
  int8_t  e  = getExponent();           // Read ranged switch exponent value
  
  // Read polarity indicator input add apropriate sign to output string
  output_str += (digitalRead(POL_IN)) ? "-" : "+";
  // Add gated clock pulse count scaled by raw sensitivity factor to output string
  output_str += String((float)count_latest * pow(10, s-8), 8-s);
  // Add base 10 exponent
  output_str += "E";
  output_str += String(e);

  // Add units to output string
  output_str += " ";
  output_str += getUnit();

  // Add instrument state indicators
  output_str += " ";
  if (timeout) {        // if count timeout condition is present
    output_str += "ERR";  // Print count timeout error
  
  } else {              // else add normal status indicators to output string
    output_str += (count_latest < 2000) ? (!digitalRead(ZCHK_IN) ? "Z" : "N") : "O";  // Normal/Overflow/ZeroCheck indicator
    if (instrument_flags.remote_zero) output_str += "R";    // Display remote zero check indicator
    output_str += instrument_flags.manual_sens ? "M" : "A"; // Auto/Manual sensitivity indicator
    if (instrument_flags.disp_hold) output_str += "H";      // Display hold indicator
  }
}

// ===== Generete human friendly output string ====================================================
void makeHumanString(uint8_t timeout) {
  output_str = "";            // Start with an empty string

  int8_t e = getExponent();   // Get range switch exponent
  e += getSensitivity() - 5;  // Add sensitivity setting exponent to range switch exponent

  int8_t i_pfx;               // Unit prefix character index variable
  int8_t e_mod;               // Mantissa scaling exponent variable

  // Calculate prefix caharacter index and mantissa scaling exponent
  if (e < 0) {
    i_pfx = 5 + ((e-2) / 3);
    e_mod = (15+e) % 3;
  } else {
    i_pfx = 5 + (e / 3);
    e_mod = e % 3;
  }

  // Calculate mantissa
  float m = (float)count_latest * pow(10, e_mod-3);
  
  // Read polarity indicator input add apropriate sign to output string
  output_str += (digitalRead(POL_IN)) ? "-" : "+";
  // Add mantissa to outout string
  output_str += String(m, 3-e_mod);
  output_str += " ";
  // Add unit with apropriate prefix to output string
  output_str += prefix_char[i_pfx];
  output_str += getUnit();

  // Add instrument state indicators
  output_str += " ";
  if (timeout) {        // if count timeout condition is present
    output_str += "Count Error";  // Print count timeout error
  } else {              // else add normal status indicators to output string
    output_str += (count_latest < 2000) ? (!digitalRead(ZCHK_IN) ? "Zero " : "Normal ") : "Overflow ";  // Normal/Overflow/ZeroCheck indicator
    if (instrument_flags.remote_zero) output_str += "Remote ";        // Display remote zero check indicator
    output_str += instrument_flags.manual_sens ? "Manual " : "Auto "; // Auto/Manual sensitivity indicator
    if (instrument_flags.disp_hold) output_str += "Display Hold";     // Display hold indicator
  }
}

// ===== Get range switch exponent value ==========================================================
int8_t getExponent() {
  int8_t e = (~(PINF) & 0b11110000)>>4;
  if (~(PINF) & 0b00000010) e += 10;
  if ((PINF) & 0b00000001) e *= -1;
  return e;
}

// ===== Get sensitivity setting number ===========================================================
int8_t getSensitivity() {
  uint8_t s  = (PINB & 0b11100000)>>5;  // Read raw sensitivity setting number
  if (s < 3) s = 3;                     // Constrain sensitivity setting number
  return s;
}

// ===== Get range switch unit string =============================================================
String getUnit() {
  uint8_t f = (PIND & 0b00001100)>>2;
  switch(f) {
    case FN_VOLT:
      return "V";
      break;
    case FN_COULOMB:
      return "C";
      break;
    case FN_AMP:
      return "A";
      break;
    case FN_OHM:
      return "Ohm";
      break;
  }
}

// ===== Set sensitivity ==========================================================================
void setSensitivity(int8_t sens) {
  if (sens == 0) {                        // Clear sensitivity selection and manual range pins
    PORTB &= 0b11110001;                    // Clear sensitivity select pins PB1, PB2 and PB3
    digitalWrite(SENS_MR_OUT, LOW);         // Disable manual sensitivity
    instrument_flags.manual_sens = 0;                  // Clear manual sensitivity flag

  } else if ((sens < 6) && (sens > 0)) {  // Disable auto ranging and set apropriate sensitivity
    digitalWrite(SENS_MR_OUT, HIGH);        // Enable manual sensitivity
    uint8_t temp = PORTB & 0b11110001;      // Save current PORTB state without sens. sel. bits
    PORTB = temp | (5 - sens)<<1;           // Write PORTB with correct sens. sel bits set
    instrument_flags.manual_sens = 1;                  // Set manual sensitivity flag

  } else {                                // Print error message
    Serial.println("Error: Invalid sensitivity range number");
  }
}

// ===== Read serial and process incomming commands ===============================================
void getCommands() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r':   // Set sensitivity
        setSensitivity(Serial.parseInt());
        break;
      case 'Z':   // Enable remote zero check
        digitalWrite(ZCHK_OUT, HIGH);
        instrument_flags.remote_zero = 1;
        break;
      case 'z':   // Disable remote zero check
        digitalWrite(ZCHK_OUT, LOW);
        instrument_flags.remote_zero = 0;
        break;
      case 'H':   // Enable instrument display hold
        digitalWrite(HOLD_OUT, HIGH);
        instrument_flags.disp_hold = 1;
        break;
      case 'h':   // Disable instrument display hold
        digitalWrite(HOLD_OUT, LOW);
        instrument_flags.disp_hold = 0;
        break;
      case 'S':   // Enable measurement streaming (print every time new measurement is available)
        interface_flags.meas_stream = 1;
        break;
      case 's':   // Disable measurement streaming
        interface_flags.meas_stream = 0;
        break;
      case 'p':   // Print current measurement (when streaming is disabled)
        interface_flags.print_once = 1;
        break;
      case 'U':   // Print measurements in human friendly format
        interface_flags.human_units = 1;
        break;
      case 'u':   // Print measurements in raw format
        interface_flags.human_units = 0;
        break;
      case 'f':   // Save current interface configuration to EEPROM
        EEPROM.put(EE_SETTINGS_ADDR, interface_flags);
        Serial.println("Configuration saved.");
        break;
      case 'i':   // Print ID string
        Serial.println(ID_STRING);
        break;
      case '?':   // Print help message
        printHelpMsg();
        break;
      default:    // Error message
        Serial.print("Error: Unknown command character '");
        Serial.print(c);
        Serial.println("', send '?' to list available commands");
        break;
    }
  }
}

// ===== Print help message =======================================================================
void printHelpMsg() {
  Serial.println("Instrument controll commands:");
  Serial.println("  'Z|z'  -> Enable|Disable remote zero check");
  Serial.println("  'H|h'  -> Enable|Disable display hold");
  Serial.println("  'r[x]' -> Set sensitivity x=[0-5]");
  Serial.println("Interface controll commands:");
  Serial.println("  'S|s'  -> Enable|Disable continuous measurement printing");
  Serial.println("   'p'   -> Print latest measurement once");
  Serial.println("  'U|u   -> Enable|Disable human friendly output format");
  Serial.println("    f    -> Save current configuration to load on startup");
  Serial.println("   'i'   -> Print ID string");
  Serial.println("   '?'   -> Print this help message");
}