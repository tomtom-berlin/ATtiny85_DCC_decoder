/* Multifunktionsdekoder für Waggonlicht, dimmbar per CV
  DEFAULT-Einstellungen (nach factory reset)
---------------------------------------------
   F0 vorwärts schaltet Port B0
   F0 rückwärts schaltet Port B1
   F1 schaltet Port B4 

Jede Funktion von F0 .. F12 kann einem der drei Ausgänge zugeordnet werden
Jede Funktion kann einen PWM-Wert aktivieren

Programmierung ist möglich via Programmiergleis oder POM
*/

/*
   Arduino ATtiny85                                 
              +--------------------+                
              |                    |   Pin             
         |-----                [ ] |    5  PB5  --- 10kOhm --> VCC
         | [-]    ATtiny85     [ ] |    4  PB4  F10
    USB  | [ ]                 [ ] |    3  PB3  Ack  --> 470 Ohm -> PNP Basis, Emitter GND, Collector + 100 Ohm zwischen Vin/GND
         | [ ]                 [ ] |    2  PB2  DCC Signal Eingang --> 22kOhm --> Track B
         | [+]                 [ ] |    1  PB1  F9  
         |-----                [ ] |    0  PB0  F0  
              |  [ ] [ ] [ ]    [+]|                
              +--------------------+                
                +5V GND  Vin  ------> Graetzbrücke +
                 |   |
                 |   +--------------> Graetzbrücke -
                 +------------------> Supercap 0.047mF/5.4V +
                     

            Track A ----------------> Graetzbrücke ~
            Track B ----------------> Graetzbrücke ~
*/

/* CV-Plan für die "herstellereigenen"  CVs:
|  CV |  Inhalt                                             | Defaultwert
+-----+-----------------------------------------------------------------
|  33 |  Ausgang F0v (Spitzensignal) aktiv                  |  1 (PORT B0)
|  34 |  Ausgang F0r (Schlusssignal) aktiv                  |  2 (PORT B1)
|  35 |  Ausgang F1                                         |  16 (PORT B4)
|  36 |  Ausgang F2                                         |  0 (kann Wert 1 = Port B0, 2 = Port B1 oder 16 = Port B4 annehmen
|  37 |  Ausgang F3                                         |  0
|  38 |  Ausgang F4                                         |  0
|  39 |  Ausgang F5                                         |  0
|  40 |  Ausgang F6                                         |  0
|  41 |  Ausgang F7                                         |  0
|  42 |  Ausgang F8                                         |  0
|  43 |  Ausgang F9                                         |  0
|  44 |  Ausgang F10                                        |  0
|  45 |  Ausgang F11                                        |  0
|  46 |  Ausgang F12                                        |  0
+-----+-----------------------------------------------------------------
| 112 |  PWM Ausgang F0v                                    |  255
| 113 |  PWM Ausgang F0r                                    |  255
| 114 |  PWM Ausgang F1                                     |  255
| 115 |  PWM Ausgang F2                                     |  255
| 116 |  PWM Ausgang F3                                     |  255
| 117 |  PWM Ausgang F4                                     |  255
| 118 |  PWM Ausgang F5                                     |  255
| 119 |  PWM Ausgang F6                                     |  255
| 120 |  PWM Ausgang F7                                     |  255
| 121 |  PWM Ausgang F8                                     |  255
| 122 |  PWM Ausgang F9                                     |  255
| 123 |  PWM Ausgang F10                                    |  255
| 124 |  PWM Ausgang F11                                    |  255
| 125 |  PWM Ausgang F12                                    |  255
+-----+-----------------------------------------------------------------
*/
#ifndef ATTINY_CORE
#define __DEBUG__
#endif

#include <NmraDcc.h>
// Fn--- F0v F0r  F1  F2  F3  F4  F5  F6  F7  F8  F9 F10 F11 F12
// CV---  33  34  35  36  37  38  39  40  41  42  43  44  45  46
// PWM-- 112 113 114 115 116 117 118 119 120 121 122 123 124 125
typedef enum {
  CV_FLv = 33,
  CV_FLr,
  CV_F1,
  CV_F2,
  CV_F3,
  CV_F4,
  CV_F5,
  CV_F6,
  CV_F7,
  CV_F8,
  CV_F9,
  CV_F10,
  CV_F11,
  CV_F12,
} CV_FN_MAP;

typedef enum {
  CV_FN0v_PWM = 112,
  CV_FN0r_PWM,
  CV_FN1_PWM,
  CV_FN2_PWM,
  CV_FN3_PWM,
  CV_FN4_PWM,
  CV_FN5_PWM,
  CV_FN6_PWM,
  CV_FN7_PWM,
  CV_FN8_PWM,
  CV_FN9_PWM,
  CV_FN10_PWM,
  CV_FN11_PWM,
  CV_FN12_PWM,
} CV_FN_PWM;

typedef enum {
  FN0 = 1,
  FN1 = 2,
  FN2 = 4,
  FN3 = 8,
  FN4 = 16,
  FN5 = 32,
  FN6 = 64,
  FN7 = 128,
  FN8 = 256,
  FN9 = 512,
  FN10 = 1024,
  FN11 = 2048,
  FN12 = 4096,
} FN_STATE;

typedef enum {
  F0v = 0,
  F0r,
  F1,
  F2,
  F3,
  F4,
  F5,
  F6,
  F7,
  F8,
  F9,
  F10,
  F11,
  F12,
} PINDEX;

const uint8_t OUTPUT_NO_PIN = 0;
const uint8_t OUTPUT_PIN0 = 1;
const uint8_t OUTPUT_PIN1 = 2;
const uint8_t OUTPUT_PIN4 = 16;

const uint8_t DEFAULT_PWM = 255;  // PWM-Value

#ifdef __DEBUG__
const uint8_t DCC_PIN = 2;
const uint8_t DCC_ACK_PIN = 6;
const uint8_t F_Pin0 = 3;
const uint8_t F_Pin1 = 4;
const uint8_t F_Pin4 = 5;
#else
const uint8_t DCC_PIN = 2;      // DCC Input Pin
const uint8_t DCC_ACK_PIN = 3;  // Ack-Pin
const uint8_t F_Pin0 = 0;
const uint8_t F_Pin1 = 1;
const uint8_t F_Pin4 = 4;
#endif

uint16_t fn_state = 0b0000000000000000;  // je Bit Funktion an oder aus, Funktionen 0..12
uint16_t fn_state_last;                  // letzter Wert

bool direction_forward = false;
bool direction_forward_last;

uint8_t pwm_values[] = {
  0,  // F0v
  0,  // F0r
  0,  // F1
  0,  // F2
  0,  // F3
  0,  // F4
  0,  // F5
  0,  // F6
  0,  // F7
  0,  // F8
  0,  // F9
  0,  // F10
  0,  // F11
  0,  // F12
};

uint8_t pins[] = {
  0,  // F0v
  0,  // F0r
  0,  // F1
  0,  // F2
  0,  // F3
  0,  // F4
  0,  // F5
  0,  // F6
  0,  // F7
  0,  // F8
  0,  // F9
  0,  // F10
  0,  // F11
  0,  // F12
};

NmraDcc Dcc;

struct CVPair {
  uint16_t CV;
  uint8_t Value;
};

CVPair FactoryDefaultCVs[] = {
  { CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_MULTIFUNCTION_DECODER_ADDRESS },

  // These two CVs define the Long DCC Address
  { CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0 },
  { CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0 },

  { 2, 0 },  // unused but mandatory
  { 3, 0 },  // unused but mandatory
  { 4, 0 },  // unused but mandatory
  { 5, 0 },  // unused but mandatory

  // ONLY uncomment 1 CV_29_CONFIG line below as approprate DEFAULT IS SHORT ADDRESS
  //  {CV_29_CONFIG,          0},                               // Short Address 14 Speed Steps
  { CV_29_CONFIG, CV29_F0_LOCATION },  // Short Address 28/128 Speed Steps
  //  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION},   // Long  Address 28/128 Speed Steps

  { CV_FLv, OUTPUT_PIN0 },  // Funktion inaktiv == 0, sonst : Bit 0 => Pin0, Bit1 = Pin1, Bit4 = Pin4
  { CV_FLr, OUTPUT_PIN1 },
  { CV_F1, OUTPUT_PIN4 },
  { CV_F2, OUTPUT_NO_PIN },
  { CV_F3, OUTPUT_NO_PIN },
  { CV_F4, OUTPUT_NO_PIN },
  { CV_F5, OUTPUT_NO_PIN },
  { CV_F6, OUTPUT_NO_PIN },
  { CV_F7, OUTPUT_NO_PIN },
  { CV_F8, OUTPUT_NO_PIN },
  { CV_F9, OUTPUT_NO_PIN },
  { CV_F10, OUTPUT_NO_PIN },
  { CV_F11, OUTPUT_NO_PIN },
  { CV_F12, OUTPUT_NO_PIN },
  { CV_FN0v_PWM, DEFAULT_PWM },
  { CV_FN0r_PWM, DEFAULT_PWM },
  { CV_FN1_PWM, DEFAULT_PWM },
  { CV_FN2_PWM, DEFAULT_PWM },
  { CV_FN3_PWM, DEFAULT_PWM },
  { CV_FN4_PWM, DEFAULT_PWM },
  { CV_FN5_PWM, DEFAULT_PWM },
  { CV_FN6_PWM, DEFAULT_PWM },
  { CV_FN7_PWM, DEFAULT_PWM },
  { CV_FN8_PWM, DEFAULT_PWM },
  { CV_FN9_PWM, DEFAULT_PWM },
  { CV_FN10_PWM, DEFAULT_PWM },
  { CV_FN11_PWM, DEFAULT_PWM },
  { CV_FN12_PWM, DEFAULT_PWM },
};

// Uncomment this line below to force resetting the CVs back to Factory Defaults
//uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
// Or uncomment this DO NOT uncomment both
uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault() {
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
};

void notifyReset(uint8_t hardreset) {
  if (!hardreset) {
    init_cvs();
  } else {
    exit(0);
  }
}

void init_cvs(void) {
  pwm_values[F0v] = Dcc.getCV(CV_FN0v_PWM);
  pwm_values[F0r] = Dcc.getCV(CV_FN0r_PWM);
  pwm_values[F1] = Dcc.getCV(CV_FN1_PWM);
  pwm_values[F2] = Dcc.getCV(CV_FN2_PWM);
  pwm_values[F3] = Dcc.getCV(CV_FN3_PWM);
  pwm_values[F4] = Dcc.getCV(CV_FN4_PWM);
  pwm_values[F5] = Dcc.getCV(CV_FN5_PWM);
  pwm_values[F6] = Dcc.getCV(CV_FN6_PWM);
  pwm_values[F7] = Dcc.getCV(CV_FN7_PWM);
  pwm_values[F8] = Dcc.getCV(CV_FN8_PWM);
  pwm_values[F9] = Dcc.getCV(CV_FN9_PWM);
  pwm_values[F10] = Dcc.getCV(CV_FN10_PWM);
  pwm_values[F11] = Dcc.getCV(CV_FN11_PWM);
  pwm_values[F12] = Dcc.getCV(CV_FN12_PWM);
  pins[F0v] = Dcc.getCV(CV_FLv);
  pins[F0r] = Dcc.getCV(CV_FLr);
  pins[F1] = Dcc.getCV(CV_F1);
  pins[F2] = Dcc.getCV(CV_F2);
  pins[F3] = Dcc.getCV(CV_F3);
  pins[F4] = Dcc.getCV(CV_F4);
  pins[F5] = Dcc.getCV(CV_F5);
  pins[F6] = Dcc.getCV(CV_F6);
  pins[F7] = Dcc.getCV(CV_F7);
  pins[F8] = Dcc.getCV(CV_F8);
  pins[F9] = Dcc.getCV(CV_F9);
  pins[F10] = Dcc.getCV(CV_F10);
  pins[F11] = Dcc.getCV(CV_F11);
  pins[F12] = Dcc.getCV(CV_F12);

#ifdef __DEBUG__
  Serial.println();
  Serial.println(Dcc.getCV(1));
  for (int i = 0; i < 14; i++) {
    if (i < 10) {
      Serial.print("   ");

    } else {
      Serial.print("  ");
    }
    Serial.print(i);
  }
  Serial.println();
  for (int i = 0; i < 14; i++) {
    Serial.print(" ");
    Serial.print(pwm_values[i]);
  }
  Serial.println();
  for (int i = 0; i < 14; i++) {
    if (pins[i] < 10) {
      Serial.print("   ");
    } else {
      Serial.print("  ");
    }
    Serial.print(pins[i]);
  }
  Serial.println();

#endif
}

// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
#define NOTIFY_CV_ACK
#ifdef NOTIFY_CV_ACK
void notifyCVAck(void) {
#ifdef __DEBUG__
  Serial.println("ACK");
#endif
  digitalWrite(DCC_ACK_PIN, HIGH);
  delay(6);
  digitalWrite(DCC_ACK_PIN, LOW);
}
#endif

void setFactoryDefaults() {
  if (Dcc.isSetCVReady()) {
    FactoryDefaultCVIndex--;  // Decrement first as initially it is the size of the array
    Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}

void notifyDccSpeed(uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps) {
  if (Dir == DCC_DIR_FWD) {
    direction_forward = true;
  } else {
    direction_forward = false;
  }
}

void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState) {

  switch (FuncGrp) {
    case FN_0_4:
      if (FuncState & FN_BIT_00) {
        fn_state |= FN0;
      } else {
        fn_state &= ~FN0;
      }
      if (FuncState & FN_BIT_01) {
        fn_state |= FN1;
      } else {
        fn_state &= ~FN1;
      }
      if (FuncState & FN_BIT_02) {
        fn_state |= FN2;
      } else {
        fn_state &= ~FN2;
      }
      if (FuncState & FN_BIT_03) {
        fn_state |= FN3;
      } else {
        fn_state &= ~FN3;
      }
      if (FuncState & FN_BIT_04) {
        fn_state |= FN4;
      } else {
        fn_state &= ~FN4;
      }
      break;

    case FN_5_8:
      if (FuncState & FN_BIT_05) {
        fn_state |= FN5;
      } else {
        fn_state &= ~FN5;
      }
      if (FuncState & FN_BIT_06) {
        fn_state |= FN6;
      } else {
        fn_state &= ~FN6;
      }
      if (FuncState & FN_BIT_07) {
        fn_state |= FN7;
      } else {
        fn_state &= ~FN7;
      }
      if (FuncState & FN_BIT_08) {
        fn_state |= FN8;
      } else {
        fn_state &= ~FN8;
      }
      break;

    case FN_9_12:
      if (FuncState & FN_BIT_09) {
        fn_state |= FN9;
      } else {
        fn_state &= ~FN9;
      }
      if (FuncState & FN_BIT_10) {
        fn_state |= FN10;
      } else {
        fn_state &= ~FN10;
      }
      if (FuncState & FN_BIT_11) {
        fn_state |= FN11;
      } else {
        fn_state &= ~FN11;
      }
      if (FuncState & FN_BIT_12) {
        fn_state |= FN12;
      } else {
        fn_state &= ~FN12;
      }
  }
}

void activate(uint8_t pin, uint8_t pwm) {
  if (pin) {
#ifdef __DEBUG__
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.print(" PWM ");
    Serial.println(pwm);
#endif
    switch (pin) {
      case OUTPUT_PIN0:
        analogWrite(F_Pin0, pwm);
        break;
      case OUTPUT_PIN1:
        analogWrite(F_Pin1, pwm);
        break;
      case OUTPUT_PIN4:
        analogWrite(F_Pin4, pwm);
        break;
    }
  }
}

void deactivate(uint8_t pin) {
  if (pin) {
#ifdef __DEBUG__
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.println(" aus");
#endif
    switch (pin) {
      case OUTPUT_PIN0:
        digitalWrite(F_Pin0, 0);
        break;
      case OUTPUT_PIN1:
        digitalWrite(F_Pin1, 0);
        break;
      case OUTPUT_PIN4:
        digitalWrite(F_Pin4, 0);
        break;
    }
  }
}

void set_outputs(void) {
  for (uint8_t i = 0; i < 13; i++) {
    if (fn_state & (1 << i)) {
#ifdef __DEBUG__
      // Serial.print(i);
      // Serial.print(": ");
      // Serial.print(1 << i);
      // Serial.print(" > ");
      // Serial.println(fn_state & (1 << i));
#endif
      if (i == 0) {  // F0
        if (direction_forward) {
          deactivate(pins[F0r]);
          activate(pins[F0v], pwm_values[F0v]);
        } else {
          deactivate(pins[F0v]);
          activate(pins[F0r], pwm_values[F0r]);
        }
      } else {  // F1..F12 = 2..13
        activate(pins[i + 1], pwm_values[i + 1]);
      }
    } else {
      if (i == 0) {  // F0
        deactivate(pins[F0v]);
        deactivate(pins[F0r]);
      } else {
        deactivate(pins[i + 1]);
      }
    }
  }
}

void setup() {
#ifdef __DEBUG__
  Serial.begin(115200);
  // Configure the DCC CV Programing ACK pin for an output
  pinMode(DCC_PIN, INPUT);
  pinMode(DCC_ACK_PIN, OUTPUT);
  digitalWrite(DCC_ACK_PIN, LOW);
  pinMode(F_Pin0, OUTPUT);
  digitalWrite(F_Pin0, LOW);
  pinMode(F_Pin1, OUTPUT);
  digitalWrite(F_Pin1, LOW);
  pinMode(F_Pin4, OUTPUT);
  digitalWrite(F_Pin4, LOW);
#else
  // initialize the output pins
  DDRB |= 1 << PB0;  // Pin 0
  DDRB |= 1 << PB1;  // Pin 1
  DDRB |= 1 << PB3;  // Pin 3 (ACK)
  DDRB |= 1 << PB4;  // Pin 4

  PORTB = 0b00000000;
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(0, DCC_PIN, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, 25, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);
  delay(100);
  init_cvs();
  fn_state_last = ~fn_state;
  direction_forward_last = !direction_forward;
}

void loop() {
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  if (FactoryDefaultCVIndex) {
    setFactoryDefaults();
  }
  if (fn_state != fn_state_last || direction_forward_last != direction_forward) {
#ifdef __DEBUG__
    Serial.println(fn_state, BIN);
#endif
    set_outputs();
    fn_state_last = fn_state;
    direction_forward_last = direction_forward;
  }
}
