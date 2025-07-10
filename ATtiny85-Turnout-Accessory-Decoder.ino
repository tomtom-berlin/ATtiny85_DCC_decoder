#include <NmraDcc.h>

// per PWM können drei Weichen bedient werden
// PWM muss manuell erzeugt werden im Bereich
// für SG90: 50 Hz, 500 µs für 0 ° bis 2400 µs für 180 °
// für Micro Linear Servo: 333Hz, Linksanschlag: 950 µs, Rechtsanschlag: 2050 µs
/*
   Arduino ATtiny85                                 
              +--------------------+                
              |                    |   Pin             
         |-----                [ ] |    5  PB5  Programmiertaster (IN--/\10k/\---PB5---/\4.7k/\---VCC)
         | [-]    ATtiny85     [ ] |    4  PB4  Weiche 3
    USB  | [ ]                 [ ] |    3  PB3  ACK
         | [ ]                 [ ] |    2  PB2  DCC Signal Eingang 
         | [+]                 [ ] |    1  PB1  Weiche 2
         |-----                [ ] |    0  PB0  Weiche 1
              |  [ ] [ ] [ ]    [+]|                
              +--------------------+                
                +5V GND  Vin                        
*/

/* CV-Plan für die "herstellereigenen"  CVs:
 | x = 50 für Weiche 1, 60 für Weiche 2, 70 für Weiche 3 
 |  CV |  Inhalt                                             | Defaultwert
 +-----+-----------------------------------------------------------------
 |  33 |  Servo-Frequenz LSB (Hz)                            |  50,  SG90
 |  34 |  Servo-Frequenz MSB (Hz)                            |   0 
 |  35 |  min. Impulslänge LSB                               | 244 (= 500), SG90
 |  36 |  min. Impulslänge MSB                               |   1
 |  37 |  max. Impulslänge LSB                               |  96 (= 2400), SG90
 |  38 |  max. Impulslänge MSB                               |   9
 |  39 ]  Schrittweite (µs)                                  |   5
 |  40 |  Wartezeit zw. zwei Schritten (ms)                  |  15
 +-----+-----------------------------------------------------------------
 |  n0 |  Winkel Weiche x links                              | 30  (R=0)
 |  n1 |  Winkel Weiche 1 rechts                             | 150 (R=1)
 |  n2 |  Grundstellung Weiche 1                             | 0 (R=0)
 +-----+-----------------------------------------------------------------
*/

#ifndef ATTINY_CORE
#define __DEBUG__
#endif


NmraDcc Dcc;

#define CALC_U32_CV_VALUE(cv_msb, cv_lsb) ((uint32_t)((Dcc.getCV(cv_msb) * 256) + Dcc.getCV(cv_lsb)))
#define CALC_U32_DEG_TO_PULSE(deg) ((uint32_t)map((deg), 0, 180, minPulse, maxPulse))

#ifdef __DEBUG__
// Define the Arduino input pin number for the DCC signal
#define DCC_PIN 2
// servicemode pin
#define ACK_PIN 7
#define RST_PIN 1
// Turnout pins
#define TURNOUT_1_PIN 3
#define TURNOUT_2_PIN 4
#define TURNOUT_3_PIN 5
#else
// Define the Arduino input pin number for the DCC signal
#define DCC_PIN 2
// servicemode pin
#define ACK_PIN 3
#define RST_PIN 5
// Turnout pins
#define TURNOUT_1_PIN 0
#define TURNOUT_2_PIN 1
#define TURNOUT_3_PIN 4
#endif

#define DEFAULT_LEFT 60
#define DEFAULT_RIGHT 120
#define DEFAULT_WAIT_MILLIS 15
#define DEFAULT_STEP_MICROS 5



typedef enum {
  SG90_FREQ_MSB = 0,
  SG90_FREQ_LSB = 50,
  SG90_0DEG_MSB = 2,
  SG90_0DEG_LSB = 3,
  SG90_180DEG_MSB = 9,
  SG90_180DEG_LSB = 81,
  // MIL_FREQ_MSB = 1,
  // MIL_FREQ_LSB = 77,
  // MIL_0DEG_MSB = 3,
  // MIL_0DEG_LSB = 142,
  // MIL_180DEG_MSB = 8,
  // MIL_180DEG_LSB = 42,
} PWM_TIMES;

typedef enum {
  CV_SERVO_FREQ_MSB = 33,
  CV_SERVO_FREQ_LSB = 34,
  CV_SERVO_0DEG_MSB = 35,
  CV_SERVO_0DEG_LSB = 36,
  CV_SERVO_180DEG_MSB = 37,
  CV_SERVO_180DEG_LSB = 38,
  CV_STEP_MICROS = 39,
  CV_WAIT_MILLIS = 40,

  CV_TURNOUT_1_LEFT = 50,
  CV_TURNOUT_1_RIGHT = 51,
  CV_TURNOUT_1_POS = 52,

  CV_TURNOUT_2_LEFT = 60,
  CV_TURNOUT_2_RIGHT = 61,
  CV_TURNOUT_2_POS = 62,

  CV_TURNOUT_3_LEFT = 70,
  CV_TURNOUT_3_RIGHT = 71,
  CV_TURNOUT_3_POS = 72,
} CV_LIST;

typedef struct {
  uint8_t pin;
  uint32_t pulse;
} SERVO_PIPE;

SERVO_PIPE servo_pipe[5];

typedef struct {
  uint8_t pin;
  uint32_t left;
  uint32_t right;
  uint32_t pos;
} TURNOUT_DATA;

TURNOUT_DATA turnout[] = {
  { .pin = TURNOUT_1_PIN, .left = 0, .right = 0, .pos = 0 },
  { .pin = TURNOUT_2_PIN, .left = 0, .right = 0, .pos = 0 },
  { .pin = TURNOUT_3_PIN, .left = 0, .right = 0, .pos = 0 },
};

uint32_t minPulse = 515;
uint32_t maxPulse = 2385;
uint32_t middlePos = (uint32_t)((minPulse + maxPulse) / 2);
uint32_t period = (uint32_t)(1e6 / 50);
uint32_t pwm_timer = 0UL;

uint8_t active = 0x80;
uint32_t wait = 0UL;
uint32_t pos_soll = 0UL;
uint8_t step_micros = DEFAULT_STEP_MICROS;
uint8_t wait_millis = DEFAULT_WAIT_MILLIS;


uint16_t baseAddr = DEFAULT_ACCESSORY_DECODER_ADDRESS;

uint8_t pipe_in = 0;
uint8_t pipe_out = 0;

// This Example shows how to use the library as a DCC Accessory Decoder or a DCC Signalling Decoder
struct CVPair {
  uint16_t CV;
  uint8_t Value;
};

CVPair FactoryDefaultCVs[] = {
  { CV_ACCESSORY_DECODER_ADDRESS_LSB, DEFAULT_ACCESSORY_DECODER_ADDRESS & 0xFF },
  { CV_ACCESSORY_DECODER_ADDRESS_MSB, DEFAULT_ACCESSORY_DECODER_ADDRESS >> 8 },
  // { CV_SERVO_FREQ_LSB, MIL_FREQ_LSB },
  // { CV_SERVO_FREQ_MSB, MIL_FREQ_MSB },
  // { CV_SERVO_0DEG_LSB, MIL_0DEG_LSB },
  // { CV_SERVO_0DEG_MSB, MIL_0DEG_MSB },
  // { CV_SERVO_180DEG_LSB, MIL_180DEG_LSB },
  // { CV_SERVO_180DEG_MSB, MIL_180DEG_MSB },
  { CV_SERVO_FREQ_LSB, SG90_FREQ_LSB },
  { CV_SERVO_FREQ_MSB, SG90_FREQ_MSB },
  { CV_SERVO_0DEG_LSB, SG90_0DEG_LSB },
  { CV_SERVO_0DEG_MSB, SG90_0DEG_MSB },
  { CV_SERVO_180DEG_LSB, SG90_180DEG_LSB },
  { CV_SERVO_180DEG_MSB, SG90_180DEG_MSB },
  { CV_STEP_MICROS, DEFAULT_STEP_MICROS },
  { CV_WAIT_MILLIS, DEFAULT_WAIT_MILLIS },

  { CV_TURNOUT_1_LEFT, DEFAULT_LEFT },
  { CV_TURNOUT_1_RIGHT, DEFAULT_RIGHT },
  { CV_TURNOUT_1_POS, 0 },
  { CV_TURNOUT_2_LEFT, DEFAULT_LEFT },
  { CV_TURNOUT_2_RIGHT, DEFAULT_RIGHT },
  { CV_TURNOUT_2_POS, 0 },
  { CV_TURNOUT_3_LEFT, DEFAULT_LEFT },
  { CV_TURNOUT_3_RIGHT, DEFAULT_RIGHT },
  { CV_TURNOUT_3_POS, 0 },
};

// uncomment ONLY ONE of the next two lines
uint8_t FactoryDefaultCVIndex = 0;
// uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

static uint8_t in = 0;
void push(SERVO_PIPE p) {
  // #ifdef __DEBUG__
  //   if (pipe_in != in) {
  //     Serial.print("In: ");
  //     Serial.println(pipe_in);
  //     in = pipe_in;
  //   }
  // #endif
  servo_pipe[pipe_in++] = p;
  pipe_in %= (sizeof(servo_pipe) / sizeof(SERVO_PIPE));
}

static uint8_t out = 0;
SERVO_PIPE pull() {
  SERVO_PIPE p = { 0xff, 0 };
  // #ifdef __DEBUG__
  //   if (pipe_out != out) {
  //     Serial.print("OUT: ");
  //     Serial.println(pipe_out);
  //     out = pipe_out;
  //   }
  // #endif
  if (pipe_in != pipe_out) {
    p = servo_pipe[pipe_out++];
    pipe_out %= (sizeof(servo_pipe) / sizeof(SERVO_PIPE));
  }
  return p;
}

// #define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg(DCC_MSG *Msg) {
#ifdef __DEBUG__
  for(uint8_t i = 0; i < Msg->PreambleBits; i++) {
    Serial.print('1');
  }
  uint8_t e = 0;
  for(uint8_t i = 0; i < Msg->Size ; i++) {
    e ^= *(Msg->Data + i);
    Serial.print(" 0 ");
    Serial.print(*(Msg->Data + i), BIN); 
  }
  Serial.println(" 1");
#endif
  }
#endif

  void notifyCVResetFactoryDefault() {
#ifdef __DEBUG__
    Serial.println("Factory Reset");
#endif
    // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset
    // to flag to the loop() function that a reset to Factory Defaults needs to be done
    FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
  };

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
#define NOTIFY_CV_ACK
#ifdef NOTIFY_CV_ACK
  void notifyCVAck(void) {
    //  if (in_service_mode) {
    digitalWrite(ACK_PIN, HIGH);
    delay(6);
    digitalWrite(ACK_PIN, LOW);
    //  }
  }
#endif

#define NOTIFY_RESET
#ifdef NOTIFY_RESET
  void notifyReset(uint8_t hardreset) {
    if (!hardreset) {
      init_cvs();
    } else {
      exit(0);
    }
  }
#endif
  // Servo bewegen
  void servo_ctrl(uint8_t servo_pin, uint32_t pulse) {
#ifdef __DEBUG__
    // Serial.print(".");
    // Serial.print("\rPin: ");
    // Serial.print(servo_pin);
    // Serial.print("  ");
    Serial.print(pulse);
    Serial.print(" µs ");
#endif
    if (micros() < pwm_timer) {
      return;
    }
    digitalWrite(servo_pin, HIGH);
    uint32_t p = micros() + pulse - 8;
    pwm_timer = micros() + period;
    while (micros() < p) {
      ;
    }
    digitalWrite(servo_pin, LOW);
  }

  // Weiche einen Schritt weiter stellen, wenn Wartezeit um
  void ctrlTurnout() {
    if (millis() >= wait) {
      if (!(active & 0x80)) {
        if (turnout[active].pos != pos_soll) {
          if (turnout[active].pos > pos_soll) {
            turnout[active].pos -= step_micros;
          } else {
            turnout[active].pos += step_micros;
          }
          wait = millis() + wait_millis;
        }
        turnout[active].pos = constrain(turnout[active].pos, min(turnout[active].left, turnout[active].right), max(turnout[active].left, turnout[active].right));
        servo_ctrl(turnout[active].pin, turnout[active].pos);
        if (turnout[active].pos == pos_soll) {
#ifdef __DEBUG__
          Serial.println();
          Serial.print(turnout[active].pin);
          Serial.print(" erreicht: ");
          Serial.print(turnout[active].pos);
          Serial.println(" µs\n");
#endif
          active |= 0x80;
        }
      }
    }
  }

  // This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
  void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t R, uint8_t D) {
    Addr &= 0x7ff;
    uint16_t relAddr = Addr - baseAddr;  // relative address starting with 0
    uint8_t x = pipe_in;
    if (relAddr >= 0 && relAddr < sizeof(turnout) / sizeof(TURNOUT_DATA)) {
      if (D == 1) {
        SERVO_PIPE p = { turnout[relAddr].pin, R == 0 ? turnout[relAddr].left : turnout[relAddr].right };
        push(p);
#ifdef __DEBUG__
        Serial.print("Addr: ");
        Serial.print(Addr, HEX);
        Serial.print(" (");
        Serial.print(Addr, DEC);
        Serial.print("), Rel: ");
        Serial.print(relAddr);
        Serial.print(", R: ");
        Serial.print(R);
        Serial.print(", D: ");
        Serial.print(D);
        Serial.print(" (");
        Serial.print(p.pulse);
        Serial.println(")");
#endif
      }
    }
  }

  void init_cvs() {
    minPulse = CALC_U32_CV_VALUE(CV_SERVO_0DEG_MSB, CV_SERVO_0DEG_LSB);
    maxPulse = CALC_U32_CV_VALUE(CV_SERVO_180DEG_MSB, CV_SERVO_180DEG_LSB);
    period = 1e6 / CALC_U32_CV_VALUE(CV_SERVO_FREQ_MSB, CV_SERVO_FREQ_LSB);
    wait_millis = Dcc.getCV(CV_WAIT_MILLIS);
    step_micros = Dcc.getCV(CV_STEP_MICROS);
#ifdef __DEBUG__
    Serial.print("Frequenz: ");
    Serial.print(CALC_U32_CV_VALUE(CV_SERVO_FREQ_MSB, CV_SERVO_FREQ_LSB));
    Serial.print(" Hz, Periode: ");
    Serial.println(period);
#endif
    for (uint8_t i = 0; i < sizeof(turnout) / sizeof(TURNOUT_DATA); i++) {
      switch (i) {
        case 0:
          pinMode(turnout[i].pin, OUTPUT);
          digitalWrite(turnout[i].pin, LOW);
          turnout[i].left = CALC_U32_DEG_TO_PULSE(Dcc.getCV(CV_TURNOUT_1_LEFT));
          turnout[i].right = CALC_U32_DEG_TO_PULSE(Dcc.getCV(CV_TURNOUT_1_RIGHT));
          turnout[i].pos = middlePos;
          servo_ctrl(turnout[i].pin, middlePos);
          push({ turnout[i].pin, (Dcc.getCV(CV_TURNOUT_1_POS) == 0 ? turnout[i].left : turnout[i].right) });
          break;
        case 1:
          pinMode(turnout[i].pin, OUTPUT);
          digitalWrite(turnout[i].pin, LOW);
          turnout[i].left = CALC_U32_DEG_TO_PULSE(Dcc.getCV(CV_TURNOUT_2_LEFT));
          turnout[i].right = CALC_U32_DEG_TO_PULSE(Dcc.getCV(CV_TURNOUT_2_RIGHT));
          turnout[i].pos = middlePos;
          servo_ctrl(turnout[i].pin, middlePos);
          push({ turnout[i].pin, (Dcc.getCV(CV_TURNOUT_2_POS) == 0 ? turnout[i].left : turnout[i].right) });
          break;
        case 2:
          pinMode(turnout[i].pin, OUTPUT);
          digitalWrite(turnout[i].pin, LOW);
          turnout[i].left = CALC_U32_DEG_TO_PULSE(Dcc.getCV(CV_TURNOUT_3_LEFT));
          turnout[i].right = CALC_U32_DEG_TO_PULSE(Dcc.getCV(CV_TURNOUT_3_RIGHT));
          turnout[i].pos = middlePos;
          servo_ctrl(turnout[i].pin, middlePos);
          push({ turnout[i].pin, (Dcc.getCV(CV_TURNOUT_3_POS) == 0 ? turnout[i].left : turnout[i].right) });
          break;
      }
      pwm_timer = micros();
#ifdef __DEBUG__
      Serial.print("Weiche: ");
      Serial.print(i);
      Serial.print(", Pin: ");
      Serial.print(turnout[i].pin);
      Serial.print(", Links: ");
      Serial.print(turnout[i].left);
      Serial.print(", Rechts: ");
      Serial.print(turnout[i].right);
      Serial.print(", Grundstellung: ");
      Serial.println(turnout[i].pos == turnout[i].left ? "Gerade" : "Abzweig");
#endif
    }
  }

  void setFactoryDefaults() {
    if (Dcc.isSetCVReady()) {
      FactoryDefaultCVIndex--;  // Decrement first as initially it is the size of the array
      Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
#ifdef __DEBUG__
      Serial.print("CV ");
      Serial.print(FactoryDefaultCVs[FactoryDefaultCVIndex].CV);
      Serial.print(" = ");
      Serial.println(Dcc.getCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV));
#endif
    }
  }

  void setup() {
    uint8_t cv;
#ifdef __DEBUG__
    Serial.begin(115200);
#endif
    // Configure the DCC CV Programing ACK pin for an output
    pinMode(DCC_PIN, INPUT);
    pinMode(RST_PIN, INPUT_PULLUP);
    pinMode(ACK_PIN, OUTPUT);
    digitalWrite(ACK_PIN, LOW);
    for (uint8_t i = 0; i < sizeof(servo_pipe) / sizeof(SERVO_PIPE); i++) {
      servo_pipe[i].pin = 0xff;
      servo_pipe[i].pulse = 0;
    }
    Dcc.pin(0, DCC_PIN, 0);

    // Call the main DCC Init function to enable the DCC Receiver
    Dcc.initAccessoryDecoder(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE | CV29_EXT_ADDRESSING | FLAGS_AUTO_FACTORY_DEFAULT, 0);
    baseAddr = Dcc.getAddr();
#ifdef __DEBUG__
    Serial.print("Base Addr: ");
    Serial.print(baseAddr, DEC);
    Serial.print(" CV29: 0b");
    Serial.println(Dcc.getCV(29), BIN);
#endif

    init_cvs();
    delay(300);  // 0.3 Sekunden warten auf Einpendeln
    pwm_timer = micros() + period;
  }


  void loop() {
    // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
    Dcc.process();

    if (FactoryDefaultCVIndex) {
      setFactoryDefaults();
    } else {
      if (active & 0x80) {
        SERVO_PIPE p = pull();
        for (uint8_t i = 0; i < sizeof(turnout) / sizeof(TURNOUT_DATA); i++) {
          if (p.pin == turnout[i].pin && p.pulse != turnout[i].pos) {
            active = i;
            pos_soll = p.pulse;
          }
        }
      }
      ctrlTurnout();
    }
  }
