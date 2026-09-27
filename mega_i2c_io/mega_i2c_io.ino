/*
Arduino MEGA - I2C digital/analog IO slave

Ports:
  Digital: 0 .. 53
  Analog:  A0..A15  (= pin index 0..15 nei comandi CMD_ANALOG_READ)

  I2C hardware sul Mega e' su SDA=20, SCL=21 (NON A4/A5 come sull'Uno):
  questi due pin vengono quindi esclusi dalla lettura digitale generica.
*/

//CON WATCHDOG

#include <Arduino.h>
#include <Wire.h>

#define DEBUG // rimuovere debug per liberare pin 0 e 1 (Serial) come IO

#define I2C_ADDRESS 8

// 54 pin digitali (0..53) -> servono ceil(54/8) = 7 byte
#define DIGITAL_PIN_COUNT 54
#define DIGITAL_BUFFER_LEN 7

// I2C hardware sul Mega: da escludere dalla lettura digitale generica
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

void onRequest();
void onReceive(int);

volatile unsigned long lastCommMillis = 0;


void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println(F("Init "));
#endif

  analogReference(INTERNAL1V1); // sul Mega INTERNAL e' INTERNAL1V1 (1.1V) o INTERNAL2V56

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);

#ifdef DEBUG
  Serial.println(F("Wire ok"));
#endif

  lastCommMillis = millis();
}

void loop()
{
  checkCommWatchdog();
}

// --- Watchdog comunicazione I2C ------------------------------------------
// Se l'ESP (master) crasha/si riavvia mentre un relè e' attivo, senza questo
// controllo il Mega continuerebbe a pilotare quel pin per sempre. Il master
// interroga il Mega di continuo (CMD_DIGITAL_READ ogni ~100ms lato ESP), quindi
// se non arriva NESSUN comando I2C per COMM_TIMEOUT_MS vuol dire che il master
// e' sparito: a quel punto rilasciamo tutti i pin configurati come OUTPUT
// riportandoli a INPUT (stesso stato "neutro" del boot), cosi' il relè si
// affida al proprio pull-up/pull-down di bordo invece di restare pilotato.
// Quando l'ESP torna, il suo resync automatico riconfigura i pin da capo.
#define COMM_TIMEOUT_MS 1000UL

volatile bool failsafeTripped = false;
volatile byte outputPinsMask[DIGITAL_BUFFER_LEN]; // bit=1 -> pin configurato come OUTPUT

void setOutputPinBit(int pin, bool isOutput)
{
  if (isOutput)
    outputPinsMask[pin / 8] |= (1 << (pin % 8));
  else
    outputPinsMask[pin / 8] &= ~(1 << (pin % 8));
}

void checkCommWatchdog()
{
  if (failsafeTripped) return;

  unsigned long lastComm;
  noInterrupts();
  lastComm = lastCommMillis;
  interrupts();

  if (millis() - lastComm < COMM_TIMEOUT_MS) return;
  failsafeTripped = true;
  for (int pin = 0; pin < DIGITAL_PIN_COUNT; pin++)
  {
    if (pin == I2C_SDA_PIN || pin == I2C_SCL_PIN)
      continue;
    if (outputPinsMask[pin / 8] & (1 << (pin % 8)))
      pinMode(pin, INPUT);
  }
  for (byte b = 0; b < DIGITAL_BUFFER_LEN; b++)
    outputPinsMask[b] = 0;

#ifdef DEBUG
  Serial.println(F("WATCHDOG: master I2C perso, pin OUTPUT rilasciati a INPUT"));
#endif
}

// --- Buffer condiviso con onRequest -----------------------------------

volatile byte buffer[DIGITAL_BUFFER_LEN]; // riusato anche per readAnalog (2 byte)
volatile byte len = 1;

// --- Lettura digitale ----------------------------------------------------

void readDigital()
{
  len = DIGITAL_BUFFER_LEN;

  for (byte b = 0; b < DIGITAL_BUFFER_LEN; b++)
    buffer[b] = 0;

  for (int pin = 0; pin < DIGITAL_PIN_COUNT; pin++)
  {
    if (pin == I2C_SDA_PIN || pin == I2C_SCL_PIN)
      continue; // occupati dal bus I2C

    if (digitalRead(pin))
      buffer[pin / 8] |= (1 << (pin % 8));
  }

#ifdef DEBUG_READ
  Serial.print(F("Read "));
  Serial.print(DIGITAL_BUFFER_LEN);
  Serial.print(F(" bytes: "));
  for (byte b = 0; b < DIGITAL_BUFFER_LEN; b++)
  {
    Serial.print(buffer[b]);
    Serial.print(' ');
  }
  Serial.println();
#endif
}

// --- Lettura analogica ----------------------------------------------------

// pin: indice logico 0..15 -> A0..A15
void readAnalog(int pin)
{
  int val = analogRead(A0 + pin);
  len = 2;
  buffer[0] = val & 0xFF;
  buffer[1] = (val >> 8) & 0b11;
#ifdef DEBUG_READ
  Serial.print(F("Read analog pin A"));
  Serial.println(pin);
#endif
}

void onRequest()
{
  lastCommMillis = millis();
  failsafeTripped = false;
  Wire.write(const_cast<uint8_t *>(buffer), len);
}

// --- Protocollo comandi -----------------------------------------------
// NB: rispetto alla versione Uno, la lettura analogica non e' piu' codificata
// nei bit alti del comando stesso (li' bastavano 3 bit per 8 canali A0-A7).
// Sul Mega servono 16 canali (A0-A15, 4 bit), che andrebbero a sovrapporsi
// con CMD_SETUP_ANALOG_INTERNAL/DEFAULT. Per questo CMD_ANALOG_READ ora e'
// un comando singolo seguito da un secondo byte con l'indice del pin (0-15).

#define CMD_DIGITAL_READ 0x0
#define CMD_ANALOG_READ 0x1 // + 1 byte: indice pin analogico 0..15 (A0..A15)

#define CMD_WRITE_ANALOG 0x2
#define CMD_WRITE_DIGITAL_HIGH 0x3
#define CMD_WRITE_DIGITAL_LOW 0x4

#define CMD_SETUP_PIN_OUTPUT 0x5
#define CMD_SETUP_PIN_INPUT_PULLUP 0x6
#define CMD_SETUP_PIN_INPUT 0x7

#define CMD_SETUP_ANALOG_INTERNAL 0x10
#define CMD_SETUP_ANALOG_DEFAULT 0x11

void onReceive(int numBytes)
{
  lastCommMillis = millis();
  failsafeTripped = false;

#ifdef DEBUG_READ
  Serial.print("Received bytes: ");
  Serial.println(numBytes);
#endif
  int cmd = Wire.read();

  switch (cmd)
  {
  case CMD_DIGITAL_READ:
    readDigital();
    return;

  case CMD_ANALOG_READ:
  {
    int pin = Wire.read(); // 0..15
    readAnalog(pin);
    return;
  }
  }

  int pin = Wire.read();

  switch (cmd)
  {
  case CMD_WRITE_DIGITAL_HIGH:
  case CMD_WRITE_DIGITAL_LOW:
  {
    bool output = cmd == CMD_WRITE_DIGITAL_HIGH;
    digitalWrite(pin, output);
#ifdef DEBUG
    Serial.print(F("Pin "));
    Serial.print(pin);
    Serial.println(output ? F(" HIGH") : F(" LOW"));
#endif
    break;
  }
  case CMD_WRITE_ANALOG:
  {
    int val = Wire.read() | (Wire.read() << 8);
    analogWrite(pin, val);
#ifdef DEBUG
    Serial.print(F("Pin "));
    Serial.print(pin);
    Serial.print(F(" Analog write "));
    Serial.println(val);
#endif
    break;
  }
  case CMD_SETUP_PIN_OUTPUT:
    pinMode(pin, OUTPUT);
    setOutputPinBit(pin, true);
#ifdef DEBUG
    Serial.print(F("Pin "));
    Serial.print(pin);
    Serial.println(F(" OUTPUT"));
#endif
    break;
  case CMD_SETUP_PIN_INPUT:
    pinMode(pin, INPUT);
    setOutputPinBit(pin, false);
#ifdef DEBUG
    Serial.print(F("Pin "));
    Serial.print(pin);
    Serial.println(F("INPUT"));
#endif
    break;
  case CMD_SETUP_PIN_INPUT_PULLUP:
    pinMode(pin, INPUT_PULLUP);
    setOutputPinBit(pin, false);
#ifdef DEBUG
    Serial.print(F("Pin "));
    Serial.print(pin);
    Serial.println(F("INPUT PULLUP"));
#endif
    break;
  case CMD_SETUP_ANALOG_INTERNAL:
    analogReference(INTERNAL1V1);
#ifdef DEBUG
    Serial.println(F("Analog reference INTERNAL1V1"));
#endif
    break;
  case CMD_SETUP_ANALOG_DEFAULT:
    analogReference(DEFAULT);
#ifdef DEBUG
    Serial.println(F("Analog reference DEFAULT"));
#endif
    break;
  }
}
