/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Pin assignments
//              bottom_jaw  tongues  flames
// Snake 1      0           13          9
// Snake 2      2           14          10
// Snake 3      4           15          11
// Snake 4      6           16          12

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ModbusRTUSlave.h>

uint16_t bottom_jaw_1 = 0;
uint16_t bottom_jaw_2 = 2;
uint16_t bottom_jaw_3 = 4;
uint16_t bottom_jaw_4 = 6;

// Timing
uint32_t currentTick = 0;

//  Modbus ID
const uint16_t id = 3;

//  Modbus Serial configuration
const uint32_t baud = 115200;
const uint8_t config = SERIAL_8E1;
const uint16_t bufferSize = 256;
const uint8_t dePin = 17;

//  Modbus data structures
const uint8_t coils = 5; // Flamethrowers + 1

uint32_t flameStartTick[coils] = { 0 };

uint8_t buffer[bufferSize];
ModbusRTUSlave modbus(Serial1, buffer, bufferSize, dePin);

//  Modbus Functions
int8_t coilRead(uint16_t address)
{
    return false;
}

bool coilWrite(uint16_t address, bool data)
{
    if (address <= coils && address > 0)
    {
        if (data)
        {   
            flameStartTick[address-1] = currentTick;
            return true;
        }
        return true;
    }
    return false;
}

// Called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
// Need to setup the second pwm board with address 0x41 (this board has A0 jumper bridged)
Adafruit_PWMServoDriver eyePWM = Adafruit_PWMServoDriver(0x41);

// Find these values from running servo-max-min-finder
#define SERVOFULLOPEN  185
#define SERVOFULLCLOSE  590

// Initialise arrays of size 8 (one value for each servo)
uint16_t random_open_vals[8];
uint16_t random_closed_vals[8];

int flamesequence[] = {13, 16, 14, 15}; //this is the sequence of relay triggers the numbers are the digital pins (need to be the correct digi pins on arduino
int randNumber;

void setup()
{
    // initialise Serial
    Serial.begin(9600);

    // Initialise Modbus
    // Physical Modbus setup on the MEGA2560 should be as follows:
    //  DE PIN  ->  17
    //  TX      ->  18
    //  RX      ->  19
    //  This leaves Serial free foor debugging if required!

    Serial1.begin(baud, config);
    modbus.begin(id, baud, config);
    modbus.configureCoils(coils, coilRead, coilWrite);

    // RS485 Enable / Disable
    pinMode(17, OUTPUT);

    // initialise relay pins for the tongues
    // I've changed this to initialise the flame pins too
    for (int i = 9; i <= 12; i++) {
        // set digital pins which trigger the relays to output mode
        pinMode(i, OUTPUT);
        // set relay to open (tongues in) //flames off (hopefully)
        digitalWrite(i, HIGH);
    }
    //Initialise relay pins for the flames
    for (int i = 13; i <= 16; i++) { //these are pin numbers
        // set digital pins which trigger the relays to output mode
        pinMode(i, OUTPUT);
        // set relay to open //flames off 
        digitalWrite(i, HIGH);
    }

    pwm.begin();
    pwm.setPWMFreq(60);

    eyePWM.begin();
    eyePWM.setPWMFreq(1600);  // This is the maximum PWM frequency

    // save I2C bitrate (not sure how this will influence the mouths and tongues
    uint8_t twbrbackup = TWBR;
    // must be changed after calling Wire.begin() (inside pwm.begin())
    TWBR = 12; // upgrade to 400KHz!

    // when programme starts, set all 8 servos to minimum position
    for (int i = 0; i < 8; i++) {
        pwm.setPWM(i, 0, SERVOFULLCLOSE);
    }
    // wait for them to get there

    currentTick = millis();
    
    delay(5000);
}

// function which causes each jaw to open and close to a random position
void random_mouths()
{
    // fill an array of random numbers to open the mouth to (different one for each servo). Number is between min and max servo vals
    for (int i = 0; i < 8; i++) {
        random_open_vals[i] = random(SERVOFULLOPEN, SERVOFULLCLOSE);
    }
    // same again with closed values. Numbers are between minimum servo val and 10 above this. Change 10 to make it more extreme
    for (int i = 0; i < 8; i++) {
        random_closed_vals[i] = random(SERVOFULLCLOSE - 10, SERVOFULLCLOSE);
    }
    // tells servo number i to go to the random open position at number i in the array
    Serial.println("mouth opening");
    // initialise array to keep track of which bottom jaws are open enough
    bool open_enough[4];
    for (int i = 0; i < 8; i++) {
        // open the mouths
        pwm.setPWM(i, 0, random_open_vals[i]);
        // store the pin numbers for bottom jaws which are open enough for the tongues in an array

        if (i == bottom_jaw_1) {
            if (random_open_vals[i] < ((SERVOFULLCLOSE + SERVOFULLOPEN) / 2)) {
                open_enough[0] = true;
            }
            else {
                open_enough[0] = false;
            }
        }

        if (i == bottom_jaw_2) {
            if (random_open_vals[i] < ((SERVOFULLCLOSE + SERVOFULLOPEN) / 2)) {
                open_enough[1] = true;
            }
            else {
                open_enough[1] = false;
            }
        }
        if (i == bottom_jaw_3) {
            if (random_open_vals[i] < ((SERVOFULLCLOSE + SERVOFULLOPEN) / 2)) {
                open_enough[2] = true;
            }
            else {
                open_enough[2] = false;
            }
        }
        if (i == bottom_jaw_4) {
            if (random_open_vals[i] < ((SERVOFULLCLOSE + SERVOFULLOPEN) / 2)) {
                open_enough[3] = true;
            }
            else {
                open_enough[3] = false;
            }
        }
        Serial.print("servo ");
        Serial.print(i);
        Serial.print(" at position ");
        Serial.println(random_open_vals[i]);
    }
    // trigger tongues for mouths which are open enough
    // true is for synchronised behaviour and false is for random
    tongue_out_in(false, open_enough);

    Serial.println();
    // waits for the servo to get there
    delay(800);
    Serial.println("mouth closing");
    // same with the closed positions
    for (int i = 0; i < 8; i++) {
        pwm.setPWM(i, 0, random_closed_vals[i]);
        Serial.print("servo ");
        Serial.print(i);
        Serial.print(" at position ");
        Serial.println(random_closed_vals[i]);
    }
    Serial.println();
    // waits for the servo to get there
    delay(800);
}

// function which makes all the jaws open and close synchronously
void synchronous_mouths() {
    // set all of the jaws to the same random open position
    uint16_t random_open = random(SERVOFULLOPEN, SERVOFULLCLOSE);
    for (int i = 0; i < 8; i++) {
        pwm.setPWM(i, 0, random_open);
    }

    Serial.print("servos at position ");
    Serial.println(random_open);

    bool empty[] = {};
    // if the mouths are open enough, stick the tongues out
    if (random_open < ((SERVOFULLCLOSE + SERVOFULLOPEN) / 2)) {
        // true is for synchronised behaviour and false is for random
        tongue_out_in(true, empty);
    }


    // wait for the servos to get there
    delay(800);
    // set all of the jaws to the same random closed position
    // Increase 10 to make the closed position more random
    uint16_t random_closed = random(SERVOFULLCLOSE - 10, SERVOFULLCLOSE);
    for (int i = 0; i < 8; i++) {
        pwm.setPWM(i, 0, random_closed);
    }
    Serial.print("servos at position ");
    Serial.println(random_closed);
    // wait for the servos to get there
    delay(800);
}

// function which makes the tongue stick out and come back in again
void tongue_out_in(bool sync, bool relay_num[])
{ 
    // syncronised tonuges out
    if (sync) {
        // wait for jaws to open
        delay(1000);
        // close the relays - tongue out
        for (int i = 9; i <= 12; i++) {
            digitalWrite(i, LOW);
        }
        // keep the tongue out for this many milliseconds
        delay(1000);
        for (int i = 9; i <= 12; i++) {
            // open the relay - tongue in
            digitalWrite(i, HIGH);
        }
    } else {
        // randomised tongues out
        // wait for jaws to open
        delay(1000);

        for (int i = 0; i < 4; i++) {
            Serial.print(relay_num[i]);
            // close the relays - tongue out
            if (relay_num[i] == true) {
                digitalWrite((i + 9), LOW);
            }
        }
        // keep the tongue out for this many milliseconds
        delay(1000);
    }
    for (int i = 9; i <= 12; i++) {
        // open the relay - all tongues in
        digitalWrite(i, HIGH);
    }
}

//  It is only necessary to know the position of the lower jaws!
uint16_t mouthTarget[4] = { SERVOFULLCLOSE };
//  Is it important to know the actual mouth target posittion at any point?
uint8_t mouthState[4] = { 0 };
uint32_t mouthDelay[4] = { 5000 };
// 0 is closed!
// 1 is opening
// 2 is open!
// 3 is closing"
// 4 is sticking a tongue out!
// 5 is done with sticking out its tongue!
uint32_t mouthStartTick[4];

uint32_t mouthTick = 0;
uint32_t mouthInterval = 1000/10;

//  For now I am going to assume random, then we can worry about synchronised movements later!
void mouthAnimate()
{
    if (currentTick - mouthTick >= mouthInterval)
    {
        for ( uint8_t n = 0; n < 4; n++)
        {
            switch (mouthState[n])
            {
                case 0:
                //  CLOSED: Holding the mouth closed!
                    if ( currentTick - mouthStartTick[n] > mouthDelay[n])
                    {
                        //  The head has been paused long enough
                        //  Start opening!
                        mouthState[n] = 1;
                        mouthDelay[n] = 2000;   //  Adjust to make sure this is low, but always works! This is a mechanical constraint!
                        mouthStartTick[n] = currentTick;
                        mouthTarget[n] = random(SERVOFULLOPEN, SERVOFULLCLOSE); //  This is keeping track of the bottom mouth
                        pwm.setPWM((2*n), 0, random(SERVOFULLOPEN, SERVOFULLCLOSE););
                        pwm.setPWM((2*n)+1, 0, mouthTarget[n]);
                    }
                    //else 
                    //{
                        // Wait for mouth to be closed long enough to start a new sequence!
                    //}
                    break;
                case 1:
                //  OPENING! Waiting for the mouth to open!
                    if ( currentTick - mouthStartTick[n] > mouthDelay[n])
                    {
                        //  Mouth is open now!

                        //  Insert code to check for sticking out tongue here
                        //if ( mouthTarget[n] < ((SERVOFULLCLOSE + SERVOFULLOPEN) / 2) )
                        //{
                        //    mouthState[n] = 4; // Stick out tongue!
                        //    mouthDelay = 1000; // Keep tongue out for 1 second!
                        //    mouthStartTick[n] = currentTick;
                        //    digitalWrite(9+n, LOW); // Actually stick out the tongue!
                        //} else {
                        mouthState[n] = 2;
                        mouthDelay[n] = random(500, 1000);  // Duration to wait with mouth open!
                        mouthStartTick[n] = currentTick;
                        //  No position updates!
                        //}

                    }
                    // else { wait ... }
                    break;
                case 2:
                //  OPEN! Waiting for a period!
                    if ( currentTick - mouthStartTick[n] > mouthDelay[n])
                    {
                        //  Mouth is closing now!
                        mouthState[n] = 3;
                        mouthDelay[n] = 2000;  // Duration to wait with mouth open!
                        mouthStartTick[n] = currentTick;
                        mouthTarget[n] = random(SERVOFULLCLOSE-10, SERVOFULLCLOSE);
                        pwm.setPWM((2*n), 0, random(SERVOFULLCLOSE-10, SERVOFULLCLOSE););
                        pwm.setPWM((2*n)+1, 0, mouthTarget[n]);

                    }
                    // else { wait ... }
                    break;
                case 3:
                //  CLOSING!
                    if ( currentTick - mouthStartTick[n] > mouthDelay[n])
                    {
                        //  Mouth is mouth is closed now!
                        mouthState[n] = 0;
                        mouthDelay[n] = random(1000, 3000);  // Duration to wait with mouth open!
                        mouthStartTick[n] = currentTick;
                        // No position updates!
                    }
                    break;
                case 4:
                //  STICKING TONGUE OUT!
                    if ( currentTick - mouthStartTick[n] > mouthDelay[n])
                    {
                        //  Pull back in tongue!
                        digitalWrite(9+n, HIGH);

                        // Switch state to closing the mouth
                        mouthState[n] = 2;
                        mouthDelay[n] = 1000;  // Duration to wait for tongue to go back in and for the mouth to close
                        mouthStartTick[n] = currentTick;
                        // No position updates!
                    }
                    break;
                case default:
                    break;
            }
        }
        mouthTick = currentTick;
    }
}

//function for generating random brightness values for leds
/*
int getRandomPwmValue() {
    static const int candidate[] = {0, 1000, 3000, 4096};
    static const int count = sizeof(candidate) / sizeof(*candidate);
    return candidate[random(0, count)];
}
*/

//A function for the leds (snake eye wave)
/*
void eyesofled()
{
    // Drive each PWM in a 'wave'
    for (uint16_t i = 0; i < 4096; i += 8)
    {
        for (uint8_t pwmnum = 0; pwmnum < 16; pwmnum++)
        {
            eyePWM.setPWM(pwmnum, 0, (i + (4096 / 16)*pwmnum) % 4096 );
        }
    }
}
*/

//A function to randomly light pairs of leds (this could be better)
/*
void randomeyes()
{
    for (int i = 0; i < 7; i++)
    {
        if ( (i % 2) == 0) { //This checks if even and if even then add if odd then minus
            eyePWM.setPWM(i, 0, getRandomPwmValue());
            eyePWM.setPWM(i + 1, 0, getRandomPwmValue());
            delay(1000);
            eyePWM.setPWM(i, 0, 0);
            eyePWM.setPWM(i + 1, 0, 0);
        } else {
            eyePWM.setPWM(i, 0, getRandomPwmValue());
            eyePWM.setPWM(i - 1, 0, getRandomPwmValue());
            delay(1000);
            eyePWM.setPWM(i, 0, 0);
            eyePWM.setPWM(i - 1, 0, 0);
        }
    }
}
*/

//  SNAKE MODES!
//  0 This will eventually be standby mode. This is the mode when the medusa is 
uint8_t mode = 0;

//  Non-blocking eye LED code
//  Only 4 values as eyes are always paired!
//  Leds are given a PWM value of between 0 and 4096

// Sync led maximum value to flamethrowers?
uint32_t eyeStartTick[4] = { 0, 0, 0, 0 };
uint16_t eyeValue[4] = { 0 };
uint16_t eyeDuration[4] = { 1, 1, 1, 1 };

uint32_t eyeTick = 0;
uint32_t eyeInterval = 1000/30;   // 30 fps

void randomEyes()
{
    for (uint8_t n = 0; n < 4; n++)
    {
        float progress = float(currentTick - eyeStartTick[n]) / float(eyeDuration[n]);

        if (progress >= 1.0)
        {
            //  Set new value for eyeValue
            eyeValue[n] = random(2048, 4096);
            progress = 0.0;
            eyeStartTick[n] = currentTick;
            eyeDuration[n] = eyeValue[n] / 2;  // Scale duration according to intensity
        }

        uint16_t eyeIntensity = eyeValue[n] * (1.0-progress);

        eyePWM.setPWM(2*n, 0, eyeIntensity);
        eyePWM.setPWM((2*n)+1, 0, eyeIntensity);
    }
}

void syncedEyes() {
    //  Use eyeValue[0] for calculations for all eyes!

    float progress = float(currentTick - eyeStartTick[0]) / float(eyeDuration[0]);

    if (progress >= 1.0)
    {
        //  Set new value for eyeValue
        eyeValue[0] = random(2048, 4096);
        progress = 0.0;
        eyeStartTick[0] = currentTick;
        eyeDuration[0] = eyeValue[0] / 2;  //  Scale duration according to intensity
    }

    uint16_t eyeIntensity = eyeValue[0] * (1.0-progress);

    for (uint8_t n = 0; n < 4; n++)
    {
        eyePWM.setPWM(2*n, 0, eyeIntensity);
        eyePWM.setPWM((2*n)+1, 0, eyeIntensity);
    }
}

void updateEyes()
{
    if (currentTick - eyeTick >= eyeInterval)
    {   
        // Deal with switching between random and synced eyes here!
        randomEyes();
        // syncedEyes();

        eyeTick = currentTick;
    }
}

//  Flamethrower settings
uint8_t flamethrowers = 4;

uint32_t flameOnTime = 150;

const bool LowOn = true;

uint32_t lastFlamethrowerTick = 0;
uint32_t flamethrowerInterval = 1000/60;    // 60fps

void updateFlamethrowers()
{
    if (currentTick - lastFlamethrowerTick >= flamethrowerInterval)
    {
        // Currently this will fire all the flames on start up!
        for (uint8_t n = 0; n < flamethrowers; n++)
        {
            if (currentTick - flameStartTick[n] >= flameOnTime)
            {
                //  LED OFF!
                if (LowOn)
                {
                    digitalWrite(9+n, HIGH);
                } 
                else
                {
                    digitalWrite(9+n, LOW);
                }
            } else {
                //  LED ON!
                if (LowOn)
                {
                    digitalWrite(9+n, LOW);
                }
                else
                {
                    digitalWrite(9+n, HIGH);
                }
            }
        }
        lastFlamethrowerTick = currentTick;
    }
}

void loop()
{
    currentTick = millis();
    updateFlamethrowers();
    updateEyes();
    //mouthAnimate();
    modbus.poll();

    /*
    // choose the number of times the random function is called
    uint16_t how_many_random = 6;
    // choose the number of times the synchronous function is called
    uint16_t how_many_sync = 4;

    // call random function
    Serial.println("random");
    for (int i = 0; i < how_many_random; i++) {
        random_mouths();
        eyesofled();
        flamerelays();
    }
    Serial.println("synchronous");
    // call synchronous function
    for (int i = 0; i < how_many_sync; i++) {
        synchronous_mouths();
        randomeyes();
        flamerelays();
    }
    Serial.println();
    */
}
