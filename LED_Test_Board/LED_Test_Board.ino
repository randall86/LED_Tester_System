#include <Adafruit_PWMServoDriver.h>
#include <Countimer.h>
#include <DTIOI2CtoParallelConverter.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

const char * app_ver = "v1.4";

const byte PWM_OUTPUT_EN = 4; //default is low
const byte PWM_OUTPUT_PIN_R = 0;
const byte PWM_OUTPUT_PIN_G = 1;
const byte PWM_OUTPUT_PIN_B = 2;
const byte PWM_OUTPUT_PIN_FW = 3;
const byte PWM_OUTPUT_PIN_CW = 4;

const byte PWM_EXTCLK_PRESCALE = 5;

const byte EXP_INTR_PIN = 2;
const byte SW_INTR_PIN = 3;

const byte EXP_ROTARY_SW_1 = PIN0_0;
const byte EXP_ROTARY_SW_2 = PIN0_1;
const byte EXP_ROTARY_SW_3 = PIN0_2;
const byte EXP_ROTARY_SW_4 = PIN0_3;
const byte EXP_ROTARY_SW_5 = PIN0_4;
const byte EXP_ROTARY_SW_6 = PIN0_5;
const byte EXP_ROTARY_SW_7 = PIN0_6;
const byte EXP_ROTARY_SW_8 = PIN0_7;
const byte EXP_ROTARY_SW_9 = PIN1_0;
const byte EXP_ROTARY_SW_10 = PIN1_1;
const byte EXP_ROTARY_SW_11 = PIN1_2;
const byte EXP_ROTARY_SW_12 = PIN1_3;

const byte DBG_LED3_RED = PIN1_4;
const byte DBG_LED4_GREEN = PIN1_5;
const byte PWM_LED_ON = PIN1_6;

const uint32_t REFRESH_RATE_MSEC = 1000; //update lcd timer count every 1 sec
const uint32_t DEBOUNCE_MSEC = 300; //stable time before registering state change
const uint32_t CHECK_MSEC = 100; //read switch every 100ms when detected state change

char *LEDTestMsg[] =
{
  "1 Red Primary",
  "2 Green Primary",
  "3 Blue Primary",
  "4 2200K Primary",
  "5 6500K Primary",
  "6 Yellow",
  "7 Cyan",
  "8 Magenta",
  "9 4000K"
};

//types of LED test configurations
enum _LED_config
{
  RED_PRI,
  GREEN_PRI,
  BLUE_PRI,
  TEMP_2200K_PRI,
  TEMP_6500K_PRI,
  YELLOW,
  CYAN,
  MAGENTA,
  TEMP_4000K
};

typedef struct _LED_dutycycle_config_t
{
  uint16_t red_dutycycle;
  uint16_t green_dutycycle;
  uint16_t blue_dutycycle;
  uint16_t fw_dutycycle;
  uint16_t cw_dutycycle;
}LED_dutycycle_config_t;

//the calculation for the PCA9685 LED controller PWM output register
//duty cycle in % * 4096 - 1 (i.e. 85.7% * 4096 - 1 = ~3509)
//the counter starts at 0 and ends at 4095, so minus 1 is required
//where 4095 will be fully on LED and 0 will be fully off LED
LED_dutycycle_config_t LED_cfg_table[] = 
{ 
  {0x0FFF, 0x0000, 0x0000, 0x0000, 0x0000}, //RED_PRI
  {0x0000, 0x0FFF, 0x0000, 0x0000, 0x0000}, //GREEN_PRI
  {0x0000, 0x0000, 0x0FFF, 0x0000, 0x0000}, //BLUE_PRI
  {0x0000, 0x0000, 0x0000, 0x0FFF, 0x0000}, //TEMP_2200K_PRI
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0FFF}, //TEMP_6500K_PRI
  {0x0FFF, 0x0F09, 0x0000, 0x0028, 0x0000}, //YELLOW
  {0x0000, 0x0FFF, 0x022B, 0x0000, 0x006C}, //CYAN
  {0x0FFF, 0x0000, 0x048E, 0x0000, 0x0018}, //MAGENTA
  {0x0000, 0x0E18, 0x0000, 0x0F36, 0x0A72}  //TEMP_4000K
};

//timer for counting the test duration
Countimer countUpTimer;

//timer for start/stop switch debouncing
Countimer debounceTimer;

//uses the default address 0x6A
Adafruit_PWMServoDriver pwmLEDDrv = Adafruit_PWMServoDriver(0x6A);

// The LCD constructor - I2C address 0x38
LiquidCrystal_I2C lcd(0x38, 4, 5, 6, 0, 1, 2, 3, 7, POSITIVE);

//PCA9539 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter ioExpandr(0x77); 

int g_test_selection = RED_PRI; //default is RED_PRI
int g_display_selection = RED_PRI;
volatile int g_sw_intr_state = 0;
volatile int g_exp_intr_state = 0;
byte g_debouncedSwState = 0; //off

void swInterruptHandler()
{
  g_sw_intr_state = 1;
}

void expInterruptHandler()
{
  g_exp_intr_state = 1;
}

int getSWSelection()
{
  int ret = -1;
  byte input_sw_9 = EXP_ROTARY_SW_9;
  byte input_sw_1_8[8] = { EXP_ROTARY_SW_1, EXP_ROTARY_SW_2, EXP_ROTARY_SW_3, EXP_ROTARY_SW_4,\
                          EXP_ROTARY_SW_5, EXP_ROTARY_SW_6, EXP_ROTARY_SW_7, EXP_ROTARY_SW_8 };

  for(int index = 0; index < sizeof(input_sw_1_8); index++)
  {
    if(ioExpandr.digitalRead0(input_sw_1_8[index]))
    {
      if(!input_sw_1_8[index])
      {
        ret = index;
        break;
      }
    } 
  }

  if(ioExpandr.digitalRead1(input_sw_9))
  {
    if(!input_sw_9)
    {
      ret = TEMP_4000K;
    }
  }

  //no valid input found
  if(ret == -1)
  {
    ret = g_test_selection; // return the current test selection
  }
  
  return ret;
}

void setPWMLEDsOff()
{
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_R, 0x0000);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_G, 0x0000);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_B, 0x0000);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_FW, 0x0000);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_CW, 0x0000);
  
  //when output enable is set to HIGH, all the LED outputs are programmed to the value 
  //that is defined by OUTNE[1:0] where OUTNE[1:0] of 00 the LED outputs will be 0 
  digitalWrite(PWM_OUTPUT_EN, HIGH);

  ioExpandr.digitalWrite1(PWM_LED_ON, LOW); //off the LEDs
}

void setPWMOutput(LED_dutycycle_config_t * cfg)
{
  ioExpandr.digitalWrite1(PWM_LED_ON, HIGH); //on the LEDs connected to PWM
  
  //when output enable is set to LOW, all the PWM outputs are enabled to follow the ON_OFF registers
  digitalWrite(PWM_OUTPUT_EN, LOW);
  
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_R, cfg->red_dutycycle);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_G, cfg->green_dutycycle);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_B, cfg->blue_dutycycle);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_FW, cfg->fw_dutycycle);
  pwmLEDDrv.setPin(PWM_OUTPUT_PIN_CW, cfg->cw_dutycycle);
}

void completeTimerCount()
{
  countUpTimer.restart(); //restart the timer once end
}

void displayTimerCount()
{
  lcd.setCursor(0,1);
  lcd.print("Timer");
  lcd.setCursor(6,1);
  lcd.print(countUpTimer.getCurrentTime());
}

void displayLEDTestMsg()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(LEDTestMsg[g_test_selection]);
}

void displayStartMsg()
{
  lcd.setCursor(0,0);
  lcd.print("LED Tester");
  lcd.setCursor(11,0);
  lcd.print(app_ver);
  lcd.setCursor(0,1);
  lcd.print("Select test:1-9");
}

//returns true if state changed
bool debounceSwitch(byte *state)
{
  static uint8_t count = DEBOUNCE_MSEC/CHECK_MSEC;
  bool state_changed = false;

  //read the switch from the HW
  byte raw_state = digitalRead(SW_INTR_PIN);
  *state = g_debouncedSwState;

  if (raw_state == g_debouncedSwState)
  {
    //set the timer which allows a change from current state.
    count = DEBOUNCE_MSEC/CHECK_MSEC;
  }
  else
  {
    //state has changed - wait for new state to become stable.
    if (--count == 0)
    {
        // Timer expired - accept the change.
        g_debouncedSwState = raw_state;
        state_changed = true;
        *state = g_debouncedSwState;
            
        // And reset the timer.
        count = DEBOUNCE_MSEC/CHECK_MSEC;
    }
  }

  return state_changed;
}

void debounceSwRoutine()
{
  byte switch_state = 0;
    
  //if switch state changed, update the state
  if(debounceSwitch(&switch_state))
  {
    if(switch_state)
    {
      debounceTimer.stop();

      //begin selected test sequence
      setPWMOutput(&LED_cfg_table[g_test_selection]);

      displayLEDTestMsg(); //update the display
      countUpTimer.restart();
    }
    else
    {
      debounceTimer.stop();
      
      countUpTimer.pause();
      
      //off all PWM outputs
      setPWMLEDsOff();
    }
  }
}

void setup()
{
  Wire.begin(); //need to start the Wire for I2C devices to function

  //initialize the timer to count up to max 999 hours 59 mins and 59 secs
  countUpTimer.setCounter(COUNTIMER_MAX_HOURS, COUNTIMER_MAX_MINUTES_SECONDS, COUNTIMER_MAX_MINUTES_SECONDS, countUpTimer.COUNT_UP, completeTimerCount);
  countUpTimer.setInterval(displayTimerCount, REFRESH_RATE_MSEC);

  debounceTimer.setInterval(debounceSwRoutine, CHECK_MSEC);

  pinMode(PWM_OUTPUT_EN, OUTPUT);
  pinMode(SW_INTR_PIN, INPUT);
  pinMode(EXP_INTR_PIN, INPUT);

  //initialize the LED relay pin, for controlling LEDs connected to PWM
  ioExpandr.pinMode1(PWM_LED_ON, LOW);
  ioExpandr.digitalWrite1(PWM_LED_ON, LOW);

  pwmLEDDrv.begin(PWM_EXTCLK_PRESCALE); //set to use the external oscillator
  
  //sets all the PWM output signal to off
  setPWMLEDsOff();

  lcd.begin(16,2); // sixteen characters across - 2 lines
  lcd.backlight();

  //intialize the rotary switch input pins
  ioExpandr.portMode0(ALLINPUT);
  ioExpandr.pinMode1(EXP_ROTARY_SW_9, HIGH);
  ioExpandr.pinMode1(EXP_ROTARY_SW_10, HIGH); //spare
  ioExpandr.pinMode1(EXP_ROTARY_SW_11, HIGH); //spare
  ioExpandr.pinMode1(EXP_ROTARY_SW_12, HIGH); //spare

  ioExpandr.pinMode1(DBG_LED3_RED, LOW);
  ioExpandr.digitalWrite1(DBG_LED3_RED, HIGH);

  ioExpandr.pinMode1(DBG_LED4_GREEN, LOW);
  ioExpandr.digitalWrite1(DBG_LED4_GREEN, HIGH);

  g_test_selection = getSWSelection();
  g_display_selection = g_test_selection;

  g_sw_intr_state = digitalRead(SW_INTR_PIN);

  attachInterrupt(digitalPinToInterrupt(SW_INTR_PIN), swInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EXP_INTR_PIN), expInterruptHandler, CHANGE);

  //display app title and version
  displayStartMsg();
}

void loop()
{
  debounceTimer.run();
  countUpTimer.run();
  
  //check test selection from rotary switch
  if(g_exp_intr_state)
  {
    g_exp_intr_state = 0;
    g_test_selection = getSWSelection();
  }

  //handle start_stop switch interrupt
  if(g_sw_intr_state)
  {
    g_sw_intr_state = 0;
    debounceTimer.start();
  }

  //update the display if test selection is changed
  if(g_display_selection != g_test_selection)
  {
    g_display_selection = g_test_selection;

    displayLEDTestMsg(); //update the display
    
    if(g_debouncedSwState)
    {
      //begin selected test sequence
      setPWMOutput(&LED_cfg_table[g_test_selection]);

      countUpTimer.restart();
    }
  }
}

