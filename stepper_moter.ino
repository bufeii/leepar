/*
   Example of the Timer Input Capture mode combined with one pulse mode

   This example uses:
   - Timer2 channel 1 as capture input PA0,HC-SR04 echo pin
   - Timer 3 to generate a PWM trigger signal for stepper motor PA6

  -PB1 as potential input 电位器输入
  -PB8 as direction
  -PB7 as Left limit
  -PB5 as right limit

  -PA3 ultrasonic HC-SR04 trig pin
*/
#include "MapleFreeRTOS900.h"
#include <Streaming.h>
#include"PID_v1.h"
//---------------------------------------------------------------
#define LED_BUILTIN PC13

//-------------------------------------------------------------

//analog input
int aninput = 0;

//L R limitation
bool r_lim = LOW;
bool l_lim = LOW;

int wave_high_length;

//--------------PID---------------------------------------------------------
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//--------------PID---------------------------------------------------------

//-----------------------------------------------------------------------------
void vToggle_ledTask(void *pvParameters)
{
  do {
    vTaskDelay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial << (11 - aninput) * 100 << "Hz\n";
  }
  while (true);
}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void vStepperMoterTask(void *pvParameters)
{
  do {
    vTaskDelay(5);

    //1k-100Hz
    aninput = map(analogRead(PB1), 0, 4095, 1, 10);

    //update the timer3
    pwmRefresh(aninput);

    r_lim = digitalRead(PB5);
    l_lim = digitalRead(PB7);

    if (r_lim == LOW && l_lim == HIGH) {
      //set the direction High
      digitalWrite(PB8, HIGH);
      Serial << "Left limit reached,DIR:LOW\n";
    }
    if (r_lim == HIGH && l_lim == LOW) {
      //set the direction Low
      digitalWrite(PB8, LOW);
      Serial << "Right limit reached,DIR:HIGH\n";
    }
  }
  while (true);


}

void vUltrasonicTask(void *pvParameters)
{
  do {
    vTaskDelay(40);
    //init direction HIGH
    digitalWrite(PA3, HIGH);
    delay(1);
    digitalWrite(PA3, LOW);


    while (!Timer2.getInputCaptureFlag(TIMER_CH2) ) ;// high pulse end
    wave_high_length = Timer2.getCompare(TIMER_CH2) - 3;
    Input = (double)wave_high_length / 2000 * 340.0;
    myPID.Compute();
    Serial << "PWM pulse width: " << wave_high_length << "us\tdistance:" << Input << "mm" <<"\tPID output:"<<Output<< endl;

    //if ( Timer2.getInputCaptureFlag(TIMER_CH1) ) // period end
    //{
    //  Serial << ", period: " << Timer2.getCompare(TIMER_CH1) <<endl;

    //}
  }
  while (true);
}

void pwmRefresh(int x)
{
  //Timer3.pause();
  //Timer3.setPrescaleFactor(72 * x); // x µs resolution
  Timer3.setCompare(TIMER_CH1, 500 * x);
  Timer3.setOverflow(1000 * x);
  //Timer3.refresh();
  //Timer3.resume(); // let timer 3 run

}




void setup()
{
  Serial.begin(115200);

  //PB1 as potential input 电位器输入
  pinMode(PB1, INPUT_ANALOG);

  //PB8 as direction
  pinMode(PB8, OUTPUT);
  //PB7 as Left limit
  pinMode(PB7, INPUT_PULLDOWN);
  //PB5 as right limit
  pinMode(PB5, INPUT_PULLDOWN);


  //PA3 as HC-SR04 trig
  pinMode(PA3, OUTPUT);

  //LED setup
  pinMode(LED_BUILTIN, OUTPUT);

  //init stepper moter direction HIGH
  digitalWrite(PB3, LOW);


  // setup PA6 (Timer3 channel 1) to generate 1 ms period PWM with 50% DC(Duty Cycle)
  pinMode(PA6, PWM);


  //--------------PID-------------setpoint--------------------------------------------
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //--------------PID---------------------------------------------------------

  /* ---------------add ultrasonic meauser code-----------------------------------------------------------*/
  // setup PA0 (Timer 2 channel 1) as input capture mode
  pinMode(PA0, INPUT);

  // stop the timers before configuring them
  Timer2.pause();

  Timer2.setPrescaleFactor(72); // 1 microsecond resolution

  // setup timer 2 channel 1 capture on rising edge
  Timer2.setInputCaptureMode(TIMER_CH1, TIMER_IC_INPUT_DEFAULT); // use default input TI1
  // setup timer 2 channel 2 capture on falling edge
  Timer2.setInputCaptureMode(TIMER_CH2, TIMER_IC_INPUT_SWITCH); // use switched input TI1
  Timer2.setPolarity(TIMER_CH2, 1); // trigger on falling edge

  // counter setup as slave triggered by TI1 in reset mode
  Timer2.setSlaveFlags( TIMER_SMCR_TS_TI1FP1 | TIMER_SMCR_SMS_RESET );

  Timer2.refresh();
  Timer2.getCompare(TIMER_CH1); // clear capture flag
  Timer2.getCompare(TIMER_CH2); // clear capture flag
  Timer2.resume(); // let timer 2 run

  /*------------------up--add----InputCapture----code----------------------------------------------------------*/


  Timer3.pause();
  Timer3.setPrescaleFactor(18); // 1 µs resolution
  Timer3.setCompare(TIMER_CH1, 500);
  Timer3.setOverflow(1000);
  Timer3.refresh();
  Timer3.resume(); // let timer 3 run


  /*-----------------beneth--add----InputCapture----code----------------------------------------------------------*/
  // discard first reading
  // wait for first period end
  while ( !Timer2.getInputCaptureFlag(TIMER_CH1) );
  Timer2.getCompare(TIMER_CH1); // clear capture flag
  /*------------------up--add----InputCapture----code----------------------------------------------------------*/


  xTaskCreate(vStepperMoterTask,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);

  xTaskCreate(vToggle_ledTask,
              "LED",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);


  xTaskCreate(vUltrasonicTask,
              "ultrasonic",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);

  vTaskStartScheduler();
}


void loop() {
  // Insert background code here
}
