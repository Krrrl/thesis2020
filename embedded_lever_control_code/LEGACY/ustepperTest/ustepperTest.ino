#include <uStepper.h>

#define MAXACCELERATION 1500         //Max acceleration = 1500 Steps/s^2
#define MAXVELOCITY 1100           //Max velocity = 1100 steps/s

uStepper stepper(MAXACCELERATION, MAXVELOCITY);



void take_step(bool dir)
{
    stepper.moveSteps(15, dir, SOFT);
}

void take_n_steps(bool dir, int n)
{
  for(int i = 0; i < n; i++)
  {
    take_step(dir);
  }
}

void encoder_init()
{
  stepper.encoder.setHome();
  stepper.encoder.encoderOffset += 180;
}

void setup() {
  stepper.setup();
  encoder_init();
  Serial.begin(115200);
}



void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Encoder reads angle: ");
  Serial.println(stepper.encoder.getAngle());

  Serial.println();
  Serial.print("The angle moved is however: ");
  Serial.println(stepper.encoder.getAngleMoved());

  take_n_steps(CW, 15);
  delay(1000);
}
