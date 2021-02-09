/* clang-format off */
// https://forum.arduino.cc/index.php?topic=453208.0
// https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf
// https://github.com/ivanseidel/DueTimer

#include <PID_v1.h>
#include <DueTimer.h>

bool loopTimeFlag = false;

struct pressureSensor_t {
	int pin;
	double zero;
	double scale;
};

const int numSensors = 2;
pressureSensor_t pressureSensor[numSensors] = {};

double setpoint, input, output;
double Kp = 1;
double Ki = 0;
double Kd = 0;
PID myController(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//struct controllerParams{
//	double setpoint, input, output;
//	double Kp;
//	double Ki;
//	double Kd;
//};
//PID controller();


void setup() {
  // Open a serial port
  Serial.begin(115200);
  
	// PWM set-up on pins
	// | PC3  | D35 | P1   |
	// | PC5  | D37 | P2   |
	// | PC7  | D39 | P3   |
	// | PC9  | D41 | P4   |
	// | PA19 | D42 | P5   |
	// | PC18 | D45 | MTR  |
	REG_PMC_PCER1 |= PMC_PCER1_PID36;  // Enable PWM

	REG_PIOA_ABSR |= PIO_ABSR_P19;  // Set the port A PWM pins to peripheral type B
	REG_PIOC_ABSR |= PIO_ABSR_P3 | PIO_ABSR_P5 | PIO_ABSR_P7 | PIO_ABSR_P9 | PIO_ABSR_P18;  // Set the port C PWM pins to peripheral type B

	REG_PIOA_PDR |= PIO_ABSR_P19;
	REG_PIOC_PDR |= PIO_ABSR_P3 | PIO_ABSR_P5 | PIO_ABSR_P7 | PIO_ABSR_P9 | PIO_ABSR_P18;  // Set the port C PWM pins to outputs

	REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);  // Set the PWM clock A rate to 84MHz (84MHz/1)

	// Loop for each PWM channel (6 in total)
	for (uint8_t i = 0; i < PWMCH_NUM_NUMBER; i++){
		PWM->PWM_CH_NUM[i].PWM_CMR = PWM_CMR_CPRE_CLKA;  // Enable single slope PWM and set the clock source as CLKA
		PWM->PWM_CH_NUM[i].PWM_CPRD = 2100;  // Set the PWM period register 84MHz/(40kHz)=2100;
	}

	REG_PWM_ENA = PWM_ENA_CHID5 | PWM_ENA_CHID4 | PWM_ENA_CHID3 | PWM_ENA_CHID2 | PWM_ENA_CHID1 | PWM_ENA_CHID0; // Enable all PWM channels

	// Loop for each PWM channel (6 in total)
	for (uint8_t i = 0; i < PWMCH_NUM_NUMBER; i++){
		PWM->PWM_CH_NUM[i].PWM_CDTYUPD = 1050;  // Set the PWM duty cycle to 50% (2100/2=1050) on all channels
	}

	pressureSensor[0]  = {A0, 406, 1600.0/1024.0};
	pressureSensor[1]  = {A1, 406, 1600.0/1024.0};
//	pressureSensor[2]  = {A2, 406, 1600.0/1024.0};
//	pressureSensor[3]  = {A3, 406, 1600.0/1024.0};
//	pressureSensor[4]  = {A4, 406, 1600.0/1024.0};
//	pressureSensor[5]  = {A5, 406, 1600.0/1024.0};
//	pressureSensor[6]  = {A6, 406, 1600.0/1024.0};
//	pressureSensor[7]  = {A7, 406, 1600.0/1024.0};
//	pressureSensor[8]  = {A8, 406, 1600.0/1024.0};
//	pressureSensor[9]  = {A9, 406, 1600.0/1024.0};
//	pressureSensor[10] = {A10, 406, 1600.0/1024.0};
//	pressureSensor[11] = {A11, 406, 1600.0/1024.0};

	myController.SetOutputLimits(0, 2100);
	myController.SetSampleTime(1);

  setpoint = 60; //kPa
//  controllerParams it = {0, 0, 0, 0, 0, 0};
//  PID *controller = new PID(&it.input, &it.output, &it.setpoint, it.Kp, it.Ki, it.Kd, DIRECT);

	// Setup a loop time for the controllers
	DueTimer loopTime = DueTimer(1);
	loopTime.attachInterrupt(loopTimeHandler).setFrequency(100).start();
}

void loop() {
	while(loopTimeFlag){
		// Wait for the loop to expire
	}

	input = analogToKpa(pressureSensor[0], analogRead(pressureSensor[0].pin));
	myController.Compute();
	setPWMDutyCycle(0, output);

	// Trigger the EOL condition
	loopTimeFlag = true;
}

double analogToKpa(pressureSensor_t sensor, int input){
	return((input - sensor.zero)*sensor.scale >= 0 ? (input - sensor.zero)*sensor.scale : 0);
}

void setPWMDutyCycle(int channelNumber, double value){
	PWM->PWM_CH_NUM[channelNumber].PWM_CDTYUPD = value;
}

void loopTimeHandler(){
	if(!loopTimeFlag){
		// Loop too long, probs print something
    Serial.println("Loop time exceeded");
		loopTimeFlag = false;
	}
}

// TODO thinking of making a class for a PID controller that either extends the normal arduino PID class or overwrites it. 
// This class would be of type "controllerPID" and allow the user to set a loop time etc


// TODO can you make an electron app or otherwise that you can interface with the arduino to give you real time feedback with dials and sliders? 