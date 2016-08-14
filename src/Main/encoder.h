// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 3.0v: Not Optimized
// Last mod: 12-11-2012


#ifndef encorder_H
#define encorder_H



/*! @define EncoderL_PIN  Left Motor Encoder PIN number */
#define EncoderL_PIN 3 //int0 --> pin 3
/*! @define EncoderL_INT  Left Motor Encoder Interrupt number */
#define EncoderL_INT 1 //int0 --> INT 1
/*! @define EncoderR_PIN  Right Motor Encoder PIN number */
#define EncoderR_PIN 2 //int1 --> pin 2
/*! @define EncoderL_INT  Right Motor Encoder Interrupt number */
#define EncoderR_INT 0 //int1 --> INT 0


/*!
 * @var         encoderCountLeft
 * @abstract    store count of left encoder.
 * @discussion  stores encorder count by encoderLeftInterruptRoutine()
 */
int encoderCountLeft;

/*!
 * @var         encoderCountRight
 * @abstract    store count of right encoder.
 * @discussion  stores encorder count by encoderRightInterruptRoutine()
 */
int encoderCountRight;


void initializeEncorders();
void encoderLeftInterruptRoutine();
void encoderRightInterruptRoutine();
void encoderLeftCountReset();
void encoderRightCountReset();

void initializeEncorders(){
	
	encoderCountLeft  = 0;
	encoderCountRight = 0;

	pinMode(EncoderL_PIN,INPUT);
	attachInterrupt(EncoderL_INT, encoderLeftInterruptRoutine, CHANGE);
	pinMode(EncoderR_PIN,INPUT);
	attachInterrupt(EncoderR_INT, encoderRightInterruptRoutine, CHANGE);
}


/*!
 * @function    encoderLeftInterruptRoutine
 * @abstract    Count left encoder changes
 * @discussion  This function count encoder changes using external interrupt. Whenever int0 triggers(by change it's status), this will be called. 
 *              The attached encoder has 32 teeths. that means this function will be called 64 times per round. 
 *              Wheel diameter is 9cm. Circumference ~= 28cm. ==> ~68.5 teeths per short path on the grid.
 * @param       none
 * @result      Set encoderCountLeft. returns nothing.
 */
void encoderLeftInterruptRoutine(){
	
	if(encoderCountLeft>10000)
		encoderCountLeft = 0;
	else
		encoderCountLeft++;
	/* test 
	if(encoderCountLeft%4==0){
		Serial.print("L: ");
		Serial.println(encoderCountLeft);
	} 
	*/
}

/*!
 * @function    encoderRightInterruptRoutine
 * @abstract    Count right encoder changes
 * @discussion  This function count encoder changes using external interrupt. Whenever int0 triggers(by change it's status), this will be called. 
 * @result      Set encoderCountRight. returns nothing.
 */
void encoderRightInterruptRoutine(){
	if(encoderCountRight>10000)
		encoderCountRight = 0;
	else
		encoderCountRight++;
	/* test
	if(encoderCountRight%4==0){
		Serial.print("R: ");
		Serial.println(encoderCountRight);
	}
	*/
}

void encoderLeftCountReset(){
	encoderCountLeft  = 0;
}

void encoderRightCountReset(){
	encoderCountRight  = 0;
}

int getLeftEncoderCount(){
	return encoderCountLeft;
}

int getRighttEncoderCount(){
	return encoderCountRight;
}

#endif