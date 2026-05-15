/*
* KobukiNavigationStatechart.c
*
*/

/*
* KobukiNavigationStatechart.c
*
*/


//main changes from G-
// made turn size either -1 or 1 for navigation , it does a flat 90 degree turn whenever it reverses - this made the corner mechanism a bit easier to do and also prevents wall hugging too much
//commented out some transitions as they werent relevant for the navigation stage , but thye may be relevant for the hill climb so feel free to uncomment
// removed the 2nd guard condition for the pivot left and right stages as they werent really relevant now that the turn size is 90 degrees, also in that same line added a couple more absolute values cos why not
// added a cornerdetected flag: it activates if in the travel state it bumps into another opbect (a corner) before it gets to the correct orinertation state. when this happens, it reverses the turn size signs,
// so now the object pivots the other way, away from the corner. Once the robot manages to get in the test new direction state and travels a reasonably large distance (indivating the obstsble is passed),
//only then the corner detected flag is turned off and it goes into its normal drive and obstable detected stage where turn size signs are back to normal

#include "kobukiNavigationStatechart.h"
#include "myrio/MyRio.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

extern NiFpga_Session myrio_session; // to control LEDs on the myRIO

// Program States
typedef enum {
	INITIAL = 0, // Initial state
	PAUSE_WAIT_BUTTON_RELEASE, // Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,  // Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE, // Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE, // Drive straight
	REVERSE, // Drive Backwards
	PIVOT_LEFT, // Turn Left
	PIVOT_RIGHT, // Turn Right
	TRAVEL,
	CORRECT_ORIENTATION, // Correct to Ground Orientation
	TEST_NEW_DIRECTION,
	OBJECT_DETECTED,
	CLIFF_DETECTED,
	WHEEL_HAZARD_DETECTED,
	ASCEND,
	DESCEND,
	REACH_PLATEAU,
	COMPLETE
} robotState_t;

#define DEG_PER_RAD (180.0 / M_PI) // degrees per radian
#define RAD_PER_DEG (M_PI / 180.0) // radians per degree

void KobukiNavigationStatechart(
	const short   maxWheelSpeed,
	const int  netDistance,
	const int  netAngle,
	const KobukiSensors_t sensors,
	const accelerometer_t accelAxes, // in gs
	short* const  pRightWheelSpeed,
	short* const pLeftWheelSpeed,
	const bool isSimulator

) {
	// to control LEDs
	NiFpga_Status status;
	status = MyRio_Open();

	// local state
	static robotState_t  state = INITIAL; // current program state
	static robotState_t unpausedState = DRIVE; // state history for pause region
	static int distanceAtManeuverStart = 0; // distance robot had travelled when a maneuver begins, in mm
	static int angleAtManeuverStart = 0; // angle through which the robot had turned when a maneuver begins, in deg
	static short turnSize = 0;
	static bool cornerDetected = false;
	static int thresholdCliff = 700;
	bool leftBumper = sensors.bumps_wheelDrops.bumpLeft;
	bool rightBumper = sensors.bumps_wheelDrops.bumpRight;
	bool centerBumper = sensors.bumps_wheelDrops.bumpCenter;
	bool leftDrop = sensors.bumps_wheelDrops.wheeldropLeft;
	bool rightDrop = sensors.bumps_wheelDrops.wheeldropRight;
	bool cliffLeft = (sensors.cliffLeft < thresholdCliff);
	bool cliffCenter = (sensors.cliffCenter < thresholdCliff);
	bool cliffRight = (sensors.cliffRight < thresholdCliff);
	static int groundOrientation = 0;
	static bool groundOrientationSet = false;

	// hill-exclusive
	bool hasTilted = false; // to indicate that it is on a plateau
	bool isAscending = accelAxes.x < -0.1; // x is facing up and gravity component is negative (noseUp)
	bool isDescending = accelAxes.x > 0.1; // x is facing down and gravity component is positive (noseDown)

	// outputs
	short  leftWheelSpeed = 0; // speed of the left wheel, in mm/s
	short rightWheelSpeed = 0; // speed of the right wheel, in mm/s
	int reverseDistance = 0;
	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************



	if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0
		) {
		switch (state) {
		case INITIAL:
			// set state data that may change between simulation and real-world
			if (isSimulator) {
			}
			else {
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state

			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!sensors.buttons.B0) {
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!sensors.buttons.B0) {
				if (!groundOrientationSet)// sets ground orientation at very start as starting net angle value , and then never change it as flag is always 1 so cant enter this loop
				{
					groundOrientation = netAngle;
					groundOrientationSet = true;
				}
				state = unpausedState;
			}

			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (sensors.buttons.B0) {
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}

			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// state transition - play region     *
	//*************************************
	else if (state == DRIVE && (leftBumper || rightBumper || centerBumper)) {
		cornerDetected = false; // failsafe to make sure corners never detected in drive state , only travel

		if (leftBumper && !rightBumper)
		{
			turnSize = -1; //+ centerBumper + leftBumper);
		}
		else
		{
			turnSize = 1;// + centerBumper + rightBumper);
		}
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = OBJECT_DETECTED;
	}
	else if (state == DRIVE && (leftDrop || rightDrop)) {
		state = WHEEL_HAZARD_DETECTED;
	}
	else if (state == DRIVE && isAscending) {
		state = ASCEND;
	}
	//*******************************************************
	// state transition - OBSTACLE DETECTION REGION	*
	//*******************************************************
	else if (state == OBJECT_DETECTED) { // what guard to put here instead of false
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = REVERSE;
	}

	else if (state == WHEEL_HAZARD_DETECTED && false) { // what guard to put here instead of false
		state = REVERSE;
	}
	else if (state == CLIFF_DETECTED) {
		distanceAtManeuverStart = netDistance;
		state = REVERSE;
	}

	//*******************************************************
	// state transition - OBSTACLE AVOIDANCE REGION			*
	//*******************************************************

	// REVERSE transitions
	else if (state == REVERSE && (abs(netDistance - distanceAtManeuverStart) > 100)) {//PIVOT LEFT AND PIVOT RIGHT TRANSITIONS
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		if (isAscending || isDescending || hasTilted) { // hill-specific indicating ascend/descend/plateau
			state = CORRECT_ORIENTATION;
		}
		else if (turnSize == -1)
		{
			state = PIVOT_RIGHT;
		}
		else {
			state = PIVOT_LEFT;
		}

	}
	//these are commented out for navigation as it doesnt apply
	//else if (state == REVERSE && (abs(netDistance - distanceAtManeuverStart) > 100) && false) {// what guard to put here instead of false
	//	state = TRAVEL;
//	}
	//else if (state == REVERSE && (abs(netDistance - distanceAtManeuverStart) > 100) && false) {// what guard to put here instead of false
//		state = CORRECT_ORIENTATION;
//	}
	//changed turnsize to 90 degrees here , USED ABS FOR TURNSIZE *90
	//atm pivot left and right transitions only consider ground navigation , have not consiodered hill climb yet . //removed the 2nd guard for both piv left and right
	else if (state == PIVOT_LEFT && ((abs(netAngle - angleAtManeuverStart) >= abs(turnSize * 90)))) {// || (abs(netAngle - groundOrientation) > 89))) {//needs to be adjusted to include grad
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TRAVEL;
	}
	else if (state == PIVOT_RIGHT && ((abs(netAngle - angleAtManeuverStart) >= abs(-turnSize * 90)))) {// || (abs(netAngle - groundOrientation) > 89))) {//needs to be adjusted to include grad
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TRAVEL;
	}






	//TRAVEL TRANSITIONS
	else if (state == TRAVEL && (abs(netDistance - distanceAtManeuverStart) > 250)) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = CORRECT_ORIENTATION;
	}

	else if (state == TRAVEL && (leftBumper || rightBumper || centerBumper) && (cornerDetected == false)) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		cornerDetected = true;
		turnSize = -turnSize; //switch the direction of the last turnsize, so that now the robot moves away from the corner
		state = OBJECT_DETECTED;
	}

	else if (state == TRAVEL && (leftBumper || rightBumper || centerBumper) && (cornerDetected == true)) {// here if it bumps again int a wall whill travelling away from corner , turn size does not change
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = OBJECT_DETECTED;
	}

	//CORRECT_ORIENTATION TRANSITIONS

	else if (state == CORRECT_ORIENTATION && (abs(netAngle - groundOrientation) < 2)) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TEST_NEW_DIRECTION;
	}

	//TEST TRANSITIONS

		//once test direction has travelled far enohg, assume obstabvle passed/corner passed therefore default to drive and thus cornerflag is off
	else if (state == TEST_NEW_DIRECTION && (abs(netDistance - distanceAtManeuverStart) > 400)) {//i feel like this is more approriate guard for test than below, cos angle is already ok from correct orientation
		cornerDetected = false;
		if (isAscending) {
			state = ASCEND;
		}
		else if (isDescending) {
			state = DESCEND;
		}
		else if (hasTilted) {
			state = REACH_PLATEAU;
		}
		else {
			state = DRIVE;
		}

	}
	//again commented out for naviagation as not relevant
//	else if (state == TEST_NEW_DIRECTION && (abs(netAngle - groundOrientation) < 2) && false) { // what guard to put here instead of false
//		state = DRIVE;
//	}
	else if (state == TEST_NEW_DIRECTION && (leftBumper || rightBumper || centerBumper)) { // what guard to put here?
		state = OBJECT_DETECTED;
	}
	else if (state == TEST_NEW_DIRECTION && (leftDrop || rightDrop)) { // what guard to put here?
		state = WHEEL_HAZARD_DETECTED;
	}
	else if (state == TEST_NEW_DIRECTION && (abs(netAngle - groundOrientation) < 2) && false) { // what guard to put here instead of false
		state = ASCEND;
	}


	//*******************************************************
	// state transition - HILL REGION			*
	//*******************************************************
	else if (state == ASCEND && !isAscending) {
		state = REACH_PLATEAU;
	}
	else if (state == ASCEND && (cliffLeft || cliffCenter || cliffRight)) {
		state = CLIFF_DETECTED;
	}
	else if (state == REACH_PLATEAU && isDescending) {
		state = DESCEND;
	}
	else if (state == REACH_PLATEAU && (cliffLeft || cliffCenter || cliffRight)) {
		hasTilted = true;
		state = CLIFF_DETECTED;
	}
	else if (state == DESCEND && (cliffLeft || cliffCenter || cliffRight)) {
		state = CLIFF_DETECTED;
	}

	//*******************************************************
	// state transition - TERMINATE REGION			*
	//*******************************************************

	else if (state == DESCEND && !isDescending) {
		distanceAtManeuverStart = netDistance;
		state = COMPLETE;
	}

	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	switch (state) {
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 100;
		break;
	case REVERSE:
		if (turnSize > 0) {
			leftWheelSpeed = -50;
			rightWheelSpeed = -50; //- turnSize * 5;
		}
		else {
			leftWheelSpeed = -50;// +turnSize * 5;
			rightWheelSpeed = -50;
		}
		break;
	case PIVOT_LEFT:
		leftWheelSpeed = -100;
		rightWheelSpeed = -leftWheelSpeed;
		break;
	case PIVOT_RIGHT:
		leftWheelSpeed = 100;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case CORRECT_ORIENTATION:
		if (turnSize < 0) {
			leftWheelSpeed = -20;
			rightWheelSpeed = -leftWheelSpeed;
		}
		else {
			leftWheelSpeed = 20;
			rightWheelSpeed = -leftWheelSpeed;
		}
		break;
	case TRAVEL:
		leftWheelSpeed = rightWheelSpeed = 50;
		break;
	case TEST_NEW_DIRECTION:
		leftWheelSpeed = rightWheelSpeed = 50;
		break;
	case CLIFF_DETECTED:
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	case WHEEL_HAZARD_DETECTED:
		leftWheelSpeed = rightWheelSpeed = 100;
		//what to put here?
		break;
	case ASCEND:
		// TURN ON LED 1 (ignore red squiggly)
		status = NiFpga_WriteU8(myrio_session, DOLED30, 0x02);

		// Correct the orientation first as a precaution
		if (turnSize < 0) {
			leftWheelSpeed = -20;
			rightWheelSpeed = -leftWheelSpeed;
		}
		else {
			leftWheelSpeed = 20;
			rightWheelSpeed = -leftWheelSpeed;
		}

		if (abs(netAngle - groundOrientation) < 2) {
			leftWheelSpeed = rightWheelSpeed = 100;
		}
		break;
	case DESCEND:
		// TURN ON LED 3 (ignore red squiggly)
		status = NiFpga_WriteU8(myrio_session, DOLED30, 0x08);

		// Correct the orientation first as a precaution
		// If ground orientation reached, go 80% speed.
		if (turnSize < 0) {
			leftWheelSpeed = -20;
			rightWheelSpeed = -leftWheelSpeed;
		}
		else {
			leftWheelSpeed = 20;
			rightWheelSpeed = -leftWheelSpeed;
		}
		// If ground orientation reached, go 80% speed.
		if (abs(netAngle - groundOrientation) < 2) {
			leftWheelSpeed = rightWheelSpeed = 60;
		}
		break;
	case REACH_PLATEAU:
		// TURN ON LED 2 (ignore red squiggly)
		status = NiFpga_WriteU8(myrio_session, DOLED30, 0x04);

		leftWheelSpeed = rightWheelSpeed = 100;
		break;
	case COMPLETE:
		// Full speed to the finish
		leftWheelSpeed = rightWheelSpeed = 100;
		// Stop the wheels when its travelled more than 10 cm beyond the hill
		if (abs(netDistance - distanceAtManeuverStart) > 100) {
			leftWheelSpeed = rightWheelSpeed = 0;
		}
		break;
	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}


	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}