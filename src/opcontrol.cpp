#include "main.h"


int exp_curves(int joystickValue){
	return abs(joystickValue)/joystickValue * (1.2*pow(1.0356, abs(joystickValue)) - 1.2 + 0.2*abs(joystickValue));
}

// int exp_curves(int x){
// 	double e=  2.718281;
// 	int t = 9.8;
// 	if(x==0){
// 		return 0;
// 	} else if(x > 0){
// 		return 127/(1+pow(e, (-1/t)*(x-127/2)));
// 	} else {
// 		return -127/(1+pow(e, (-1/t)*(-x-127/2)));
// 	}
// }

bool bowlModeFlag = false;
void controllerControl() {
	int left = master.get_analog(ANALOG_LEFT_Y);
	int right = master.get_analog(ANALOG_RIGHT_Y);
	// rightMotors =  exp_curves(right*(127/100));
	// leftMotors = exp_curves(left*(127/100));
	chassis.tank(left, right);
	// rightMotors = right;
	// leftMotors = left;
}

char intakeControlState = 'N';
// void intakeControl(){
// 	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
// 		if(intakeControlState == 'N' || intakeControlState == 'R'){
// 			intake = 127;
// 			intakeControlState = 'F';
// 		} else { 
// 			intake = 0;
// 			intakeControlState = 'N';
// 		}
// 		// pros::delay(180);
// 	}
// 	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
// 		if(intakeControlState == 'N' || intakeControlState == 'F'){
// 			intake = -127;
// 			intakeControlState = 'R';
// 		} else {
// 			intake = 0;
// 			intakeControlState = 'N';
// 		}
// 		// pros::delay(180);
// 	}
// }
bool kickerControlFlag = false;
void kickerControl(){
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		kickerControlFlag = !kickerControlFlag;
		if(kickerControlFlag){
			kicker=127;
			// pros::delay(20);
	    } else {
			kicker=0;
			// pros::delay(20);
		}
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
		kicker=100;
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
		kicker= 40;
	}
}
void intakeControl(){
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		if(intake1.get_target_velocity()>0){
			intake=0;
		} else {
			intake=127;
		}
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
		if(intake1.get_target_velocity()<0){
			intake=0;
		} else {
			intake=-127;
		}
	}
}

bool flapControlFlag = false;
void flapControl() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
		bowlModeFlag=false;
		flapControlFlag = !flapControlFlag;
		if(flapControlFlag){
			flapLeftPiston.set_value(true);
			flapRightPiston.set_value(true);
			// pros::delay(20);
	    } else {
			flapLeftPiston.set_value(false);
			flapRightPiston.set_value(false);
			// pros::delay(20);
		}
	}
}

// bool laDiablaFlag = false;
// void laDiablaControl() {
// 	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
// 		laDiablaFlag = !laDiablaFlag;
// 		if(laDiablaFlag){
// 			laDiabla.set_value(true);
// 			pros::delay(20);
// 	    } else {
// 			laDiabla.set_value(false);
// 			pros::delay(20);
// 		}
// 	}
// }


void bowlModeControl(){
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
		bowlModeFlag= !bowlModeFlag;
		if(bowlModeFlag){
			flapLeftPiston.set_value(true);
			flapRightPiston.set_value(true);
			intake=127;
			intakeControlState='F';
			flapControlFlag = true;
			pros::delay(20);

		} else {
			flapLeftPiston.set_value(false);
			flapRightPiston.set_value(false);
			intake= 0;
			intakeControlState='N';
			flapControlFlag=false;
			pros::delay(20);
		}
	}
}
// char color = 'W';
// int flickerTime	= 500;
// void screenFlicker(){
// 	if(color=='W'){
		
// 		pros::screen::set_pen(COLOR_WHITE);
// 		pros::screen::fill_rect(5,5,480,200);
// 		pros::delay(flickerTime);
// 		pros::screen::set_pen(COLOR_BLACK);
// 		pros::screen::fill_rect(5,5,480,200);
// 	} else if(color=='R'){
// 		pros::screen::set_pen(COLOR_RED);
// 		pros::screen::fill_rect(5,5,480,200);
// 		pros::delay(flickerTime);
// 		pros::screen::set_pen(COLOR_BLACK);
// 		pros::screen::fill_rect(5,5,480,200);
// 	} else {
// 		pros::screen::set_pen(COLOR_GREEN);
// 		pros::screen::fill_rect(5,5,480,200);
// 		pros::delay(flickerTime);
// 		pros::screen::set_pen(COLOR_BLACK);
// 		pros::screen::fill_rect(5,5,480,200);
// 	}
// }

void screenControl(){
	if(intake1.get_target_velocity()>0){
		// pros::screen::set_eraser(COLOR_GREEN);
		// pros::screen::erase();
		pros::screen::set_pen(COLOR_GREEN);
		pros::screen::fill_rect(5,5,480,200);
	} else if(intake1.get_target_velocity()<0) {
		pros::screen::set_pen(COLOR_RED);
		pros::screen::fill_rect(5,5,480,200);
	} else {
		pros::screen::set_pen(COLOR_WHITE);

		pros::screen::fill_rect(5,5,480,200);
	}
}
bool hangControlFlag = false;
void hangControl() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		hangControlFlag = !hangControlFlag;
		if(hangControlFlag){
			hangUp.set_value(false);
			hangDown.set_value(false);
			// pros::delay(20);
	    } else {
			hangUp.set_value(true);
			hangDown.set_value(true);
			// pros::delay(20);
		}
	}
}

bool vertControlFlag = false;
void vertControl() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
		vertControlFlag = !vertControlFlag;
		if(vertControlFlag){
			vertWing.set_value(true);
			// pros::delay(20);
	    } else {
			vertWing.set_value(false);
			// pros::delay(20);
		}
	}
}

// bool blockerFlag = false;
// void blockerControl() {
// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
// 		blockerFlag = !blockerFlag;
// 		if(blockerFlag){
// 			blocker.set_value(true);
// 			pros::delay(200);
// 	    } else {
// 			blocker.set_value(false);
// 			pros::delay(200);
// 		}
// 	}
// }
// bool cataControlFlag = false;
// void cataControl() {
// 	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
// 		cataControlFlag = !cataControlFlag;
// 		if(cataControlFlag){
// 			cata = -127;
// 			pros::delay(160);
// 		} else {
// 			cata = 0;
// 			pros::delay(160);
// 		}
// 	}
// }



//-9500
// void cataTask(){
// 	cur_pos=rot_sensor.get_position();
// 	// if((((abs(cur_pos))-(abs(cur_pos)/pos)*3100 > error) || cur_pos==0) && !cataControlFlag){
// 	if((abs(abs(cur_pos)-target)) > error && !cataControlFlag){
// 		cata = -127;
// 	} else if(cataControlFlag){
// 		cata = -127;
// 	} else {
// 		cata=0;
// 	}
// }

// bool underBarrierControlFlag = false;
// void underBarrierControl(){
// 	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
// 		underBarrierControlFlag = !underBarrierControlFlag;
// 		if(underBarrierControlFlag){
// 			// target = 0; //up position
// 			laDiabla.set_value(true);
// 			pros::delay(200);
// 		} else {
// 			// target = 1500; // down position
// 			laDiabla.set_value(false);
// 			pros::delay(200);
// 		}
// 	}
// }
// bool laDiablaFlag = false;
// void laDiablaControl(){
// 	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
// 		laDiablaFlag = !laDiablaFlag;
// 		if(laDiablaFlag){
// 			laDiabla.set_value(true);
// 			pros::delay(200);
// 		} else {
// 			laDiabla.set_value(false);
// 			pros::delay(200);
// 		}
// 	}
// }

// bool hangFireControlFlag = false;
// void hangFireControl(){
// 	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
// 		hangFireControlFlag = !hangFireControlFlag;
// 		if(hangFireControlFlag){
// 			hang.set_value(true);
// 			pros::delay(200);
// 		} else {
// 			hang.set_value(false);
// 			cata = -127;
// 			pros::delay(200);
// 		}
// 	}
// }
// int target=0;
// int threshhold=400;
// int cur_pos=0;
// float hangkP = 3;
// float output;

// void moveHangTo(){
// 	int error = target - rot_sensor.get_position();
// 	if(abs(error) > threshhold){
// 		// cur_pos = rot_sensor.get_position();
// 		// error= target - cur_pos;
// 		output = -(abs(error)/error)*127;
// 		hang = output;
// 		pros::lcd::print(6, "Error: %i", error);
// 		pros::delay(10);
// 	} else {
	// hang.brake();
// 	}
// }


// char hangState = 'D';
// void hangControl(){
	// hang.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
//         hangState = 'D';
//     } else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//         hangState = 'B';
//     } else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
//         hangState = 'V';
//     }
//     if(hangState == 'D'){
//         target = 8000;
// 		// pros::Task e(moveHangTo);
//     } else if(hangState == 'B'){
// 		ratchet.set_value(true);
//         target = -4675;
// 		// pros::Task e(moveHangTo);
//     } else if(hangState == 'V'){
//         target = -15500; //ignore its vertical hang
// 		// pros::Task e(moveHangTo);
//     }
// } 


// void hangPistonControl(){
// 	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
// 		hangPistonControlFlag = !hangPistonControlFlag;
// 		if(hangPistonControlFlag){
// 			hang.set_value(true);
// 			target=3800;
// 			pros::delay(200);
// 		} else {
// 			hang.set_value(false);
// 			target = 0;
// 			pros::delay(200);
// 		}
// 	}
// }