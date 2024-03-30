#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include <cmath>
#include <vector>
ASSET(wingstart_txt);

// void cata_pulldown(){
// 	while((abs(abs(cur_pos)-target)) > error){
// 		cur_pos=rot_sensor.get_position();
// 		cata = -127;
// 		pros::delay(10);
// 	}
// 	cata =0;
// }
// void collisionDetection(){
// 	lemlib::Timer collision_timer(
// 		500
// 	);
// 	while(true){
// 		if(collision_timer.isDone()){
// 			chassis.moveToPose(0, 0, 0, 15000);
// 			break;
// 		} else if(lF.get_target_velocity()){

// 		}
// 		pros::delay(30);
// 	}
// }
lemlib::ControllerSettings g_lead_linear_Controller(17, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            30, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings g_lead_angular_Controller(3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             40, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

std::vector<float> pid(const float error, float prevError, float inte, lemlib::ControllerSettings constants){
	inte += error;
	if (lemlib::sgn(error) != lemlib::sgn((prevError))) inte = 0;
    if (fabs(error) > constants.windupRange) inte = 0;
	const float derivative = error - prevError;
    prevError = error;
    // calculate output
    std::vector<float> output = {error * constants.kP + inte * constants.kI + derivative * constants.kD , prevError, inte};
	return output;
}

// vector<float> angular_pid(const float error, const float prev_error, const float inte, glead_boom_angular_controller constants){
// 	inte += error;
// 	if (sgn(error) != sgn((prevError))) inte = 0;
//     if (fabs(error) > constants.antiWindup) inte = 0;
// 	const float derivative = error - prevError;
//     prevError = error;
//     // calculate output
//     vector<float> output = {error * constants.kP + integral * constants.kI + derivative * constants.kD , prev_error, inte};
// 	return output;
// }
struct glead_boom_params {
        bool forwards = true;
    	float chasePower = 0;
        float dlead = 0.6;
		float glead = 0; //Come back to tune it(0 by default makes it a regular boom movement)
		float exitLineDistance = 1;
		float exitSemicircleRadius = 1;
        float maxSpeed = 127;
        float minSpeed = 0;
        float earlyExitRange = 0;
};

bool semicircle_exit(lemlib::Pose pose, lemlib::Pose target, const float distTarget, glead_boom_params params){
	bool settled = false;
		if(-90 < target.theta && target.theta < 90){
			// if((pose.y > tan(theta)(pose.x - x) + y) && ((distTarget < exitSemicircleRadius) || (pose.x > x - exitLineDistance*abs(cos(t)) && x < pose.x > x - exitLineDistance*abs(cos(t))))) settled=true;
			if((pose.y > tan(target.theta)*(pose.x - target.x) + target.y) && ((distTarget < params.exitSemicircleRadius))) settled=true;
			settled = true;
		} else if (-90 > target.theta && target.theta > 90){
			// if((pose.y < tan(theta)(pose.x - x) + y) && ((distTarget < exitSemicircleRadius) || (pose.x > x - exitLineDistance*abs(cos(t)) && x < pose.x > x - exitLineDistance*abs(cos(t))))) settled=true;
			if((pose.y < tan(target.theta)*(pose.x - target.x) + target.y) && ((distTarget < params.exitSemicircleRadius))) settled=true;
			settled = true;
		} else if(target.theta == 90){
			if((pose.x > target.x) && (abs(pose.y - target.y) < params.exitLineDistance)) settled=true;
			settled = true;
		} else if(target.theta == -90){
			if((pose.x < target.x) && (abs(pose.y - target.y) > params.exitLineDistance)) settled=true;
			settled = true;
		}
	return settled;
}
void g_lead_boom(double x, double y, double theta, int timeout, glead_boom_params params, bool async){
	// chassis.lateralPID.reset();
	// chassis.lateralLargeExit.reset();
    // chassis.lateralSmallExit.reset(); //might need to change these exits or remove them
    // chassis.angularPID.reset();
    // chassis.angularLargeExit.reset();
    // chassis.angularSmallExit.reset();

	lemlib::Pose target(x, y, M_PI_2 - lemlib::degToRad(theta));
	if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI);
	if (params.chasePower == 0) params.chasePower = drivetrain.chasePower; // drivetrain match name in botinit

	lemlib::Pose lastPose = chassis.getPose();
	int distTravelled=0;
	lemlib::Timer timer(timeout);

	bool close = false;
	bool lateralSettled = false;
    bool prevSameSide = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();

	// bool settled = false;
	float distTarget = chassis.getPose(true, true).distance(target);
	float prev_error = 0;
	float inte = 0;
	lemlib::Pose initialCarrot = target - lemlib::Pose(cos(target.theta), sin(target.theta))*params.dlead*distTarget;
	lemlib::Pose pose = chassis.getPose(true, true);
	while(!timer.isDone() && (!semicircle_exit(pose, target, distTarget, params))){
		const lemlib::Pose pose = chassis.getPose(true, true);
		//semicircle exit
		// if(-90 < theta && theta < 90){
		// 	// if((pose.y > tan(theta)(pose.x - x) + y) && ((distTarget < exitSemicircleRadius) || (pose.x > x - exitLineDistance*abs(cos(t)) && x < pose.x > x - exitLineDistance*abs(cos(t))))) settled=true;
		// 	if((pose.y > tan(theta)(pose.x - x) + y) && ((distTarget < exitSemicircleRadius))) settled=true;
		// } else if (-90 > theta && theta > 90){
		// 	// if((pose.y < tan(theta)(pose.x - x) + y) && ((distTarget < exitSemicircleRadius) || (pose.x > x - exitLineDistance*abs(cos(t)) && x < pose.x > x - exitLineDistance*abs(cos(t))))) settled=true;
		// 	if((pose.y < tan(theta)(pose.x - x) + y) && ((distTarget < exitSemicircleRadius))) settled=true;
		// } else if(theta == 90){
		// 	if((pose.x > x) && (abs(pose.y - y) < exitLineDistance)) settled=true;
		// } else if(theta=-90){
		// 	if((pose.x < x) && (abs(pose.y - y) > exitLineDistance)) settled=true;
		// }
		distTravelled += pose.distance(lastPose);
		lastPose = pose;

		distTarget = pose.distance(target);

		if(distTarget < 7.5 && close == false){
			close = true;
			params.maxSpeed=fmax(fabs(prevLateralOut), 60);
		}

		lemlib::Pose carrot = initialCarrot+(carrot-initialCarrot)*(1-params.glead);

		if (close) carrot = target;

		const bool robotSide = (pose.y - target.y) * -sin(target.theta) <= 
			(pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
		const bool carrotSide = (carrot.y - target.y) * -sin(target.theta) <=
            (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
		const bool sameSide = robotSide == carrotSide;

		if (!sameSide && prevSameSide && close && params.minSpeed != 0) break;
		prevSameSide = sameSide;

		const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
		const float angularError =
            close ? lemlib::angleError(adjustedRobotTheta, target.theta) : lemlib::angleError(adjustedRobotTheta, pose.angle(carrot));
		float lateralError = pose.distance(carrot);

		if (close) lateralError *= cos(lemlib::angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= lemlib::sgn(cos(lemlib::angleError(pose.theta, pose.angle(carrot))));
		// lateralSmallExit.update(lateralError);
        // lateralLargeExit.update(lateralError);
        // angularSmallExit.update(radToDeg(angularError));
        // angularLargeExit.update(radToDeg(angularError));
		// float lateralOut = chassis.lateralPID.update(lateralError);
        // float angularOut = chassis.angularPID.update(lemlib::radToDeg(angularError));
		std::vector<float> lateralValues = pid(lateralError, prev_error, inte, g_lead_linear_Controller);
		float lateralOut = lateralValues[0];
		prev_error = lateralValues[1];
		inte = lateralValues[2];

		std::vector<float> angularValues = pid(angularError, prev_error, inte, g_lead_angular_Controller);
		float angularOut = angularValues[0];
		prev_error = angularValues[1];
		inte = angularValues[2];

		angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
		lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

		if (!close) lateralOut = lemlib::slew(lateralOut, prevLateralOut, linearController.slew);

		const float radius = 1 / fabs(lemlib::getCurvature(pose, carrot));
		const float maxSlipSpeed(sqrt(params.chasePower * radius * 9.8));

		lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
		const float overturn = fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

        // prevent moving in the wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

		if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

		prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

		float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

		drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

		pros::delay(10);
    }
	drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
	// if (lateralLargeExit.getExit() && lateralSmallExit.getExit()) lateralSettled = true;
}

	


void pullHangDown(){
	int target = 622;
	int error;
	int thresh = 50;
	while(true){
		error = target - abs(kicker2.get_position());
		if(error < thresh){
			break;
		}
		kicker=90;
		pros::delay(20);
	}
	kicker =0;

}

void fastTurn(float thetatarget, float error, bool isClockwise){
	int reverser = 1;
	if(isClockwise){
		reverser = -1;
	}
	leftMotors=64 * reverser;
	rightMotors=-64 * reverser;
	while(abs(thetatarget - chassis.getPose().theta) > error){
		pros::delay(10);
	}
	leftMotors=0;
	rightMotors=0;
}

void rush(float y, float error){
	
	leftMotors = (abs(y)/y) * 127;
	rightMotors = (abs(y)/y) * 127;
	//(abs(x - chassis.getPose().x) > error) &&
	while( (abs(y - chassis.getPose().y) > error)){
		pros::delay(10);
	} 
	leftMotors=0;
	rightMotors=0;
}
void cancel(){
	leftMotors = 0;
	rightMotors = 0;
}
void ram(int time){
	leftMotors = 127;
	rightMotors = 127;
	pros::delay(time);
	leftMotors=0;
	rightMotors=0;
}
void ram_reverse(int time){
	leftMotors=-127;
	rightMotors=-127;
	pros::delay(time);
	leftMotors=0;
	rightMotors=0;
}
void ram_power(int time, int power){
	leftMotors=power;
	rightMotors=power;
	pros::delay(time);
	leftMotors=0;
	rightMotors=0;
}
void oneBallPush(){
	chassis.moveToPose(22, -17, -90, 3000, {.forwards=false, .minSpeed=65}, false);
	ram(3000);
	chassis.moveToPose(0, 0, 0, 3000);
}
void threeballsafe(){
	chassis.moveToPose(3, 13, 45, 3000, {}, false);
	flapLeftPiston.set_value(true);
	chassis.turnToPoint(0, 20, 3000);
	flapLeftPiston.set_value(false);
	// leftMotors=-36;
	// rightMotors=-36;
	chassis.moveToPose(-4, -4, 0, 3000, {.forwards=false}, false);
	int target = -12000;
	int error= 150;
	while(abs(target - rot_sensor.get_position())> error){
		hang=127;
		pros::delay(10);
	}
	hang = 0;
	leftMotors=-30;
	rightMotors=-30;


	// intake = 127;
	// chassis.moveToPose(0, 7, 0, 3000, {.forwards=true}, false);
	// chassis.moveToPose(1.75, -16, 0, 3000, {.forwards=false}, false);
	// flapLeftPiston.set_value(true);
	// chassis.moveToPose(8.5, -34, -75, 3000, {.forwards=false}, false);
	// flapLeftPiston.set_value(false);
	// chassis.moveToPose(3, -25, -50, 3000);
	// ram(3000);
	// chassis.moveToPose(3, -25, -50, 3000);
	// chassis.turnToPoint(40, -32, 3000);
	// chassis.moveToPose(40, -32, 90, 3000);

}
void qual_score(){

	chassis.moveToPose(-1, 14, -30, 1000);
	intake=127;
	chassis.waitUntil(1);
	intake=0;

	chassis.waitUntil(2);
		vertWing.set_value(true);
	pros::delay(200);
	vertWing.set_value(false);

	chassis.turnToHeading(-180, 800, {.minSpeed=127});
	chassis.moveToPose(0, 19.5, -194, 800, {.forwards=false}, false);

}
void sixballsafe(){
	chassis.setPose(0, 0, 0);
		// pros::delay(200);
	intake=127;
	pros::delay(200);
	int msb = 40;
	chassis.moveToPoint(2.5, -20, 3000, {.forwards=false});
	chassis.moveToPose(12, -47, -30, 3000, {.forwards=false});
	chassis.turnToHeading(135, 3000);
		chassis.moveToPose(20, -50, 100, 3000, {.forwards=true}, false);
		chassis.moveToPose(20, -50, 120, 3000, {.forwards=false}, false);
	chassis.moveToPose(19.5, -49, 110, 1000, {.minSpeed=127}, false);
	ram(1000);
	double temp_1_x = chassis.getPose().x; double temp_1_y = chassis.getPose().y;
	chassis.moveToPoint(temp_1_x-8.5, temp_1_y-21, 3000, {.forwards=false});
	chassis.turnToHeading(-60, 3000);
	chassis.moveToPose(temp_1_x+3, temp_1_y-4, -60, 3000, {.forwards=true, .minSpeed=127}, false);




	// chassis.moveToPoint(2.5, -32, 1000, {.forwards=false, .minSpeed=msb});
	// chassis.turnToPoint(8,-40, 1000, {.forwards=false, .minSpeed=20});
	// chassis.moveToPoint(13, -47, 3000, {.forwards=false});
	// chassis.turnToPoint(33, -67, 1000, {.minSpeed=20}, false);
	// vertWing.set_value(true);
	// pros::delay(200);
	// // chassis.turnToHeading(34, 3000); //just a test
	// chassis.turnToPoint(99, -30, 1000, {.minSpeed=msb}, false);
	// pros::delay(200);
	// vertWing.set_value(false);
	// chassis.turnToHeading(120, 500, {.minSpeed=msb}, false);
	// intake=-127;
	// pros::delay(300);
	// chassis.moveToPoint(7, -41, 1000, {.forwards=false});

	// chassis.turnToPoint(1.5, -30, 1000, false);
	// chassis.turnToHeading(-58, 1000, false);
	// vertWing.set_value(true);
	//comment out mar 2
	// flapLeftPiston.set_value(true);
	// ram(700);

	// flapLeftPiston.set_value(false);
	// pros::delay(300);
	// chassis.moveToPoint(13, -47, 3000, {.forwards=false},false);
	// 	flapLeftPiston.set_value(false);
	// ram(650);
	// chassis.setPose(37, -59, chassis.getPose().theta);
	// ram(100);
	// chassis.moveToPose(36, -37.5, 35, 3000, {.forwards=false, .minSpeed=67});
	// // chassis.turnToPoint(45, -35, 3000);
	// intake=127;
	// chassis.moveToPose(51.5, 0, 0, 2000, {.minSpeed=67});
	// // pros::delay(200);
	// chassis.turnToPoint(68, -40.5, 1000, {.minSpeed=67});
	// chassis.moveToPose(68, -40.5, 180, 1000, {.minSpeed=127});


	
	//temporary comment
	// // chassis.waitUntil(7);
	// // flapLeftPiston.set_value(true);
	// intake=0;
	// chassis.moveToPose(52, -5, 135, 3000, {.forwards=false, .minSpeed=64});
	// // flapLeftPiston.set_value(false);
	// chassis.turnToPoint(69, -3.5, 3000, {.minSpeed=64});
	// intake=127;
	// chassis.moveToPose(70.5, 16, 31, 1000, {.minSpeed=64});
	// chassis.turnToPoint(70.5, -36, 1000, {.minSpeed=64});
	// chassis.moveToPose(70.5, -36, 180, 1000, {.minSpeed=64});
	// flapLeftPiston.set_value(true);
	// flapRightPiston.set_value(true);
	// chassis.waitUntil(6);
	// intake=0;

	// chassis.moveToPose(29, -42, 40, 3000);
	// chassis.moveToPose(39, -9, 0, 3000);
	// chassis.moveToPose(36, -58, -90, 1000, {.minSpeed=127}, false);
	// vertWing.set_value(false);
	// pros::delay(700);

	// chassis.setPose(30, -57, chassis.getPose().theta);	

	// //uncomment
	// chassis.moveToPose(21, -35, 50, 3000);
	// intake=127;
	// chassis.moveToPose(39, -3, 0, 3000);
	// chassis.turnToPoint(51, -38, 3000);
	// chassis.moveToPose(49.5, -41, 180, 3000);
	// chassis.waitUntil(8);
	// intake=-127;
	// chassis.moveToPose(36, -16, 150, 3000, {.forwards=false});
	// intake=127;
	// chassis.moveToPose(53, 0.5, 30, 3000);
	// chassis.turnToPoint(49.5, -41, 3000);

	// chassis.moveToPose(51, -41, 180, 3000, {.minSpeed=127});
	// 	flapLeftPiston.set_value(true);
	// flapRightPiston.set_value(true);
	// //uncomment


	/*
	chassis.moveToPose(50, -3, 0, 3000);
	chassis.turnToPoint(47, -41, 3000);
	chassis.moveToPose(47, -41, 180, 3000);
	chassis.waitUntil(6);
	intake=-127;
*/
		// chassis.moveToPose(15, -41, 60, 1000, {.forwards=false, .minSpeed=127}, false);


// flapLeftPiston.set_value(false);
// flapRightPiston.set_value(false);
// while(true){
// 	if()
// }







/*
	chassis.moveToPoint(-42, 51.5, 3000, false);
	chassis.turnToPoint(-23, 77, 1000, false); // move to descore setup
	chassis.moveToPose(-52, 44, 0, 3000, {.forwards=false}); 
	chassis.turnToPoint(-76, 25, 100, false);
	// descore movement
	chassis.moveToPose(-60, 26, 0, 3000, {.forwards=false}); // ram movement
	chassis.moveToPose(-57, 44, 45, 3000, {}); //back up from side of goal to score first triball
	chassis.turnToPoint(-60, 26, 3000);
	//
	/*
	chassis.turnToPoint(-60, 26, 1500);
	chassis.turnToPoint(-51, 31, 1500);
		chassis.turnToPoint(-60, 26, 1500);
	chassis.moveToPose(-60, 26, 0, 3000, {.forwards=false}); // ram movement

/*
	chassis.moveToPose(-60, 26, 180, 3000, {}); // ram movement
	chassis.waitUntil(3);
	intake=-127;
	


	
	chassis.moveToPoint(-48, 41, 3000, false); // posibly need to change back to move to pose for speed (backwards)
	chassis.turnToPoint(-6.5, 23.5, 3000);
	intake=127;
	chassis.moveToPose(-6.5, 23.5, -90, 3000, {}); // ram movement
	chassis.turnToPoint(-45, 13, 3000); // aim left triball 
	chassis.moveToPose(-45, 13, 90, 3000, {}); // pick up first triball
	chassis.turnToPoint(-3.5, 0.5, 3000); //face goal
	chassis.moveToPose(-3.5, 0.5, 90, 3000); // pick up triball
	chassis.turnToPoint(-44, 0, 3000); // face goal
	chassis.moveToPose(-44, 0, -90, 3000); // ram movement

	*/
}

void tuning_movements(){
	// chassis.moveToPose(20, 30, 90, 3000, {.minSpeed=127});
	// rush(20, 0.5);
	// rush(-20, 0.5);
	flapRightPiston.set_value(true);
	intake=127;
	chassis.moveToPose(-9, 50, -10, 3000, {.minSpeed=127});
	pros::delay(300);
	flapRightPiston.set_value(false);
	chassis.moveToPose(-1, 4, 14, 3000, {.forwards=false, .minSpeed=127}, false);
	intake=-127;
	pros::delay(340);
	chassis.turnToPoint(0, -2, 700);
	chassis.moveToPose(-13, -1.25, -90, 3000, {});
	intake=127;
	chassis.moveToPose(-39, -1.25, -90, 3000, {.minSpeed=66});
	chassis.moveToPose(1, 2.5, -90, 3000, {.forwards=false, .minSpeed=66});
	// fastTurn(-90, 20, false);
	// chassis.moveToPose(-6, -1, -90, 3000, {.lead=0.3, .minSpeed=40});

	// intake=127;
	// //-27

	// chassis.moveToPose(-32, -1, -90, 3000, {}, false);
	// // pros::delay(500);
	// chassis.moveToPose(2, -3, -90, 3000, {.forwards=false});
	chassis.turnToPoint(18.5, 25.5, 1000, false);
	// pros::delay(400);
	flapLeftPiston.set_value(true);
	chassis.moveToPose(14, 16, 200, 3000, {.forwards=false,.lead=0.6, .minSpeed=127});
	chassis.waitUntil(16);
	flapRightPiston.set_value(true);
	// flapLeftPiston.set_value(false);
	// ram(400);
	// chassis.moveToPose(10,12 ,215, 3000);
	// chassis.turnToPoint(23, 15, 700);
	// chassis.turnToPoint(27, 30, 700);

	// chassis.moveToPose(32, 38, 45, 3000, {.lead=0.8, .minSpeed=127});
	// flapLeftPiston.set_value(true);
	// chassis.waitUntil(2);
	// intake=-127;
	



}
void winpointScore(){
	vertWing.set_value(true);
	pros::delay(500);
	chassis.turnToHeading(-65, 1000, false);
	vertWing.set_value(false);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPose(2, -15, 45, 3000, {.forwards=false}, false);
	ram_reverse(500);
	chassis.moveToPose(4, 4, -45, 3000);
	intake=-127;
	chassis.moveToPose(-25, 29, -45, 3000);
}
void winpoint_old(){
	vertWing.set_value(true);
	pros::delay(2000);
	chassis.turnToHeading(-45, 3000, false);
	pros::delay(500);
	vertWing.set_value(false);
	chassis.moveToPose(-25, 29, -45, 3000);
	intake=-30;

}
void disrupt_pass(){
	chassis.moveToPose(6, 48.5, 0, 1200, {.minSpeed=127});
	intake=127;
		flapRightPiston.set_value(true);
	pros::delay(100);
	flapRightPiston.set_value(false);
		chassis.moveToPose(-7, 49, 90, 2500, {.forwards=false, .minSpeed=127});
		chassis.moveToPose(9, 49, 90, 2500, {.minSpeed=60});
		chassis.waitUntil(2);
		intake=0;
		while(true){
			if(abs(chassis.getPose().x - 6) < 0.5){
				break;
			}
			pros::delay(10);
		}
		intake=-127;
		// pros::delay(100);
		chassis.moveToPoint(6.5, 49, 1000, {.minSpeed=127});

		// chassis.turnToPoint(18.5, 54.5, 1000, {.minSpeed=127});
		// chassis.moveToPose(18.5, 54.5, 0, 1000, {});
		// chassis.moveToPose(10, 43, 90, 2500, {});
		// chassis.waitUntil(3);
		// intake=-127;
		// chassis.turnToPoint(13, 55.5, 1000, {.minSpeed=127});
		// chassis.moveToPose(13, 53, 0, 1000);
		// intake=127;



}
void testing(){
	// Winpoint uncomment later
	// chassis.moveToPose(-22, -17, 90, 3000, {.forwards=false, .minSpeed=65});
	// pros::delay(700);
	// chassis.moveToPose(-10, -12, 45, 3000, {.forwards=true});
	// chassis.turnToPoint(-22, -17, 3000, false);
	// flapLeftPiston.set_value(true);
	// chassis.moveToPose(-3.5, 0, 180, 3000, {.forwards=false, .lead=0.4}, false);
	// flapLeftPiston.set_value(false);
	// // chassis.turnToPoint(2, 31, 3000);
	// intake=-127;
	//feb 24 breakpoint
	intake=127;
	vertWing.set_value(true);
	pros::delay(800);
	chassis.turnToHeading(-45, 1000, false);
	pros::delay(500);
	vertWing.set_value(false);
	chassis.turnToHeading(180, 1000);
	chassis.moveToPose(-5.5, -27, -135, 2500, {}, false);
	intake=-127;
	pros::delay(500);
	intake=0;
	chassis.turnToHeading(0, 1000);

	chassis.moveToPose(-18, 4.5, -40, 2500);
	chassis.moveToPose(-34, 20, -45, 2500);
	intake=-127;

	// chassis.moveToPose(-25, 29, -45, 3000);
	// intake=-30;



	// chassis.moveToPose(2, 26.5, 0, 3000, {.forwards=true, .minSpeed=127, .lead=0.4});
	

	//Disrupt Elims
	// flapLeftPiston.set_value(true);
	// intake = 127;
	// chassis.moveToPose(12, 37, 0, 3000, {.forwards=true, .minSpeed=65});
	// pros::delay(300);
	// flapLeftPiston.set_value(false);
	// chassis.moveToPose(6, 32.5, 90, 3000, {.forwards=false});
	// flapLeftPiston.set_value(true);
	// flapRightPiston.set_value(true);
	// chassis.moveToPose(27, 34, 90, 3000, {.forwards=true, .minSpeed=40}, false);
	// intake = -127;
	// pros::delay(3000);
	// flapLeftPiston.set_value(false);
	// flapRightPiston.set_value(false);

}
void states_skills(){

	flapLeftPiston.set_value(true);

	chassis.turnToHeading(165, 800);
	pros::delay(150);
	flapLeftPiston.set_value(false);
	chassis.moveToPoint(-13.5, 10.5, 2000, {.forwards=false, .maxSpeed=80});
	chassis.turnToHeading(55, 500, false);
	float preKick_x = chassis.getPose().x; float preKick_y = chassis.getPose().y; float preKick_theta = chassis.getPose().theta;
	kicker=100;
		vertWing.set_value(true);
	// chassis.moveToPose(-14, 6, 60, 3000, {.forwards=false});
	pros::delay(22000);
	vertWing.set_value(false);
	kicker=0;

	chassis.setPose(preKick_x, preKick_y, preKick_theta);
	chassis.turnToHeading(180, 500, {}, false);
	chassis.moveToPose(-15, 25, 180, 1000, {.forwards=false, .minSpeed=127}, false);
	ram_reverse(600);
	chassis.moveToPose(1, 35, 37, 2000, {.minSpeed=30});
	chassis.turnToHeading(-135, 600, {.minSpeed=127});

	chassis.moveToPose(27.5, 45, 220, 3000, {.forwards=false, .minSpeed=127});
	// chassis.turnToHeading(220, 500, {.minSpeed=127}, false);
	vertWing.set_value(true);
	chassis.moveToPose(40, 120, 180,5000, {.forwards=false, .minSpeed=127}, false);
	chassis.moveToPose(36, 90, 192, 3000, {.minSpeed=127}, false);
	// chassis.turnToHeading(220, 300, {.minSpeed=127});
	chassis.moveToPose(39.5, 120, 175, 3000, {.forwards=false, .minSpeed=127}, false);
	vertWing.set_value(false);
	// pros::delay(100);
	pros::delay(25);
	chassis.setPose(0, 0, chassis.getPose().theta-180);

	chassis.moveToPose(26, 0, 125, 3000, {.minSpeed=90});

	chassis.moveToPose(36, -7, 160, 3000, {.minSpeed=90});
	chassis.moveToPose(27, -24.5, 245, 3000, {.minSpeed=90});
	// flapLeftPiston.set_value(true);
	flapRightPiston.set_value(true);
	chassis.moveToPose(11, -31.5, 270, 3000, {.minSpeed=127});
		// flapLeftPiston.set_value(false);
	flapRightPiston.set_value(false);
	chassis.moveToPoint(-42, -31.5, 3000, {.minSpeed=127}, false);
	// chassis.waitUntil(3);
	// pullHangDown();
	// chassis.waitUntil(8);
	flapRightPiston.set_value(true);
	// flapLeftPiston.set_value(true);
	chassis.moveToPose(-78.5, 9, 348, 3000, {.minSpeed=127}, false);
	ram(300);
	// flapLeftPiston.set_value(false);
	flapRightPiston.set_value(false);
	// chassis.moveToPoint(-88, -20, 3000, {.forwards=false}, false);
	double temp_x_1 = chassis.getPose().x; double temp_y_1 = chassis.getPose().y;
	chassis.moveToPoint(chassis.getPose().x+5, chassis.getPose().y-23, 3000, {.forwards=false}, false);
	chassis.turnToHeading(140, 800, {.minSpeed=127},  false);
	//uncomment her mar 8
	chassis.moveToPose(temp_x_1-6, temp_y_1-5, 170, 3000, {.forwards=false, .minSpeed=20}, false);


	// // chassis.turnToHeading(142, 3000, false);
		ram_reverse(450);
	// // // // chassis.moveToPose()
	// // // // // ram_reverse(1700);


	pros::delay(50);
	chassis.setPose(0, 0, 0);
	chassis.moveToPose(-24, 4, -135, 3000, {.minSpeed=60});
	// chassis.moveToPoint(-47, -4.5, 3000, {.minSpeed=100});
	// chassis.turnToPoint(-23, -18, 300, {.forwards=false});
	chassis.moveToPose(-24, -21, 90, 3000, {.forwards=true, .minSpeed=80}, false);
	chassis.turnToHeading(90, 200, {.minSpeed=127}, false);


	// chassis.turnToHeading(90, 200, {.minSpeed=127}, false);
	// ram_power(1000, -127);
	int reset_y_constant = -12;
	ram(700);
	// chassis.setPose(chassis.getPose().x, reset_y_constant, chassis.getPose().theta);
	// pros::delay(50);

	chassis.moveToPose(-22, -17, -180, 2000, {.forwards=true, .minSpeed=64}, false);
	flapLeftPiston.set_value(true);
	chassis.moveToPose(-5, -37.25, 90, 1200, {.minSpeed=127}, false);
		chassis.turnToHeading(90, 200, {.minSpeed=127}, false);
	ram(550);
	// chassis.setPose(chassis.getPose().x, reset_y_constant, chassis.getPose().theta);
	flapLeftPiston.set_value(false);
	chassis.moveToPose(-16, -34.5, -180, 3000, {.forwards=true, .minSpeed=64});
	chassis.moveToPose(-2, -53, 90, 1200, {.lead=0.73, .minSpeed=64}, false);
		chassis.turnToHeading(90, 200, {.minSpeed=127}, false);
	ram(500);
	// chassis.setPose(chassis.getPose().x, reset_y_constant, chassis.getPose().theta);
	chassis.moveToPose(-9.5, -45, -180, 3000, {.forwards=true, .minSpeed=100});
	chassis.moveToPose(-11, -76, -180, 3000, {.minSpeed=65});
	chassis.moveToPose(6, -78, 35, 3000, {.minSpeed=50});
	flapRightPiston.set_value(true);
	flapLeftPiston.set_value(true);
	chassis.moveToPose(19, -75, 110, 1400, {.minSpeed=127}, false);
	chassis.waitUntil(2);
	flapRightPiston.set_value(false);
		chassis.turnToHeading(90, 200, {.minSpeed=127}, false);
	ram(450);
	// chassis.setPose(chassis.getPose().x, reset_y_constant, chassis.getPose().theta);
		flapRightPiston.set_value(false);
	flapLeftPiston.set_value(false);

 	chassis.moveToPose(23, -100, -45, 3000, {.forwards=false, .minSpeed=50});
		chassis.turnToHeading(-160, 400, {.minSpeed=127});
		chassis.moveToPose(4.5, -110, -135, 3000, {.minSpeed=127});
		// chassis.moveToPose(0, -114, 225, 3000, {.minSpeed=127});
		chassis.turnToHeading(225, 300, {.minSpeed=127});
		chassis.moveToPose(18.5, -110, 45, 3000, {.minSpeed=127});
		chassis.moveToPose(35, -103.5, 32, 3000, {.minSpeed=127}, false);
		flapLeftPiston.set_value(true);
		flapRightPiston.set_value(true);
		ram(600);
		double temp_x_2 = chassis.getPose().x; double temp_y_2 = chassis.getPose().y;
	chassis.moveToPoint(chassis.getPose().x+5, chassis.getPose().y-23, 3000, {.forwards=false}, false);
	// ram(600);

	chassis.moveToPose(temp_x_2-6, temp_y_2-5, 12, 3000, {.forwards=false, .minSpeed=20}, false);
	ram(600);
	chassis.setPose(0, 0, chassis.getPose().theta);
	chassis.moveToPoint(-7, -36, 1000, {.forwards=false, .minSpeed=127});
	chassis.waitUntil(2);
		hangUp.set_value(true);
	hangDown.set_value(true);
	chassis.turnToHeading(-90, 700, {.minSpeed=127});

	chassis.moveToPose(-33, -35, -90 ,1000, {.minSpeed=127}, false);

	// hangDown.set_value(false);
	hangDown.set_value(false);
	hangUp.set_value(false);
	// chassis.moveToPose(-3, -102.5, 150, 3000);
	// chassis.moveToPose(-2, -108, 35, 3000);






	
	// chassis.moveToPose(-15, -40, -180, {.forwards=true});
	// chassis.moveToPose(-16, -74, 35);

	// chassis.turnToHeading(-180, 3000, {.forwards=false});
	// chassis.moveToPose(-23, -56.5, -180,  3000, {.forwards=true, }, false);
	// chassis.turnToHeading(90, 1000, {.minSpeed=127});
	// 		flapLeftPiston.set_value(true);
	// chassis.moveToPose(-3, -40.5, 90, 3000, {.minSpeed=40}, false);
	// ram(1000);
	// flapLeftPiston.set_value(false);
	// pros::delay(50);
	// chassis.turnToHeading(-90, 1500, {}, false);

	// leftMotors=100;
	// rightMotors=100;
	// pros::delay(1500);
	// leftMotors=0;
	// rightMotors=0;


	// chassis.moveToPose(-45, -39, -170, 3000, {.minSpeed=10}, false);
	// ram_power(1700, 100);




	// flapLeftPiston.set_value(true);
	// chassis.moveToPose(-19, -30, -270, 3000, {.minSpeed=127});
	// chassis.moveToPose(-50, -33, -180, 3000, {.forwards=false});
	// chassis.moveToPose(-50, -61, -180, 3000);
	// chassis.turnToHeading(-285, 3000);
	// chassis.moveToPose(-21, -48, -270, 3000);
	// flapLeftPiston.set_value(true);

	// flapRightPiston.set_value(true);
	// chassis.turnToPoint(-51, 3, 3000, {.minSpeed=30});
	// chassis.moveToPose(-51, 3, -90, 3000, {.forwards=false, .minSpeed=127});

	// chassis.moveToPose(-68, 14, 155, 3000, {.minSpeed=127}, false);
	// flapRightPiston.set_value(true);
	// flapLeftPiston.set_value(true);
	// // chassis.waitUntil(6);
	// // chassis.cancelMotion();
	// ram(1000);
	// chassis.moveToPose(-68, -14, 315, 3000, {.forwards=false});
	// flapRightPiston.set_value(false);
	// 	flapLeftPiston.set_value(false);

	// chassis.turnToHeading(155, 3000, false);
	// ram_reverse(1000);
	// pros::delay(200);
	// chassis.setPose(0, 0, chassis.getPose().theta);

	// leftMotors=-60;
	// rightMotors=-60;
	// pros::delay(550);
	// leftMotors=0;
	// rightMotors=0;


	// pros::delay(400);
	// chassis.setPose(0, 0, chassis.getPose().theta);
	// pros::delay(400);
	// chassis.moveToPose(0, 10, -168, 3000, {.forwards=false});
	// chassis.moveToPose(-4, -2, 0, 3000, {.forwards=true, .minSpeed=90});
	// ram_reverse(1500);
}
// void anirudh_disrupt()
void disrupt(){
	// 	flapLeftPiston.set_value(true);
	// intake = 127;
	// chassis.moveToPose(12, 37, 0, 3000, {.forwards=true, .minSpeed=65});
	// pros::delay(300);
	// flapLeftPiston.set_value(false);
	// chassis.moveToPose(6, 32.5, 90, 3000, {.forwards=false});
	// flapLeftPiston.set_value(true);
	// flapRightPiston.set_value(true);
	// chassis.moveToPose(27, 34, 90, 3000, {.forwards=true, .minSpeed=40}, false);
	// intake = -127;
	// pros::delay(3000);
	// flapLeftPiston.set_value(false);
	// flapRightPiston.set_value(false);
	// flapRightPiston.set_value(true);

	chassis.moveToPose(6, 49, 0, 1200, {.minSpeed=127});
	intake=127;
		flapRightPiston.set_value(true);
	pros::delay(100);
	flapRightPiston.set_value(false);


	
	// chassis.moveToPose(3, 43, 0, 1200, {.forwards=false, .minSpeed=127});
			chassis.moveToPose(-18, 11.5, 40, 1800, {.forwards=false}, false);
	chassis.turnToHeading(135, 700, {}, false);
			intake=-127;
		pros::delay(300);
		intake=127;
	chassis.turnToHeading(40, 700);


		// chassis.turnToHeading(90, 3000, false);
		// chassis.turnToPoint(10, 41.5, 3000);
		
	// chassis.waitUntil(3);
	// intake=-127;
	chassis.moveToPose(13, 54 ,0, 3000, {.minSpeed=127});
	// chassis.moveToPose(25, 47, 0, 1200, {});
	// flapLeftPiston.set_value(true);
	chassis.moveToPose(-26, 12.5, 40, 1800, {.forwards=false});
	flapLeftPiston.set_value(false);
	chassis.turnToHeading(155, 500, {.minSpeed=127});
	chassis.moveToPose(-8.5, -4.5, 25, 1000, {.lead=0.5});
	chassis.waitUntil(7);
	vertWing.set_value(true);
	pros::delay(450);
	vertWing.set_value(false);
		// chassis.moveToPose(-26, 6, 40, 1800, {.forwards=false});
		// chassis.turnToHeading(-100, 450, {.minSpeed=127});

		// // chassis.moveToPoint(-26, 7.5, 1000, {.forwards=true});
		// chassis.turnToHeading(135, 400, {.minSpeed=127}, false);
		// pros::delay(100);
				chassis.turnToHeading(0, 100, {.minSpeed=127}, false);
		pros::delay(100);
		leftMotors=-100;
		rightMotors=100;
		float mod_number = 360;
		while(true){
			if(abs(-180 - chassis.getPose().theta) < 17){
				break;
			}
			pros::delay(10);
		}
		leftMotors=0;
		rightMotors=0;
	chassis.turnToHeading(-270, 250, {.minSpeed=127});
	chassis.moveToPose(17, -6, 90, 3000, {.minSpeed=127});
	chassis.waitUntil(3);
		flapRightPiston.set_value(true);
	chassis.waitUntil(11);
		flapRightPiston.set_value(false);
	chassis.waitUntil(13);
	flapLeftPiston.set_value(true);

	intake=-127;
	chassis.moveToPoint(0 , -6, 500, {.forwards=false, .minSpeed=127});
			flapLeftPiston.set_value(false);
	// chassis.moveToPose(-28, 5,  105, 1000, {.forwards=false, .minSpeed=127});
	// chassis.turnToHeading(-45, 700, {.minSpeed=127});
		// intake=127;
	// chassis.moveToPose(-38, 23, 0, 1000, {}, false);

	// // pros::delay(250);
	// intake=-127;
	// pros::delay(450);
	// chassis.turnToHeading(135,700, {.minSpeed=127});
	chassis.moveToPose(-33, 12, 108, 1000,{.forwards=false}, false);
	flapLeftPiston.set_value(true);
	flapRightPiston.set_value(true);

	// chassis.moveToPose(-32, -4, 105, 1000);


	// chassis.moveToPose(-26, 10, 90, 3000, {.forwards=false, .minSpeed=127}, false);
	// pros::delay(100);
	// flapLeftPiston.set_value(true);



	// vertWing.set_value(true);



	//new disrupt feb 24


	// chassis.setPose(0, 0, 0);
	// pros::delay(50);
	// flapRightPiston.set_value(true);
	// chassis.moveToPose(0, 47, 0, 3000, {.minSpeed=127});
	// intake=127;
	// pros::delay(50);
	// flapRightPiston.set_value(false);
	// chassis.moveToPose(-2, 2, 0, 3000, {.forwards=false, .minSpeed=64});
	// chassis.turnToHeading(80, 1000, false);
	// intake=-127;
	// pros::delay(400);
	// chassis.turnToPoint(8, 55, 1000);
	// chassis.moveToPose(8, 55, -7, 3000);
	// intake=127;
}

void disrupt_edited(){
	// 	flapLeftPiston.set_value(true);
	// intake = 127;
	// chassis.moveToPose(12, 37, 0, 3000, {.forwards=true, .minSpeed=65});
	// pros::delay(300);
	// flapLeftPiston.set_value(false);
	// chassis.moveToPose(6, 32.5, 90, 3000, {.forwards=false});
	// flapLeftPiston.set_value(true);
	// flapRightPiston.set_value(true);
	// chassis.moveToPose(27, 34, 90, 3000, {.forwards=true, .minSpeed=40}, false);
	// intake = -127;
	// pros::delay(3000);
	// flapLeftPiston.set_value(false);
	// flapRightPiston.set_value(false);
	// flapRightPiston.set_value(true);

	chassis.moveToPose(6, 49.5, 0, 1200, {.minSpeed=127});
		flapRightPiston.set_value(true);
	pros::delay(100);
	flapRightPiston.set_value(false);
	intake=127;

	
	// chassis.moveToPose(3, 43, 0, 1200, {.forwards=false, .minSpeed=127});
			chassis.moveToPose(-18, 11.5, 40, 2500, {.forwards=false, .minSpeed=50}, false);
	chassis.turnToHeading(135, 700, {}, false);
			intake=-127;
		pros::delay(500);
		intake=127;
	chassis.turnToHeading(40, 700);


		// chassis.turnToHeading(90, 3000, false);
		// chassis.turnToPoint(10, 41.5, 3000);
		
	// chassis.waitUntil(3);
	// intake=-127;
	chassis.moveToPose(13, 54 ,0, 3000, {.minSpeed=127});
	// chassis.moveToPose(25, 47, 0, 1200, {});
	// flapLeftPiston.set_value(true);
	chassis.moveToPose(-25.25, 12.5, 40, 1800, {.forwards=false});
	flapLeftPiston.set_value(false);
	chassis.turnToHeading(155, 1200, {.maxSpeed=80});
	chassis.moveToPose(-8.5, -4.5, 25, 1000, {.lead=0.5});
	chassis.waitUntil(7);
	vertWing.set_value(true);
	pros::delay(400);
	vertWing.set_value(false);
		// chassis.moveToPose(-26, 6, 40, 1800, {.forwards=false});
		// chassis.turnToHeading(-115, 1000, {.minSpeed=127, .earlyExitRange=0});
				// chassis.turnToHeading(-115, 1000, {.minSpeed=127, .earlyExitRange=0});
		// 		chassis.turnToHeading(0, 1000, {.minSpeed=127}, false);
		// 		pros::delay(150);
		// 		chassis.turnToHeading(-115, 1000, {.minSpeed=127}, false);

		// // chassis.moveToPoint(-26, 7.5, 1000, {.forwards=true});
		// chassis.turnToHeading(90, 400, {.minSpeed=127} , false);
		chassis.turnToHeading(0, 100, {.minSpeed=127}, false);
		pros::delay(100);
		leftMotors=-100;
		rightMotors=100;
		while(true){
			if(abs(90 - chassis.getPose().theta) < 10){
				break;
			}
			pros::delay(10);
		}
		leftMotors=0;
		rightMotors=0;

		// pros::delay(100);
	pros::delay(200);
	chassis.moveToPose(18, -1, 90, 3000, {.minSpeed=127});
	chassis.waitUntil(3);
		flapRightPiston.set_value(true);
	chassis.waitUntil(11);
		flapRightPiston.set_value(false);
	chassis.waitUntil(13);
	flapLeftPiston.set_value(true);

	intake=-127;
	chassis.moveToPoint(-5 , -2, 1000, {.forwards=false, .minSpeed=127});
			flapLeftPiston.set_value(false);
	chassis.moveToPose(-22, 4, 110, 1000, {.forwards=false});

	// chassis.moveToPose(-26, 10, 90, 3000, {.forwards=false, .minSpeed=127}, false);
	// pros::delay(100);
	// flapLeftPiston.set_value(true);



	// vertWing.set_value(true);



	//new disrupt feb 24


	// chassis.setPose(0, 0, 0);
	// pros::delay(50);
	// flapRightPiston.set_value(true);
	// chassis.moveToPose(0, 47, 0, 3000, {.minSpeed=127});
	// intake=127;
	// pros::delay(50);
	// flapRightPiston.set_value(false);
	// chassis.moveToPose(-2, 2, 0, 3000, {.forwards=false, .minSpeed=64});
	// chassis.turnToHeading(80, 1000, false);
	// intake=-127;
	// pros::delay(400);
	// chassis.turnToPoint(8, 55, 1000);
	// chassis.moveToPose(8, 55, -7, 3000);
	// intake=127;
}



// void skills(){
// 	target=3000;
// 	// blocker.set_value(true);
// 	flapRightPiston.set_value(true);
// 	// chassis.turnToPoint(0, -10, 3000);
// 	pros::delay(400);
// 	// blocker.set_value(false);
// 	flapRightPiston.set_value(false);
// 	chassis.moveToPose(-14.5, 9.5, -106.5, 2000, {.forwards=true, .lead=0.45});
// 	// cata=-127;
// 	pros::delay(30500);
// 	chassis.setPose(-14.5, 9.5, -105);
// 	pros::delay(200);
// 	// flapRightPiston.set_value(false);
// 	chassis.moveToPose(10.5, -6.5, -90, 3000, {.forwards=false, .lead=0.6});
// 	// cata_pulldown();
// 	chassis.moveToPose(83.4, -6.5, -90, 3000, {.forwards=false});
// 	chassis.waitUntil(50);
// 	flapRightPiston.set_value(true);
// 	chassis.turnToPoint(106.5, 20.5, 3000, false);
// 	chassis.moveToPose(101.5, 13, -160, 3000, {.forwards=false, .lead = -0.3}, false);
// 	ram(1000);
// 	chassis.moveToPose(101.5, 13, -160, 3000, {.forwards=true, .lead = 0.3}, false);
// 	flapRightPiston.set_value(false);
// 	ram(1000);
// 	pros::delay(700);
// 	chassis.setPose(0, 0, 0);
// 	pros::delay(200);
// 	chassis.moveToPose(17.5, 16.5, 55, 3000);
// 	chassis.turnToPoint(50, -10, 700);
// 	// chassis.follow(wingstart_txt, 15, 3000);
// 	flapRightPiston.set_value(false);
// 	//fix this tmw for DEC 29
// 	chassis.moveToPose(53, -12, 180, 2000, {.lead = 0.43}, false);
// 	chassis.moveToPose(20, -17.5, 90, 2000, {.forwards=false, .lead=-0.4});
// 	// chassis.turnToPoint(83, -14, 700, true, 127, false);
// 	flapLeftPiston.set_value(true);
// 	ram(700);
// 	flapLeftPiston.set_value(false);
// 	chassis.moveToPose(54, -58, 180, 2000);
// 	chassis.turnToPoint(81, -79, 700);
// 		flapLeftPiston.set_value(true);
// 	flapRightPiston.set_value(true);
// 	chassis.moveToPoint(27, -34.5, 3000, false, 64, false);
// 	flapLeftPiston.set_value(false);
// 	flapRightPiston.set_value(false);
// 	chassis.moveToPose(56, -30, 90, 3000, {.forwards=true}, false);
// 	flapLeftPiston.set_value(true);
// 	flapRightPiston.set_value(true);
// 	ram(1300);
// 		flapLeftPiston.set_value(false);
// 	flapRightPiston.set_value(false);
// 	// chassis.moveToPose(45.5, -44, 215, 3000, {.forwards=true});
// 	chassis.moveToPoint(54, -28, 3000);
// 	intake=-127;
// 	// flapLeftPiston.set_value(false);
// 	// flapRightPiston.set_value(false);




// 	// chassis.moveToPose(53, -60, 245, 2000);
// 	// flapLeftPiston.set_value(true);
// 	// 	flapRightPiston.set_value(true);

// 	// chassis.moveToPose(100, 5, -165, 3000, {.forwards=false}, false);
// 	// chassis.moveToPose(90, 0, -150, 3000, {}, false);
// 	// ram(3000);
// 	// chassis.moveToPose(90, 0, -150, 3000, {}, false);
// 	// chassis.turnToPoint(106, 27, 1000, false);
// 	// chassis.moveToPose(103, 20, -180, 3000, {.forwards = false, .lead=0.5});



// 	// blocker.set_value(true);
// 	// flapLeftPiston.set_value(true);
// 	// pros::delay(600);
// 	// flapLeftPiston.set_value(false);
// 	// blocker.set_value(false);
// 	// // chassis.moveToPose(0, 0, 100, 4000, false, true, 0, 0.6, 20, false);
// 	// chassis.turnToPoint(6, -16.3, 3000);
// 	// cata=-127;
// 	// // 26500
// 	// pros::delay(26500);

// 	// // cata=0;
// 	// // chassis.turnToPoint(-28, -5, 3000, true);
// 	// // chassis.turnToPoint(-24, 5, 3000);
// 	// // flapLeftPiston.set_value(true);
// 	// // chassis.turnToPoint(13, 5, 4000);
// 	// flapLeftPiston.set_value(true);
// 	// chassis.moveToPose(-28, -3, 90, 4000, false, false, 4, 0.6, 127);
// 	// flapLeftPiston.set_value(false);
// 	// chassis.setPose(chassis.getPose().x, chassis.getPose().y, 90);
// 	// intake = -127;
// 	// int error=500;
// 	// int cur_pos=0;
// 	// int target= 3900;
// 	// while (abs(abs(cur_pos)-target) > error){
// 	// 	cur_pos=rot_sensor.get_position();
// 	// 	cata = -127;
// 	// }
// 	// cata = 0;
// 	// // cata=-127;
// 	// chassis.moveToPose(-2, 25, 0, 3000, false, true, 4, 0.5, 127);
// 	// chassis.moveToPose(-2, 95, 0, 3000);
// 	// cata=0;
// 	// chassis.moveToPose(-20, 85, 180, 3000, false, true, 4, 0.65, 63);
// 	// chassis.moveToPose(-69, 70, -90, 3000, false, true, 4, 0.85, 63);
// 	// // intake=0;
// 	// // pros::delay(500);
// 	// // chassis.turnToPoint(chassis.getPose().x, -100, 3000, 63);

// 	// flapLeftPiston.set_value(true);
// 	// flapRightPiston.set_value(true);

// 	// chassis.moveToPose(chassis.getPose().x, 100, 180, 3000, false, false, 6, 0, 127);
// 	// chassis.setPose(chassis.getPose().x, chassis.getPose().y, 180);
// 	// flapLeftPiston.set_value(false);
// 	// flapRightPiston.set_value(false);
// 	// chassis.moveToPose(-83, 73, 200, 3000, false, true, 4, -0.8, 127);
// 	// flapLeftPiston.set_value(true);
// 	// flapRightPiston.set_value(true);
// 	// chassis.moveToPose(-68, 100, 220, 3000, false, false, 4, 0, 127);
// 	// flapLeftPiston.set_value(false);
// 	// flapRightPiston.set_value(false);
// 	// chassis.moveToPose(-41.5, 80, 150, 3000, false, true, 4, -0.8, 127);
// 	// flapLeftPiston.set_value(true);
// 	// flapRightPiston.set_value(true);
// 	// chassis.moveToPose(-40, 100, 150, 3000, false,false, 4, 0, 127);
// 	// flapLeftPiston.set_value(false);
// 	// flapRightPiston.set_value(false);
// 	// chassis.moveToPose(-38, 80, 150, 3000, false, true, 4, -0.8, 127);
// 	// flapLeftPiston.set_value(true);
// 	// flapRightPiston.set_value(true);
// 	// chassis.moveToPose(-40, 100, 150, 3000, false,false, 4, 0, 127);
// 	// flapLeftPiston.set_value(false);
// 	// flapRightPiston.set_value(false);
// 	// chassis.moveToPose(-38, 80, 150, 3000, false, true, 4, -0.8, 127);
// 	// chassis.moveToPose(-4, 110, 150, 3000, false, false, 4, 0.9, 127);
// }

// double motion_profile(max_acceleration, max_velocity, distance, elapsed_time) {
//   acceleration_dt = max_velocity / max_acceleration;

//   halfway_distance = distance / 2;
//   acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2;

//   if (acceleration_distance > halfway_distance) {
//     acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
//   }

//   acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2;

//   max_velocity = max_acceleration * acceleration_dt;

//   deceleration_dt = acceleration_dt;

//   cruise_distance = distance - 2 * acceleration_distance;
//   cruise_dt = cruise_distance / max_velocity;
//   deceleration_time = acceleration_dt + cruise_dt;
//   entire_dt = acceleration_dt + cruise_dt + deceleration_dt;

//   if (elapsed_time > entire_dt) {
//     return distance;
//   }
//   if (elapsed_time < acceleration_dt) {
//     return 0.5 * max_acceleration * elapsed_time ** 2;
//   }
//   else if (elapsed_time < deceleration_time) {
//     acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2;
//     cruise_current_dt = elapsed_time - acceleration_dt;
//     return acceleration_distance + max_velocity * cruise_current_dt;
//   }
//   else {
//     acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2;
//     cruise_distance = max_velocity * cruise_dt;
//     deceleration_time = elapsed_time - deceleration_time;
//     return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time ** 2;
//   }
// }