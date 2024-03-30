#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"

// drivetrain settings


void initialize() {
    // selector::init();
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    kicker1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    kicker2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // hang.set_value(true);


    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    // pros::delay(1000);
    // rot_sensor.reset_position();
    
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Cata Rotation: %f", kicker2.get_position());
            // pros::lcd::print(3, "LF Temp: %f", lF.get_position());
            pros::lcd::print(4, "RF Temp: %f", rF.get_temperature());
            pros::lcd::print(5, "LM Temp: %f", lM.get_temperature());
            pros::lcd::print(6, "RM Temp: %f", rM.get_temperature());
            pros::lcd::print(7, "LB Temp: %f", lB.get_temperature());
            pros::lcd::print(8, "RB Temp: %f", rB.get_temperature());

            // log position telemetry
            // lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous(){
    disrupt();
}    // if(selector::auton==1){testing();}
    // else if(selector::auton == 2){disrupt();}
    // else if(selector::auton == 3){sixballsafe();}
    // else if(selector::auton==4){states_skills();}
    // // testing();
    // disrupt();
    //60 to 180 for large turns
    // chassis.turnToHeading(90, 3000);
    // chassis.moveToPose(14, 64, 40,  5000, {.minSpeed=80});
    // chassis.moveToPose(24, 84, -32, 3000, {.minSpeed=80});
    // chassis.moveToPose(12, 86, -90, 3000, {.minSpeed=80});
    // chassis.moveToPose(-11, 86, -90, 3000);
    // disrupt();    // chassis.moveToPose()
    // chassis.turnToHeading(-90, 5000);


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
std::string runType = "DS";
void opcontrol() {
    // pros::task(cataTask);
    if(runType=="M"){
        pros::Task ee(states_skills);
        while(true){
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
                break;
            }
            pros::delay(10);
        }
        ee.remove();
    }

    // pros::lcd::print(4, "testing opcontrol");
	while (true) {
		controllerControl();
		intakeControl();
		flapControl();
        // kickerControl();
        // moveHangTo();
        // laDiablaControl();
        // bowlModeControl();
        vertControl();
        // screenControl();
        // underBarrierControl();
        // hangFireControl();
		// cataControl();
        hangControl();
		pros::delay(20);
	}
}
