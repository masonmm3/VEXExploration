#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

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
void autonomous() {
	pros::Motor left_wheels (1);
	pros::Motor right_wheels (2);

	right_wheels.move_relative(1000, 100);
	left_wheels.move_relative(1000, 100);
}

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
void opcontrol() {
	//create new motors
	pros::Motor left_wheels (1);
	pros::Motor right_wheels (10);

	//cant set ratio in declarator?
	pros::Motor arm (8);
	pros::Motor claw (3);

	arm.set_gearing(pros::E_MOTOR_GEARSET_36);
	claw.set_gearing(pros::E_MOTOR_GEARSET_36);

	pros::ADIDigitalIn left_bumper ('a');
	pros::ADIDigitalIn right_bumper ('b');
	pros::ADIDigitalIn arm_limit ('h');

	//create new controller
	pros::Controller master (CONTROLLER_MASTER);

	
	// while (true) {
	// 	left_wheels.move(master.get_analog(ANALOG_LEFT_Y)); //make sense
	// 	right_wheels.move(master.get_analog(ANALOG_RIGHT_Y));

	// 	pros::delay(2); //is in ms
	// }

	while (true) {
		//get controller inputs
		int power = master.get_analog(ANALOG_LEFT_Y);
		int steer = master.get_analog(ANALOG_RIGHT_X);

		//calc left and right
		int left = power - steer;
		int right = power + steer;

		//normalize inputs if too big
		if (left > 1 && left > right) {
			left /= left;
			right /= left;
		} else if (right > 1 && right > left) {
			right /= right;
			left /= left;
		}

		//check bumpers
		if (left_bumper.get_value() || right_bumper.get_value()) {
			// One of the bump switches is currently pressed
			if (left < 0) {
			  left = 0;
			}
			if (right < 0) {
			  right = 0;
			}
		  }

		//run motors
		left_wheels.move(left);
		right_wheels.move(right);

		// open loop arm controle
		if (master.get_digital(DIGITAL_R1)) {
			arm.move_velocity(100); //100 rpm motor
		} else if (master.get_digital(DIGITAL_R2) && !arm_limit.get_value()) {
			arm.move_velocity(100); //100 rpm motor
		} else {
			arm.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_L1)) {
			claw.move_velocity(100);
		  }
		  else if (master.get_digital(DIGITAL_L2)) {
			claw.move_velocity(-100);
		  }
		  else {
			claw.move_velocity(0);
		  }

		//timing display
		pros::delay(2);
	}
}