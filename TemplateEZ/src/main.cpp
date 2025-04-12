#include "main.h"

//Pre initialize chasis values
ez::Drive chassis(
	{1,2,3}, //Left drive ports
	{-4,-5,-6}, //Right drive ports (- ports revers it)
	7, //IMU port
	4.125, //Wheel Diameter (4 is actually 4.125)
	343.0 //Wheel RPM = cartridge * (motor gear / wheel gear)
);

ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(-9, 2.75, 4.0);  // This tracking wheel is parallel to the drive wheels
//end chasis initialization


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::delay(500);

	chassis.opcontrol_curve_buttons_toggle(true);
	chassis.opcontrol_drive_activebrake_set(0.0);
	chassis.opcontrol_curve_default_set(0.0,0.0);

	default_constants();

	ez::as::auton_selector.autons_add({
	});

	chassis.initialize();
	ez::as::initialize();

	chassis.odom_tracker_back_set(&horiz_tracker);
	chassis.odom_tracker_left_set(&vert_tracker);
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
	chassis.pid_targets_reset();                // Resets PID targets to 0
	chassis.drive_imu_reset();                  // Reset gyro position to 0
	chassis.drive_sensor_reset();               // Reset drive sensors to 0
	chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

	ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

void ez_template_extras() {
	// Only run this when not connected to a competition switch
	if (!pros::competition::is_connected()) {
	  // PID Tuner
	  // - after you find values that you're happy with, you'll have to set them in auton.cpp
  
	  // Enable / Disable PID Tuner
	  //  When enabled:
	  //  * use A and Y to increment / decrement the constants
	  //  * use the arrow keys to navigate the constants
	  if (master.get_digital_new_press(DIGITAL_X))
		chassis.pid_tuner_toggle();
  
	  // Trigger the selected autonomous routine
	  if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
		pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
		autonomous();
		chassis.drive_brake_set(preference);
	  }
  
	  // Allow PID Tuner to iterate
	  chassis.pid_tuner_iterate();
	}
  
	// Disable PID Tuner when connected to a comp switch
	else {
	  if (chassis.pid_tuner_enabled())
		chassis.pid_tuner_disable();
	}
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
	chassis.opcontrol_arcade_scaling(true);
while (true) {
	chassis.opcontrol_arcade_standard(ez::SPLIT);

	ez_template_extras();
}
}