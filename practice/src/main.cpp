#include "main.h"
#include "lemlib/api.hpp"

bool pistons = false;
bool speedToggled = false;
std::vector<int> overheat;
float speed = 0.75;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Imu imu(5); // IMU on port 5

pros::MotorGroup leftMotors({-11, -1, -2}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({20, 10, 9}, pros::MotorGearset::green);

pros::Motor top (-17);
pros::Motor bottom (-13);

pros::adi::DigitalOut front ('A'); // ADI port A

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
							  &rightMotors, // right motor group
							  12.5, // 12.5 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  333.33, // drivetrain rpm is 333.33
							  2 // horizontal drift is 2 (for now)
);
			// could definitely be better with the tracking wheels or other sensors, but we don't have the materials for that
lemlib::OdomSensors sensors(nullptr, // no vertical tracking wheel
						   nullptr, // no second vertical tracking wheel
						   nullptr, // no horizontal tracking wheel
						   nullptr, // no second horizontal tracking wheel
						   &imu); // IMU

						   // lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
											  0, // integral gain (kI)
											  3, // derivative gain (kD)
											  3, // anti windup
											  1, // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3, // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
											  0, // integral gain (kI)
											  10, // derivative gain (kD)
											  3, // anti windup
											  1, // small error range, in degrees
											  100, // small error range timeout, in milliseconds
											  3, // large error range, in degrees
											  500, // large error range timeout, in milliseconds
											  0 // maximum acceleration (slew)
); 
// controller stuff
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
									 10, // minimum output where drivetrain will move out of 127
									 1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
								  10, // minimum output where drivetrain will move out of 127
								  1.019 // expo curve gain
);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors, // odometry sensors
						&throttle_curve, 
						&steer_curve
);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // Initialize the LCD display
	
	leftMotors.set_gearing(pros::v5::MotorGears::green);
	rightMotors.set_gearing(pros::v5::MotorGears::green);
	leftMotors.set_encoder_units(pros::v5::MotorUnits::counts);
	rightMotors.set_encoder_units(pros::v5::MotorUnits::counts);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	

	chassis.calibrate(); // calibrate sensors
	// print position to brain screen
	pros::Task screen_task([&]() {
		while (true) {
			// print robot location to the brain screen
			pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
			pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			// delay to save resources
			pros::delay(20);
		}
	});

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
void competition_initialize() {
	controller.print (0,0, "Lock tf in");
}
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

ASSET(PLEASE_txt);
void autonomous() {
	// set chassis pose
	// chassis.setPose(-64.811, -14.496, 90);
	// top.move(127);
	// bottom.move(127);
	// pros::delay(250);
	// lookahead distance: 15 inches
	// timeout: 2000 ms
	// chassis.follow(PLEASE_txt, 15, 2000);
	// top.move(0);
	// bottom.move(0);
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
	//autonomous(); // run autonomous code for testing purposes
	int count = 0;
	
	
	while (true) {
		// get left y and right y positions
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // if left button is pressed
			top.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // if right button is pressed
			top.move(-127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			bottom.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			bottom.move(-127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			top.move(127);
			bottom.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			if (!speedToggled) {
				if (speed == 0.75) {
					speed = 1.0;
				}
				else	 {
					speed = 0.75;
				}
				speedToggled = true;
			}
		} else {
			top.move(0);
			bottom.move(0);
			speedToggled = false;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			pistons = !pistons;
			front.set_value(pistons);
		}
		
		// print temp to controller screen
		if (count % 25 == 0) {
			std::vector<double> leftTemps = leftMotors.get_temperature_all();
			std::vector<double> rightTemps = rightMotors.get_temperature_all();
			for (int i = 0; i < leftTemps.size(); i++) {
				if (leftTemps[i] > 55) {
					overheat.push_back(1);
				} else {
					overheat.push_back(0);
				}
				if (rightTemps[i] > 55)
				{
					overheat.push_back(1);
				} else {
					overheat.push_back(0);
				}
				
			}
			controller.print(0, 0, "%i %i %i %i %i %i  %.2f", overheat[0], overheat[1], overheat[2], overheat[3], overheat[4], overheat[5], speed);
		}
		count++;

		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * speed;
		int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * speed;
		// move the robot
		chassis.tank(leftY, rightY);

		// delay to save resources
		pros::delay(25);
	}
}
