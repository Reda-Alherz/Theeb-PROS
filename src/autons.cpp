#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.01, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants]
  chassis.pid_swing_constants_set(6.0, 0.0, 50.0);           // Swing constants //(6,0,65)
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void khabbazDrive(){
  arm_motor.move_absolute(-90,127);
  
   piston.set_value(false);
   chassis.pid_drive_set(-11_in, DRIVE_SPEED, true);
   chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 55_deg, SWING_SPEED, 32);
   chassis.pid_wait();
   chassis.pid_drive_set(-9_in, DRIVE_SPEED, true);
   chassis.pid_wait();

   chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, -30);
   chassis.pid_wait();

   chassis.pid_drive_set(-4.5_in, 80, true);
   chassis.pid_wait();
   piston.set_value(true); /// mobile goal token
  pros::delay(500);

   chassis.pid_drive_set(32_in, 80, true);  
   chassis.pid_wait_until(3.25_in);
   intake.move(-127);
   chassis.pid_wait_until(29_in);
   piston.set_value(false); /// mobile goal left
   chassis.pid_wait_until(31_in);
   intake.move(0);
   chassis.pid_wait_quick_chain();
   

  //  to the color stake
   chassis.pid_drive_set(9_in, DRIVE_SPEED, true);
   chassis.pid_wait();
   chassis.pid_turn_set(-90_deg, TURN_SPEED);   
   chassis.pid_wait();
   pros::delay(12000); //wait slee5 to go away ///11000
   chassis.pid_drive_set(-53_in, 90, true);
   chassis.pid_wait();
   chassis.pid_drive_set(5_in, 60, true);
   chassis.pid_wait();
   chassis.pid_turn_set(-177_deg, TURN_SPEED);   
   chassis.pid_wait();
   chassis.pid_drive_set(-5.2_in, 60, true);
   chassis.pid_wait();


   intake.move(-127);
   pros::delay(2500);
   chassis.pid_drive_set(30_in,DRIVE_SPEED , true);
   chassis.pid_wait();
}

void skillsTest_1(){
  // get mobile goal
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  piston.set_value(true); /// mobile goal taken

  //score first ring
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_quick_chain(); // ring scored

  //score two rings more
  chassis.pid_drive_set(63_in, DRIVE_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_until(32_in);
  chassis.pid_speed_max_set(50); //to the corner
  chassis.pid_wait_quick_chain(); /// two rings are taken

  //to the corner
  chassis.pid_drive_set(-5_in, 50, true);
  intake.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  piston.set_value(false); /// mobile goal left
  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);
  chassis.pid_wait(); ////corner approved

  //to the wall goal
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(57_in, DRIVE_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_until(52_in);
  intake.move(0);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_quick_chain(); ///// wall goal scored

  //to the red in the middle
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(135, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(74_in, DRIVE_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_until(70_in);
  intake.move(0);
  chassis.pid_wait();// red in the robot

  //to the mobile goal
  chassis.pid_turn_set(-45, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  piston.set_value(true); //mobile goal taken

  //t
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_quick_chain(); ///// wall goal scored
  chassis.pid_turn_set(-135_deg, TURN_SPEED);

  chassis.pid_wait();
  chassis.pid_drive_set(67_in, 50, true);
  intake.move(-127);
  chassis.pid_wait_until(32_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  piston.set_value(false); //mobile goal left

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(57_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intake.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_quick_chain();

  //clamp
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(45_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(23_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  Climbclamp.set_value(true);
}
// . . .
// Make your own autonomous functions here!
// . . .