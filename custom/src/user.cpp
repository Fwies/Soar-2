#include "vex.h"
#include "motor-control.h"
#include "driver-control.h"
#include "auton-selector.h"
#include "drive-log.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  stopChassis(hold);
  // Runs the routine picked on the Brain screen (or with the controller arrows while disabled).
  // Add routines to the list in runPreAutonomous() below.
  leftSide();
  //exampleAuton();
  //runSelectedAuton();
}
int clamp_lift_delay = 0;
// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;
bool clamp_lift = false;




void runDriver() {
  
  lift_thread.interrupt();
  
  stopChassis(coast);
  heading_correction = false;
  while (true) {

    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();


    // driver lift up and down
    if(lift_bumper.pressing()){
      lift.setPosition(0,rev);
    }
    if(clamp_lift_delay>15){
      clamp_lift=false;
      clamp_lift_delay=0;
      lift.spinToPosition(0.75,rev, 600, velocityUnits::rpm, false);
    }
    if(clamp_lift){
      clamp_lift_delay++;
    }
    //Brain.Screen.clearLine(0);
    //Brain.Screen.clearLine(1);
    
    //Brain.Screen.print("%d y: %d",x_pos, y_pos);
    
    if(l1 && clamp_optical.objectDistance(inches)<2){
      clamp_piston.set(false);
      lift.spinToPosition(3,rev,2,rpm,false);
      clamp_lift=true;
    }
    else if(r1){
      lift.spin(forward, 12, volt);
    }
    else if(r2&&!lift_bumper.pressing()){
      lift.spin(reverse, 10, volt);
    }
    else if (lift.isDone()){
      
      lift.stop();
    }

    
    if (l2){
      lift.spinToPosition(0,rev,450,rpm,false);
      clamp_piston.set(true);
    }


    if(button_y){
      roller.spin(forward, 12, volt);
    }
    else{
      roller.stop(brake);
    }

    // default tank drive or replace it with your preferred driver code here:
    driverTank(ch3, ch2);


    // Arcade on the left stick:           driverArcade(ch3, ch4);
    // Split arcade (left fwd, right turn): driverArcade(ch3, ch1);
    // Tune feel with driver_curve and driver_deadband in custom/src/robot-config.cpp

    wait(10, msec); 
  }
}

void runPreAutonomous() {
  //clamp_piston.set(true);
  lift.setStopping(hold);
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();
  
  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }
  controller_1.rumble(".");

  // odom tracking (picks the setup from using_horizontal_tracker / using_vertical_tracker)
  startOdometry();


  // Optional: MCL uses the distance sensors to keep odometry from drifting while driving.
  // Needs field-centered coordinates, so start each auton with setPose() at the real field position.
  //startMcl();

  // Autonomous routines for the selector, in the order they appear on the Brain screen
  //addAuton("tune", exampleAuton);
  addAuton("Drive Calibration", calibrateDrive);  // For the simulator (see README)
  startAutonSelector();
}
