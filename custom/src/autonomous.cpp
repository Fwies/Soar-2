#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"
#include "path-following.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Then add each routine to the auton selector with addAuton() in runPreAutonomous() (custom/src/user.cpp)
// Format: returnType functionName() { code }

void lift_thread_func(){
  while(true){
  if(lift_bumper.pressing()){
    lift.setPosition(0,rev);
    if(lift.isDone()){
        
      lift.stop();
    }
    
  }
  wait(10,msec);
}
  
}
thread lift_thread = thread(lift_thread_func);



void leftSide(){
//lift.spin(reverse, 10, volt);

  /*lift.setPosition(0,rev);
  setPose(11,-63,360-125);
  clamp_piston.set(true);
  lift.spinToPosition(1.5,rev,600,rpm,false);
  roller.setPosition(0,rev);
  roller.spinTo(10,rev,600,rpm,true);
  correct_angle = -125;
  driveTo(-12.5,2000);
  lift.spinToPosition(0.9,rev,600,rpm,true);
  clamp_piston.set(false);
  wait(0.2, sec);
  
  driveTo(8.5,3000);
  lift.spin(reverse, 10,volt);*/

  runAsync([]{ driveTo(-1,2000); });

  lift.setPosition(0,rev);
  lift.spin(forward,2,volt);
  setPose(8,-62,180);
  
 
  

  
  roller.setPosition(0,rev);
  roller.spinTo(8,rev,600,rpm,true);
  correct_angle = 180;
  moveToPoint(8,-52,-1,2000,true,5.5);
  lift.spinToPosition(1.8,rev,600,rpm,false);
  turnToAngle(270,3000);
  correct_angle = 270;
  runAsync([]{ driveTo(-20,2000,true,3);});
  wait(1,sec);
  lift.spinToPosition(0.8,rev,600,rpm,false);
  
  wait(0.4, sec);
  clamp_piston.set(true);
  wait(0.2,sec);
  moveToPoint(8,-51,1,3000,true,5);
  lift.spin(reverse, 10,volt);
  
  wait(0.2,sec);
  turnToAngle(180,1000,true);
  //turnToPoint(12,-36,-1,3000);
  
  if(resetPositionLeft()){
    controller_1.Screen.print("f");
  }
  if(resetPositionFront()){
    controller_1.Screen.print("b");
   
  }
  
  turnToPoint(22,-22,-1,3000);
  runAsync([]{ moveToPoint(22,-22,-1,3000, true,12); });
  waitUntilNear(22,-22,14);
  cancelMotion();
  runAsync([]{ moveToPoint(22,-22,-1,3000, true,1.5); });
  while(clamp_optical.objectDistance(inches)>2.25){
    wait(2, msec);
  }
  clamp_piston.set(false);
  lift.spinToPosition(3.4, rev, 600, rpm,false);
  waitForMotion();
  turnToPoint(23,-42,-1,3000);

  
  moveToPoint(23,-42,-1,3000,true);
  lift.spin(reverse,10,volt);
  wait(0.1,sec);
  clamp_piston.set(true);
  wait(0.2, sec);
  
  swing(90,1,1000);
  if(resetPositionRight()){
    controller_1.Screen.print("f");
  }
  if(resetPositionFront()){
    controller_1.Screen.print("b");
   
  }
  
  turnToPoint(44,-47,-1,2000);
  runAsync([]{  moveToPoint(44,-47,-1,3000,true);});
  waitUntilNear(44,-47,14);
  cancelMotion();
  runAsync([]{  moveToPoint(44,-47,-1,3000,true,1.5);});
  while(clamp_optical.objectDistance(inches)>2.25){
    wait(2, msec);
  }
  clamp_piston.set(false);
  lift.spinToPosition(4.5, rev, 600, rpm,false);
  cancelMotion();
  turnToPoint(26,-47,-1,3000);
  moveToPoint(26,-47,-1,3000);
  clamp_piston.set(true);
  lift.spin(reverse,10,volt);

  
  


  
}
void exampleAuton() {
  setPose(0,0,0);
  // Use this for tuning linear and turn pid
  driveTo(60, 3000);
  
    
  
  //turnToAngle(45,3000);
  //turnToAngle(135,3000);
  //turnToAngle(315,3000);
    
  
  
  //controller_1.rumble("...");
  driveTo(-60, 3000);
}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}

/*
 * asyncExample
 * Shows background motions: start a move, run mechanisms while it drives, and stop it early with a sensor.
 */
void asyncExample() {
  setPose(0, 0, 0);

  // Start driving in the background, then start the intake 12 inches into the move
  runAsync([]{ moveToPoint(0, 48, 1, 3000); });
  waitUntilTraveled(12);
  

  // Stop the move early once the intake sensor sees an object
  
  

  // Turn around and drive back, raising the arm once the turn is halfway done
  runAsync([]{
    turnToAngle(180, 1500);
    moveToPoint(0, 0, 1, 3000);
  });
  waitUntilTurned(90);
  
  waitForMotion();
  
}

/*
 * pathExample
 * Drives a smooth S-curve with pure pursuit, then backs along it with RAMSETE.
 * Paths use field coordinates, so the auton starts with setPose().
 */
void pathExample() {
  
}

/*
 * profiledExample
 * Straight drives and turns with motion profiles: they speed up and slow down smoothly.
 */
void profiledExample() {
  
}

double arm_pid_target = 0, arm_load_target = 60, arm_store_target = 250, arm_score_target = 470;

/*
 * armPID
 * Runs a single PID update for the arm motor to reach the specified target position.
 * - arm_target: Desired arm position (degrees).
 */
void armPID(double arm_target) {
 
}

/*
 * armPIDLoop
 * Continuously runs the arm PID control in a separate thread, keeping the arm at the target position.
 */
void armPIDLoop() {
 
}

/*
 * rushClamp
 * Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
 * Used for quickly grabbing a mobile goal at the start of autonomous.
 */
void rushClamp() {
    // Lower the rush arm
}

/*
 * intakeThread
 * Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
 * Used for picking up rings or other objects during autonomous.
 */
void intakeThread(){
                     // Stop intake motor and hold
}

/*
 * redGoalRush
 * 2024-2025 World Championship runner-up(1698V) autonomous routine.
 * This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
 * It uses multiple threads for simultaneous arm, clamp, and intake control.
 */
void redGoalRush() {
                             // Slow drive forward
}