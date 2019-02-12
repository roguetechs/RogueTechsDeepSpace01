/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Code modified and produced by FRC team 6821 Rogue Techs                    */
/* for the 2019 season Destination: Deep Space.                               */
/* Code written by Ashley M. Farrell - Last modified 2/10/2019                */
/*----------------------------------------------------------------------------*/

// FOR PROGRAM TO WORK CTRE-Phoenix LIBRARY MUST BE DOWNLOADED
#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>  // Library to connect the code and the driver station
#include <frc/Joystick.h>  // Library for robot controllers
#include <frc/Solenoid.h>  // Library for pistons (solenoids)
#include <cameraserver/CameraServer.h>  // Library for the camera to work

#include "ctre/Phoenix.h"  // Library for motor controller

// ---------------------------------------------------------------
// Define Motors
// Define the motors to their variable names and corresponding ports on the robot
// ---------------------------------------------------------------

// Drive Motors
VictorSPX *leftMotorMaster = new VictorSPX(1);
VictorSPX *leftMotorSlave = new VictorSPX(2);
VictorSPX *rightMotorMaster = new VictorSPX(3);
VictorSPX *rightMotorSlave = new VictorSPX(4);
// Manipulator Motors
VictorSPX *clawGrip = new VictorSPX(5);
VictorSPX *clawRotate = new VictorSPX(6);
VictorSPX *elevator = new VictorSPX(7);
VictorSPX *elevatorSlave = new VictorSPX(8);
VictorSPX *pneuDrive = new VictorSPX(9);

// ---------------------------------------------------------------
// Define Pneumatics
// Define the pneumatics pistons to their variable names and corresponding ports on the robot
// ---------------------------------------------------------------

frc::Solenoid *frontPiston1 = new frc::Solenoid(0);
frc::Solenoid *frontPiston2 = new frc::Solenoid(1);
frc::Solenoid *backPiston1 = new frc::Solenoid(2);
frc::Solenoid *backPiston2 = new frc::Solenoid(3);

// ---------------------------------------------------------------
// Define Controllers
// Define which controllers (joysticks) will be the driver or co driver for control
// ---------------------------------------------------------------

frc::Joystick * driver = new frc::Joystick(0);
frc::Joystick * coDriver = new frc::Joystick(1);



void Robot::RobotInit() {
  // ---------------------------------------------------------------
  // Camera code - Call the cameras and tell them to display images to the dashboard
  // ---------------------------------------------------------------
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(1);

  // Initialization code
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // ---------------------------------------------------------------
  // Define motor followers
  // Stating that slave motors will follow master instructions
  // ---------------------------------------------------------------
  
  leftMotorSlave->Set(ControlMode::Follower, 1);
  rightMotorSlave->Set(ControlMode::Follower, 3);

  elevatorSlave->Set(ControlMode::Follower, 7);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  /*                    Driver Controller Code                    */
  
  // ---------------------------------------------------------------
  // Robot Drive code
  // Left and Right Axes
  // ---------------------------------------------------------------
  
  // Driving variables (Speed of the robot)
  double leftSpeed = driver->GetRawAxis(1);
  double rightSpeed = driver->GetRawAxis(5);

  // Drive the robot code using the joysticks on the driver controller
  leftMotorMaster->Set(ControlMode::PercentOutput, leftSpeed);
  rightMotorMaster->Set(ControlMode::PercentOutput, rightSpeed);



  /*                    CoDriver Controller Code                   */

  // ---------------------------------------------------------------
  // Elevator code
  // Button 3 to go up, Button 2 to go down
  // ---------------------------------------------------------------

  // Control Variables 
  bool elevatorUp = coDriver->GetRawButton(3);
  bool elevatorDown = coDriver->GetRawButton(2);
  // Variables that dictate the speed the elevator will travel at going up and down
  float upSpeed = 0.5;
  float downSpeed = -0.5;

  // Check to see if the button to go up is pressed, if it is, move the motors at the set speed
  if(elevatorUp){
    elevator->Set(ControlMode::PercentOutput, upSpeed);
  }  
  // If the elevator up button isn't pressed, check to see is the elevator down button is pressed
  // if it is, move the motors to go down at the set speed
  else if(elevatorDown){
    elevator->Set(ControlMode::PercentOutput, downSpeed);
  }  
  // If neither of the buttons are pressed, set the speed to 0 (stop the motors)
  else{
    elevator->Set(ControlMode::PercentOutput, 0);
  }  


  // ---------------------------------------------------------------
  // Grip code
  // Button 4 to open, Button 5 to close
  // ---------------------------------------------------------------

  // Control Variables 
  bool gripClose = coDriver->GetRawButton(4);
  bool gripOpen = coDriver->GetRawButton(5);
  // Reads the input given from the axis at the bottom of the controller
  float gripAxisInput = coDriver->GetRawAxis(2);
  // Uses the input (above) and uses a simple linear function to get the proper output for the speed control
  float gripSpeed = 0.5 * gripAxisInput + 0.5;

  // Check to see if the button to open is pressed, if it is, move the motors at the set speed
  if(gripOpen){
    clawGrip->Set(ControlMode::PercentOutput, gripSpeed);
  }  
  // If the grip open button isn't pressed, check to see is the grip closed button is pressed
  // if it is, move the motors to close at the set speed
  else if(gripClose){
    clawGrip->Set(ControlMode::PercentOutput, -gripSpeed);
  }  
  // If neither of the buttons are pressed, set the speed to 0 (stop the motors)
  else{
    clawGrip->Set(ControlMode::PercentOutput, 0);
  }  


  // ---------------------------------------------------------------
  // Rotate code
  // Y Axis to control
  // ---------------------------------------------------------------

  // Collects the data from the location of the joystick axis and slow it down
  float rotateSpeed = coDriver->GetRawAxis(1)*0.5;

  // Set the rotate mechanism to be rotate speed 
  clawRotate->Set(ControlMode::PercentOutput, rotateSpeed);
  
  /*  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/
} // End of Autonomous

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  
  /*                    Driver Controller Code                    */
  
  // ---------------------------------------------------------------
  // Robot Drive code
  // Left and Right Axes
  // ---------------------------------------------------------------
  
  // Driving variables (Speed of the robot)
  double leftSpeed = driver->GetRawAxis(1);
  double rightSpeed = driver->GetRawAxis(5);

  // Drive the robot code using the joysticks on the driver controller
  leftMotorMaster->Set(ControlMode::PercentOutput, leftSpeed);
  rightMotorMaster->Set(ControlMode::PercentOutput, rightSpeed);


  // ---------------------------------------------------------------
  // Pneumatic code
  // Start to bring them all down, Y for the front 2, A for the back 2
  // For more information on this code visit https://ashleyfarrell419.wixsite.com/mysite/post/rising-and-falling-edges
  // ---------------------------------------------------------------

  // Finding the status of the buttons used to control the pistons
  bool pistonsDown = driver->GetRawButton(8);
  bool frontPistonsUp = driver->GetRawButton(4);
  bool backPistonsUp = driver->GetRawButton(1);
  // Variables to help find the rising edge of the button (When the button is first pressed)
  bool pistonsDownRE;
  bool frontPistonsRE;
  bool backPistonsRE;

  // Check for the rising edge of the piston down button.  If it has been pressed, extend the pistons
  if((!pistonsDownRE)&&(pistonsDown)){
    frontPiston1->Set(true);
    frontPiston2->Set(true);
    backPiston1->Set(true);
    backPiston2->Set(true);
  }
  // Check for the rising edge of the front piston up button.  If it is pressed, retract the front pistons
  if((!frontPistonsRE)&&(frontPistonsUp)){
    frontPiston1->Set(false);
    frontPiston2->Set(false);
  }
  // Check for the rising edge of the back piston up button.  If it is pressed, retract the back pistons 
  if((!backPistonsRE)&&(backPistonsUp)){
    backPiston1->Set(false);
    backPiston2->Set(false);
  }

  // Set the rising edge variables to be equal to the state of their corresponding buttons
  // These variables MUST be after the if statements because they must be one run behind the button states
  pistonsDownRE = pistonsDown;
  frontPistonsRE = frontPistonsUp;
  backPistonsRE = frontPistonsUp;


  // ---------------------------------------------------------------
  // Drive With Pistons Extended code
  // Right Trigger
  // ---------------------------------------------------------------

  // Find the pressure in which the trigger is being held and set as speed
  float pistonDriveSpeed = driver->GetRawAxis(3);

  // The motor is controlled at the speed given
  pneuDrive->Set(ControlMode::PercentOutput, pistonDriveSpeed);



  /*                    CoDriver Controller Code                   */

  // ---------------------------------------------------------------
  // Elevator code
  // Button 3 to go up, Button 2 to go down
  // ---------------------------------------------------------------

  // Control Variables 
  bool elevatorUp = coDriver->GetRawButton(3);
  bool elevatorDown = coDriver->GetRawButton(2);
  // Variables that dictate the speed the elevator will travel at going up and down
  float upSpeed = 0.5;
  float downSpeed = -0.5;

  // Check to see if the button to go up is pressed, if it is, move the motors at the set speed
  if(elevatorUp){
    elevator->Set(ControlMode::PercentOutput, upSpeed);
  }  
  // If the elevator up button isn't pressed, check to see is the elevator down button is pressed
  // if it is, move the motors to go down at the set speed
  else if(elevatorDown){
    elevator->Set(ControlMode::PercentOutput, downSpeed);
  }  
  // If neither of the buttons are pressed, set the speed to 0 (stop the motors)
  else{
    elevator->Set(ControlMode::PercentOutput, 0);
  }  


  // ---------------------------------------------------------------
  // Grip code
  // Button 4 to open, Button 5 to close
  // ---------------------------------------------------------------

  // Control Variables 
  bool gripClose = coDriver->GetRawButton(4);
  bool gripOpen = coDriver->GetRawButton(5);
  // Reads the input given from the axis at the bottom of the controller
  float gripAxisInput = coDriver->GetRawAxis(2);
  // Uses the input (above) and uses a simple linear function to get the proper output for the speed control
  float gripSpeed = 0.5 * gripAxisInput + 0.5;

  // Check to see if the button to open is pressed, if it is, move the motors at the set speed
  if(gripOpen){
    clawGrip->Set(ControlMode::PercentOutput, gripSpeed);
  }  
  // If the grip open button isn't pressed, check to see is the grip closed button is pressed
  // if it is, move the motors to close at the set speed
  else if(gripClose){
    clawGrip->Set(ControlMode::PercentOutput, -gripSpeed);
  }  
  // If neither of the buttons are pressed, set the speed to 0 (stop the motors)
  else{
    clawGrip->Set(ControlMode::PercentOutput, 0);
  }  


  // ---------------------------------------------------------------
  // Rotate code
  // Y Axis to control
  // ---------------------------------------------------------------

  // Collects the data from the location of the joystick axis and slow it down
  float rotateSpeed = coDriver->GetRawAxis(1)*0.5;

  // Set the rotate mechanism to be rotate speed 
  clawRotate->Set(ControlMode::PercentOutput, rotateSpeed);
}  // End of Tele-operated

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif