// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController stick = new XboxController(0);
  GrandmasWheelchair dt = new GrandmasWheelchair();
  Intake intake = new Intake();
  MountainGoat climber = new MountainGoat(intake);
  Auto auto = new Auto(dt, intake);
  
  public boolean ballThrown = false; 
  
 /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    climber.init();
    intake.init();
    dt.init();
    auto.init();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
  
  }

  @Override
  public void autonomousInit() {
    climber.init();
    intake.init();
    dt.init();
    auto.init();
  }

  @Override
  public void autonomousPeriodic() {
    auto.update();
  }

  @Override
  public void teleopInit() {
    climber.init();
    intake.init();
    dt.init();
  }

  @Override
  public void teleopPeriodic() {

    if (stick.getRightStickButton()) {
      dt.resetEncoder();
    }

    //  Drive

    dt.teleopDrive(-stick.getLeftY(), stick.getLeftX()*.7);


    //if (stick.getXButtonPressed()){                       // Sample code for Competetion testing
      //intake.prepThrower2();
    //}

    

    //  Intake

    if (stick.getLeftBumperPressed()){
      if (!climber.isClimbing()) {
        intake.intakeDown();
      }
    }

    if (stick.getRightBumperPressed()){
      if (!climber.isClimbing()) {
        intake.intakeUp();
      }
      
    }

    if (stick.getAButtonPressed()) {
      intake.toggleIntake();
    }

    if (stick.getBButtonPressed()) {
      intake.dumpIntake();
    }

    if(stick.getLeftTriggerAxis() > 0.9){
      if (intake.isReadyToThrow()) {
        intake.throwBall();
        ballThrown = true;
      } else {
       if (!ballThrown) {
          intake.prepThrower();
        } 
      }
    } else {
      ballThrown = false;
    }

    //  Climber

    if(stick.getStartButtonPressed()){
      if (climber.isClimbComplete()) {
        climber.climbAgain();
      } else {
        climber.startClimb();
      }
    }
    
    if(stick.getRightTriggerAxis() > 0.9){
      climber.nextStep();
    }

    //Testing
    if (stick.getPOV() == 0) {
      //climber.armMove(27);
      dt.turnTo(0);
    }

    if (stick.getPOV() == 180) {
      //climber.armMove(-0.5);
      //climber.armMove(2);
      dt.turnTo(180);
    }

    if (stick.getPOV() == 90){
      //climber.elbowMove(10);
      dt.turnTo(90);
    }

    if (stick.getPOV() == 270) {
      //climber.elbowMove(30);
      dt.turnTo(270);
    }


    climber.update();
    intake.update();
    dt.update(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (stick.getPOV() == 270) {
      auto.setAuto("LOW_ONE");
    }

    if (stick.getPOV() == 0) {
      auto.setAuto("LOW_TWO");
    }

    if (stick.getPOV() == 90) {
      auto.setAuto("LOW_THREE");
    }

    if (stick.getPOV() == 180) {
      auto.setAuto("DUMP_ONE");
    }

    if (stick.getPOV() == 270 && stick.getYButton()) {
      auto.setAuto("HIGH_ONE");
    }

    if (stick.getPOV() == 0 && stick.getYButton()) {
      auto.setAuto("HIGH_TWO");
    }

    if (stick.getPOV() == 90 && stick.getYButton()) {
      auto.setAuto("HIGH_THREE");
    }

    dt.coast();
    climber.update();
    intake.update();
    dt.update();
    auto.debug();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

}