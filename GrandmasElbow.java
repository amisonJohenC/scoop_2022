// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class GrandmasElbow {
    private TalonSRX motor = new TalonSRX(Constants.CAN_ELBOW);
    private double MAX_POWER = 0.8; //Was 0.65
    private double RAMP_TIME = 0.25; 
    private double COUNTS_PER_DEGREE = 62000/30; // Calibrated for 30 degrees
    private double MIN_DEGREES = 0;
    private double MAX_DEGREES = 30;
    private double TOLERANCE = 1; // number of degrees of allowable difference to consider move complete.
    private double STALL_TOLERANCE = 0.8; // velocity slower than this is considered a stall Was 1.0
    private double target = MIN_DEGREES; 
    private int stallCycleCount = 0;
    private int STALL_CYCLE_LIMIT = 7; //number of cycles of no movement to be considered a stall
    
    private DigitalInput elbowLimit = new DigitalInput(Constants.DIO_ELBOW_LIMIT);

    private enum States {INIT, IDLE, MOVING, HOLDING}
    private States state = States.INIT;


    public void init() {
        state=States.INIT;
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.setSensorPhase(true);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configPeakOutputForward(MAX_POWER);
        motor.configPeakOutputReverse(-MAX_POWER);
        motor.configClosedloopRamp(RAMP_TIME);            // Seconds from neutral to full power  (this is for PID control)
        motor.configForwardSoftLimitThreshold(degreesToEncoder(MAX_DEGREES));
        motor.configReverseSoftLimitThreshold(degreesToEncoder(MIN_DEGREES));
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        motor.config_kP(0,0.02);  //Set kP for PID0
        //motor.config_kI(0,0.05);   //Set kI for PID0
        //motor.config_kD(0,0.05);   //Set kD for PID0
        //motor.config_kD(0,0.05);   //Set kF for PID0
        stopMotor();
    }

    public void rotateTo(double degrees){
       if (state != States.INIT){
           setTarget(degrees);
           state=States.MOVING;
       }
       
    }

    private void stallCheck(){
        if (state == States.MOVING || state == States.INIT){
            if (Math.abs(getVelocity()) < STALL_TOLERANCE){
                stallCycleCount += 1;
            } else {
                stallCycleCount = 0;
            }
        } else {
            stallCycleCount = 0;
        }
    }

    private boolean isStalled(){
        return stallCycleCount > STALL_CYCLE_LIMIT;
    }

    public void update(){
        stallCheck();
        switch (state) {
            case INIT:
                if (atLimit() && isStalled()){
                    stopMotor();
                    resetEncoder();
                    motor.configReverseSoftLimitEnable(true);
                    stallCycleCount = 0;
                    rotateTo(0);
                    state=States.HOLDING;
                }else{
                    motor.configReverseSoftLimitEnable(false);
                    motor.set(ControlMode.PercentOutput, -0.28);
                }
                break;
            
            case IDLE:
                stopMotor();
                break;
            
            case MOVING:
                //if (target < getDegrees() && atLimit()){ //If we hit the limit switch and our target is still below us then stop the motor.
                //    stopMotor();
                //    state=States.IDLE;
                //} else {
                    double error=Math.abs(target-getDegrees());
                    if (error<TOLERANCE || isStalled()){
                        if (isStalled()){
                            Common.debug("Elbow: Move completed because of a stall");
                        }
                        stallCycleCount = 0;
                        state=States.HOLDING;
                    }
                //}    
                break;  

            case HOLDING:
                if (target == 0) {
                    if(atLimit() == false){
                        motor.set(ControlMode.PercentOutput, -0.2);  //Was -.09
                    }else{
                        motor.set(ControlMode.PercentOutput, -0.09);  //Was -.05
                    }
                }
                break;
        }
        debug();
    }


    //returns true if the elbow switch is pressed
    private boolean atLimit(){
        return !elbowLimit.get();
    }

    //Elbow has finished moving and is now holding.
    public boolean elbowIsComplete(){
        return (state == States.HOLDING || state == States.IDLE); 
    }

    public boolean isReady(){
        return state != States.INIT;
    }

    private void resetEncoder(){
        motor.setSelectedSensorPosition(0);
    }

    public double getRawEncoder(){
        return motor.getSelectedSensorPosition(0);
    }


    public double getDegrees(){
        return motor.getSelectedSensorPosition(0)/COUNTS_PER_DEGREE; //CHECK
    }

    //This will return inch per second
    public double getVelocity(){
        return motor.getSelectedSensorVelocity(0)*10/COUNTS_PER_DEGREE;
    }

    //Enables the PID and sets target
    private void setTarget(double degrees){
        target = Common.constrain(degrees, MIN_DEGREES, MAX_DEGREES);
        motor.set(ControlMode.Position, degreesToEncoder(target));
    }

    private double degreesToEncoder(double degrees){
        return degrees * COUNTS_PER_DEGREE;
    }

    //Force stops the motor. Takes it out of PID mode and into percentage mode.
    private void stopMotor(){
        motor.set(ControlMode.PercentOutput, 0);
        stallCycleCount = 0;
    }

    public void debug(){
        Common.dashNum("Elbow: Target", target);
       // Common.dashNum("Elbow: Encoder Count", getRawEncoder());
        Common.dashNum("Elbow: Degrees", getDegrees());
       // Common.dashNum("Elbow: Velocity", getVelocity());
       // Common.dashNum("Elbow: Stall Cycle Count", stallCycleCount);
        Common.dashStr("Elbow: State", state.toString());
        Common.dashBool("Elbow: At Limit", atLimit());
    }
            
}




