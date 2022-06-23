
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;

/** Inital startup procedures
 * 1. Test limit switch polarity
 * 2. Check encoder output and direction
 * 3. Calibrate encoders to inches or degrees as appropriate
 * 4. Run init tests and invert motors if necessary
 * PID Testing
 * 5. Verify and set Min and Max inches or degrees
 * 6. Review and set tolerance
 * 7. Review and adjust max power for slow and safe
 * 8. Review and adjust Kp
 * 9. Enable robot and test PID movement
 * PID tuning
 * 10. Make it work
 * State Movement Tuning
 */
public class GrandmasArm {
    private TalonFX motor = new TalonFX(Constants.CAN_ARM);
    private double MAX_POWER = 1.0;    
    private double RAMP_TIME = 0.27;
    private double COUNTS_PER_INCH = 415553/31; // Calibrated at physical maximum
    private double MIN_INCHES = -1.25;
    private double MAX_INCHES = 31.5;
    private double TOLERANCE = 0.25; // number of inches of allowable difference to consider move complete.
    private double STALL_TOLERANCE = .3; // velocity slower than this is considered a stall
    private double target = MIN_INCHES; 
    private int stallCycleCount = 0;
    private int STALL_CYCLE_LIMIT = 5; //number of cycles of no movement to be considered a stall

    private enum States {INIT, IDLE, MOVING, HOLDING}
    private States state = States.INIT;

    private DigitalInput armLimit = new DigitalInput(Constants.DIO_ARM_LIMIT);


    public void init() {
        state=States.INIT;
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configPeakOutputForward(MAX_POWER);
        motor.configPeakOutputReverse(-MAX_POWER);
        motor.configClosedloopRamp(RAMP_TIME);            // Seconds from neutral to full power  (this is for PID control)
        motor.configForwardSoftLimitThreshold(inchesToEncoder(MAX_INCHES));
        motor.configReverseSoftLimitThreshold(inchesToEncoder(MIN_INCHES));
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the absolute sensor for PID feedback on PID 0, timeout of 10ms.  TalonFX uses IntegratedSensor
        motor.config_kP(0, 0.045);  //Set kP for PID0  was 0.04
        //motor.config_kI(0,0.00001);   //Set kI for PID0
        //motor.config_kD(0,0.05);   //Set kD for PID0
        //motor.config_kD(0,0.05);   //Set kF for PID0
        stopMotor();
    }

    public void moveTo(double inches){
       if (state != States.INIT){
           setTarget(inches);
           state=States.MOVING;
       }
       
    }

    public void setToIdle(){
        if (state == States.HOLDING){
            state = States.IDLE;
        }
    }

    private void stallCheck(){
        if (state == States.MOVING){
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
                if (atLimit()){
                    stopMotor();
                    resetEncoder();
                    motor.configReverseSoftLimitEnable(true);
                    state=States.IDLE;
                }else{
                    motor.configReverseSoftLimitEnable(false);
                    motor.set(ControlMode.PercentOutput, -0.2);
                }
                break;
            
            case IDLE:
                stopMotor();
                break;
            
            case MOVING:
                if (target < getInches()  && atLimit()){ //If we hit the limit switch and our target is still below us then stop the motor.
                    stopMotor();
                    state=States.IDLE;
                } else {
                    double error=Math.abs(target-getInches());
                    if (error<TOLERANCE || isStalled()){
                        stallCycleCount = 0;
                        state=States.HOLDING;
                    }
                }
                break;  

            case HOLDING:
                break;
        }
        debug();
    }


    //returns true if the arm switch is pressed
    private boolean atLimit(){
        return !armLimit.get();
    }
    //Set arm speed limit as a decimal percentage between 0 and 1
    public void armSpeed(double pctOfMax){
        pctOfMax = Common.constrain(pctOfMax, 0, 1);
        motor.configPeakOutputForward(MAX_POWER*pctOfMax);
        motor.configPeakOutputReverse(-MAX_POWER*pctOfMax);
    }

    //Arm has finished moving and is now holding.
    public boolean armIsComplete(){
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


    public double getInches(){
        return motor.getSelectedSensorPosition(0)/COUNTS_PER_INCH;
    }
    //This will return inch per second
    public double getVelocity(){
        return motor.getSelectedSensorVelocity(0)*10/COUNTS_PER_INCH;
    }

    //Enables the PID and sets target
    private void setTarget(double inches){
        target = Common.constrain(inches, MIN_INCHES, MAX_INCHES);
        motor.set(ControlMode.Position, inchesToEncoder(target));
    }

    private double inchesToEncoder(double inches){
        return inches * COUNTS_PER_INCH;
    }

    //Force stops the motor. Takes it out of PID mode and into percentage mode.
    private void stopMotor(){
        motor.set(ControlMode.PercentOutput, 0);
        stallCycleCount = 0;
    }

    public void debug(){
        Common.dashNum("Arm: Target", target);
        //Common.dashNum("Arm: Encoder Count", getRawEncoder());
        Common.dashNum("Arm: Position", getInches());
       // Common.dashNum("Arm: Velocity", getVelocity());
        //Common.dashNum("Arm: Stall Cycle Count", stallCycleCount);
        Common.dashStr("Arm: State", state.toString());
        Common.dashBool("Arm: At Limit", atLimit());
        Common.dashNum("Arm: Motor Power ", motor.getMotorOutputPercent());
    }
            
}




