// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import javax.swing.event.MenuKeyEvent;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Intake.States;
import edu.wpi.first.wpilibj.Encoder;



/** Add your docs here. */
public class GrandmasWheelchair {

    private static GrandmasWheelchair instance;

    private double MAX_POWER=0.8;
    private double START_POWER=0.15;  //Minimum motor power to start drivetrain movement
    private double MOTOR_RAMP = .25;  // For closed and open loop
    private double SPEED_ACCEL = 0.025;  //Max acceleration per robot loop
    private double TURN_ACCEL = 0.04;   //Max acceleration per robot loop
    // Teleop variables
    private double teleopSpeed = 0.0;   //Velocity of fwd/back speed for teleop
    private double teleopTurn = 0.0;    //Velocity of turn rate for teleop
    // PID drive variables
    private double COUNTS_PER_INCH= 133191/120; //Needs to be calibrated
    //private double MIN_INCHES=0.5;
    //private double MAX_INCHES=24;
    private double TOLERANCE = 0.25; // number of inches of allowable difference to consider move complete.
    private double target = 0;       // Distance drive target
    //AUTO variables
    private double RAMP_INCHES = 100; //Number of inches from 0 to full power
    private double fastInches = 0;
    private double fastSpeed = 0; //Percent of full motor power for fast drive
    private double slowInches = 0;
    private double slowSpeed = 0; //Percent of power for slow drive
    private double currentSpeed = 0; //Current speed for distance drive
    private double currentTurn = 0; // Current turn for gyro
    private double encoderOffset = 0; // This is used to do a virtual encoder reset
    private double peakSpeed = 0; //This is the peak speed at the beginning of a ramp down.
    private double targetAngle = 0; //For gyro turns and heading hold
    private double STALL_TOLERANCE = 1; // velocity slower than this is considered a stall
    private int stallCycleCount = 0;
    private int STALL_CYCLE_LIMIT = 5; //number of cycles of no movement to be considered a stall
    private double error = 0 ; //Turn error calc
    private double kP = 1.0/25.0; //P term for turn rate

    private ADXRS450_Gyro gyro=new ADXRS450_Gyro(); 
    // Define motors
    private WPI_TalonSRX frontL = new WPI_TalonSRX(Constants.DRIVE_FL);
    private WPI_TalonSRX backL = new WPI_TalonSRX(Constants.DRIVE_BL);
    private WPI_TalonSRX frontR = new WPI_TalonSRX(Constants.DRIVE_FR);
    private WPI_TalonSRX backR = new WPI_TalonSRX(Constants.DRIVE_BR);
    private MotorControllerGroup left = new MotorControllerGroup(frontL, backL);
    private MotorControllerGroup right = new MotorControllerGroup(frontR, backR);

    DifferentialDrive drive = new DifferentialDrive(left, right);


    private enum States {IDLE, TELEOP, RAMP_UP, FAST_RUN, RAMP_DOWN, SLOW_RUN, TURNING}
    private States state = States.IDLE;
    
    public void init() {
        state=States.IDLE;
        resetEncoder();
        gyro.reset();
        frontL.configFactoryDefault();
        frontL.setNeutralMode(NeutralMode.Brake);
        frontL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        frontL.setInverted(false);
        frontL.configPeakOutputForward(MAX_POWER);
        frontL.configPeakOutputReverse(-MAX_POWER);
        frontL.configOpenloopRamp(MOTOR_RAMP);
        frontL.configClosedloopRamp(MOTOR_RAMP);            // Seconds from neutral to full power  (this is for PID control)
        frontL.config_kP(0,0.1);  //Set kP for PID0
        frontR.configFactoryDefault();
        frontR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        frontR.setNeutralMode(NeutralMode.Brake);
        frontR.configPeakOutputForward(MAX_POWER);
        frontR.setInverted(true);
        frontR.configPeakOutputReverse(-MAX_POWER);
        frontR.configOpenloopRamp(MOTOR_RAMP);
        frontR.configClosedloopRamp(MOTOR_RAMP);            // Seconds from neutral to full power  (this is for PID control)
        frontR.config_kP(0,0.1);  //Set kP for PID0
        backL.configFactoryDefault();
        backL.setNeutralMode(NeutralMode.Coast);
        backL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        backL.setInverted(false);
        backL.configPeakOutputForward(MAX_POWER);
        backL.configPeakOutputReverse(-MAX_POWER);
        backL.configOpenloopRamp(MOTOR_RAMP);
        backL.configClosedloopRamp(MOTOR_RAMP);            // Seconds from neutral to full power  (this is for PID control)
        backL.config_kP(0,0.1);  //Set kP for PID0
        backR.configFactoryDefault();
        backR.setInverted(true);
        backR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        backR.setNeutralMode(NeutralMode.Coast);
        backR.configPeakOutputForward(MAX_POWER);
        backR.configPeakOutputReverse(-MAX_POWER);
        backR.configOpenloopRamp(MOTOR_RAMP);
        backR.configClosedloopRamp(MOTOR_RAMP);            // Seconds from neutral to full power  (this is for PID control)
        backR.config_kP(0,0.1);  //Set kP for PID0
        //motor.config_kI(0,0.05);   //Set kI for PID0
        //motor.config_kD(0,0.05);   //Set kD for PID0
        //motor.config_kD(0,0.05);   //Set kF for PID0
        stopMotor();
    }

    private void stopMotor() {
        backR.stopMotor();
        backL.stopMotor();
        frontR.stopMotor();
        frontL.stopMotor();
        
        //motor.set(ControlMode.PercentOutput, 0);
    }

    public void coast(){
        frontL.setNeutralMode(NeutralMode.Coast);
        frontR.setNeutralMode(NeutralMode.Coast);
    }

    public void brake(){
        frontL.setNeutralMode(NeutralMode.Brake);
        frontR.setNeutralMode(NeutralMode.Brake);
    }

    //public void Wheelchair() {
        //super(left, right);

    //}
    public boolean isComplete() {
        return (state == States.IDLE);
    }

    private void stallCheck(){
        if (state == States.SLOW_RUN){
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

    public double getVelocity(){
        return (frontL.getSelectedSensorVelocity(0)*10/COUNTS_PER_INCH + frontR.getSelectedSensorVelocity(0)*10/COUNTS_PER_INCH) / 2;
    }
    
    public double getEncoderCounts() {
        return ((frontL.getSelectedSensorPosition(0) + frontR.getSelectedSensorPosition(0)) / 2);
    }

    public void resetEncoder(){
        encoderOffset = getEncoderCounts();
        //frontL.setSelectedSensorPosition(0);
        //frontR.setSelectedSensorPosition(0);
        //backL.setSelectedSensorPosition(0);
        //backR.setSelectedSensorPosition(0);
    }

    // Convert encoder counts (less the virtual offset) to Inches
    public double getInches(){
        return (getEncoderCounts() - encoderOffset)/COUNTS_PER_INCH;
    }
    
    public double accelerate(double currPower, double input, double maxAccel) {
        double diff = input - currPower;
        if (diff > 0) {
            if (currPower > -START_POWER && currPower < START_POWER) {  //Skip past deadzone of the motor, if below the START_POWER
                diff = START_POWER;
            } else {
                if (diff > maxAccel) {
                    diff = maxAccel;
                }
            }
        } else {
            if (currPower > -START_POWER && currPower < START_POWER) {  //Skip past deadzone of the motor, if below the START_POWER
                diff = -START_POWER;
            } else {
                if (diff < -maxAccel) {
                    diff = -maxAccel;
                }
            }   
        }

        return currPower + diff;
    }

    public void teleopDrive(double speed, double turn) {
        speed = Common.deadZone(speed, .15);
        turn = Common.deadZone(turn, .15);
        //Common.debug("teleop speed/turn " +String.valueOf(speed)+" "+String.valueOf(turn) + " currentSpeed " + String.valueOf(currentSpeed));
        if (speed != 0 || turn != 0) {
            state=States.TELEOP;
            teleopSpeed = accelerate(teleopSpeed, speed, SPEED_ACCEL);
            teleopTurn = accelerate(teleopTurn, turn, TURN_ACCEL);
        } else {
            if (state == States.TELEOP) {
                teleopSpeed  = accelerate(teleopSpeed, 0, SPEED_ACCEL);
                teleopTurn = accelerate(teleopTurn, 0, TURN_ACCEL);               
            }
        }
    }

    /* Fast Inches, Fast Speed, Slow Inches, Slow Speed */ 
    public void driveFwd(double fastInches, double fastSpeed, double slowInches, double slowSpeed) {
        state = States.RAMP_UP;
        resetEncoder();
        currentSpeed = 0;
        this.slowSpeed = slowSpeed;
        this.slowInches = slowInches;
        this.fastSpeed = fastSpeed;
        this.fastInches = fastInches;

    }

    public void driveBack(double fastInches, double fastSpeed, double slowInches, double slowSpeed) {
        state = States.RAMP_UP;
        resetEncoder();
        currentSpeed = 0;
        this.slowSpeed = -slowSpeed;
        this.slowInches = -slowInches;
        this.fastSpeed = -fastSpeed;
        this.fastInches = -fastInches;

    }


    public double rampDownPoint(double fastInches, double fastSpeed, double slowSpeed) {
        double distanceToSlowSpeed = RAMP_INCHES * slowSpeed;
        double remainingFastInches = fastInches - distanceToSlowSpeed;
        double distanceToFastSpeed = (fastSpeed - slowSpeed) * RAMP_INCHES;
        
        if (distanceToSlowSpeed > fastSpeed) { // Not enough distance to get to top speed, so accelerate to slow speed and stay there
            return distanceToSlowSpeed;
        }

        else if (remainingFastInches < distanceToFastSpeed * 2) { // Ramp down point when max speed cannot be held 
            return distanceToSlowSpeed + remainingFastInches / 2;
        }

        else {
            return fastInches - distanceToFastSpeed; // Maintains top speed, ramps down after a certain distance 
        }

    }

    public void turnTo(double angle){
        targetAngle = angle;
        state = States.TURNING;
    }

    //Calculate power to correct error in target angle from gyro
    public double calcTurnPower(){
        error = targetAngle - getAngle();
        double power = (0.2 * error * kP); // Was 0.23
        // Adjust for min power
        if (Math.abs(error) > 1){
            if (power < 0){
                power += -0.28; // Was -0.28
            } else {
                power += 0.28; // Was 0.28
            }
        } else {
            power = 0;
        }

        // Constrain the power so not to turn to fast
        power = Common.constrain(power, -0.5, 0.5); // Originally -0.6, 0.6
        return power;
    }

    /**
	 * Returns current angle.
	 * 
	 * @return double the current angle in degrees.
	 */
	public double getAngle() {
		return gyro.getAngle();
	}

    public void update() {
        stallCheck();
        switch(state){
            case RAMP_UP:
                currentSpeed = fastSpeed;
                state = States.FAST_RUN;
                currentTurn = calcTurnPower();   
                break;
            
            case FAST_RUN:
                //Common.debug("getInches:" + String.valueOf(getInches()) + "  fastInches: " + String.valueOf(fastInches));
                if (Math.abs(getInches()) >= Math.abs(fastInches)){
                    peakSpeed = currentSpeed;
                    Common.debug("Starting RAMP DOWN");

                    state = States.RAMP_DOWN;
                }
                currentTurn = calcTurnPower();  
                break;

            case RAMP_DOWN:
                currentSpeed = slowSpeed;
                stallCycleCount = 0;
                Common.debug("Starting SLOW RUN");
                state=States.SLOW_RUN;
                currentTurn = calcTurnPower();  
                break;

            case SLOW_RUN:
                if (Math.abs(getInches()) > Math.abs(fastInches + slowInches) || isStalled()) {
                    currentSpeed = 0;
                    Common.debug("SLOW RUN IDLING");
                    state = States.IDLE;
                }
                currentTurn = calcTurnPower();  
                break;

            case IDLE:
                currentSpeed = 0;
                currentTurn = 0;
                resetEncoder();
                break;

            case TELEOP:
                break;

            case TURNING:
                currentTurn = calcTurnPower();
                if (currentTurn == 0) {
                    Common.debug("TURN IDLING");
                    state = States.IDLE;
                }
                break;
          
        }
        
        if (state == States.TELEOP) {
            drive.arcadeDrive(teleopSpeed, teleopTurn);
        } else {
            drive.arcadeDrive(currentSpeed, currentTurn);
        }
                
        debug();
    }
    
    public void debug() {
        Common.dashNum("DT: teleopSpeed", teleopSpeed);
        Common.dashNum("DT: teleopTurn", teleopTurn);
        Common.dashNum("DT: Inches: ", getInches());
        Common.dashNum("DT: Encoder Counts: ", getEncoderCounts());
        Common.dashStr("DT: State: ", state.toString());
        Common.dashNum("DT: StallCount: ", stallCycleCount);
        Common.dashNum("DT: Current Speed: ", currentSpeed);
        Common.dashNum("DT: Current Turn: ", currentTurn);
        Common.dashNum("DT: Turn Error:", error);
        Common.dashNum("DT: Gyro angle:", getAngle());
        Common.dashNum("DT: targetAngle:", targetAngle);
    }


}

