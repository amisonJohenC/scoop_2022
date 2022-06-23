package frc.robot;

import javax.swing.event.MenuKeyEvent;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;                         
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Encoder;


public class Intake {

    private TalonFX motor_neck = new TalonFX(Constants.CAN_NECK);
    private TalonSRX motor_upper_mouth = new TalonSRX(Constants.CAN_MOUTH);
    private PWMSparkMax high_motor = new PWMSparkMax(Constants.PWM_THROWER);
    private double NECK_UP_POWER = -1.0; //was -0.3, pre comp was -0.68
    private double NECK_DOWN_POWER = 1.0;  //was 0.4, pre comp was 0.8
    private double MOUTH_EAT_POWER = 0.85;
    private double MOUTH_SPIT_POWER = -1;
    private double THROW_POWER = .7; // was 0.7, then .65
    private double COUNTS_PER_DEGREE = 74600/60; //Needs to be calibrated (72300)
    private double MIN_DEGREES = 5;
    private double MAX_DEGREES = 60;
    private double Target = MIN_DEGREES;
    private double RAMP_TIME = 1;   //Ramp used for Neck PID
    private double TOLERANCE = 3.5; //Degrees
    private double STALL_TOLERANCE = 0.5; // velocity slower than this is considered a stall
    private int stallCycleCount = 0;
    private int STALL_CYCLE_LIMIT = 5; //number of cycles of no movement to be considered a stall

    private DigitalInput upperNeckLimit = new DigitalInput(Constants.DIO_UPPER_NECK_LIMIT);
    //private DigitalInput lowerNeckLimit = new DigitalInput(Constants.DIO_LOWER_NECK_LIMIT);
        
    public enum States {INIT_DOWN, INIT_UP, IDLE, HOLDING, MOVING, EATING, SPITTING, PREP1, PREP2, PREP3, READY_TO_THROW, START_THROW, THROWING, TWO_PREP1, TWO_PREP2, TWO_PREP3, START_THROW_2, START_THROW_TWO1, START_THROW_TWO2}
    private States stateNeck = States.INIT_DOWN;
    private States stateMouth = States.IDLE;
    private double spitTime = 0;  //start time of a ball spit

    public void init() {
        stateMouth=States.IDLE;
        motor_upper_mouth.configFactoryDefault();
        motor_upper_mouth.configPeakOutputForward(MOUTH_EAT_POWER);
        motor_upper_mouth.configPeakOutputReverse(MOUTH_SPIT_POWER);

        motor_upper_mouth.setInverted(false);

        high_motor.setInverted(true);
        
        stateNeck=States.INIT_DOWN;
        motor_neck.configFactoryDefault();
        motor_neck.setInverted(true);
        motor_neck.configPeakOutputForward(NECK_DOWN_POWER);
        motor_neck.configPeakOutputReverse(NECK_UP_POWER);
        motor_neck.setSensorPhase(false);
        motor_neck.setNeutralMode(NeutralMode.Brake);
        motor_neck.configClosedloopRamp(RAMP_TIME);            // Seconds from neutral to full power  (this is for PID control)
        motor_neck.configForwardSoftLimitThreshold(degreesToEncoder(MAX_DEGREES));
        motor_neck.configReverseSoftLimitThreshold(degreesToEncoder(MIN_DEGREES));
        motor_neck.configForwardSoftLimitEnable(true);
        motor_neck.configReverseSoftLimitEnable(true);
        motor_neck.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the absolute sensor for PID feedback on PID 0, timeout of 10ms.  TalonFX uses IntegratedSensor
        
        // PID constants
        // P term base is assuming full power 10 degrees from target, or 1/10 = 0.05.  
        motor_neck.config_kP(0,0.013);  //Set kP for PID0
        //motor_neck.config_kI(0,0.000001);   //Set kI for PID0
        //motor_neck.config_kD(0,0.0);   //Set kD for PID0
        stopNeckMotor();
        stopMouthMotor();
        stopHighMotor();
    }

    public void rotateTo(double degrees){
        if (isReady()) { //(stateNeck != States.INIT_DOWN && stateNeck != States.INIT_UP)
            setTarget(degrees);
            stateNeck=States.MOVING;
        }
    }

    private void stallCheck(){
        if (stateNeck == States.INIT_UP){
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

    private long timer = 0;

    public void update(){
        
        stallCheck();
        switch (stateNeck) {

            case INIT_DOWN:     //Pull away from upper limit switch first.
                if (atUpperLimit()){
                    motor_neck.configReverseSoftLimitEnable(false);
                    motor_neck.set(ControlMode.PercentOutput, 0.12);
                }else{
                    stateNeck=States.INIT_UP;
                }
                break;


            case INIT_UP:   //Raise up to upper limit
                if (atUpperLimit() && isStalled()){
                    motor_neck.configReverseSoftLimitEnable(true);
                    stopNeckMotor();
                    resetEncoder();
                    stallCycleCount = 0;
                    stateNeck=States.IDLE;
                }else{
                    motor_neck.configReverseSoftLimitEnable(false);
                    motor_neck.set(ControlMode.PercentOutput, -0.15);
                }
                break;
            
            case IDLE:
                stopNeckMotor();
                break;

            case MOVING:
                if (Target < getDegrees() && atUpperLimit()){ //If we hit the upper limit switch then stop the motor.
                    stopNeckMotor();
                    stateNeck=States.INIT_DOWN;
                } else {
                    double error=Math.abs(Target-getDegrees());
                    if (error < TOLERANCE){
                        stateNeck = States.HOLDING;
                    }
                }
                break;

            case HOLDING:
                if (atUpperLimit()){
                    stopNeckMotor();
                    stateNeck = States.INIT_DOWN;
                }
                break; 

        }

        switch (stateMouth) {
            case IDLE:
                stopMouthMotor();
                stopHighMotor();
                break;

            case EATING:
                motor_upper_mouth.set(ControlMode.PercentOutput, MOUTH_EAT_POWER);
                high_motor.set(-0.3); // Was 0.2, changed at DCMP to keep balls in the intake
                break;

            case SPITTING:
                motor_upper_mouth.set(ControlMode.PercentOutput, MOUTH_SPIT_POWER);
                if (Common.time() > spitTime+600) {
                    stateMouth = States.IDLE;
                }
                break;

            case PREP1:
                timer = Common.time();
                rotateTo(0);
                stopHighMotor();
                motor_upper_mouth.set(ControlMode.PercentOutput, -.35); // Originally -0.3
                stateMouth = States.PREP2;
                break;

            case PREP2:
                if (timer + 175 <= Common.time()) {
                    motor_upper_mouth.set(ControlMode.PercentOutput, -0.09); // Was -0.1 before DCMP, changed to -0.8 then to -0.9 to try and mitigate error
                    high_motor.set(THROW_POWER);
                    timer = Common.time();
                    stateMouth = States.PREP3;
                }
                break;

            case PREP3:
                if (timer + 600 <= Common.time()){
                    stateMouth=States.READY_TO_THROW;
                }
                break;


            case READY_TO_THROW:
                break;

            case START_THROW:
                timer = Common.time();
                motor_upper_mouth.set(ControlMode.PercentOutput, 0.9);
                stateMouth=States.THROWING;
                break;
            
            /*    
            case START_THROW_TWO1:                                   // Effectively same thing as START_THROW but slightly faster
                timer = Common.time();
                motor_upper_mouth.set(ControlMode.PercentOutput, 1); // Adjusted to 1 because of added friction of ball in the intake
                stateMouth=States.START_THROW_TWO2;
                break;

            case START_THROW_TWO2:                                   // Waits for second before shooting first ball, and changes speed on intake back to 0.9
                if (timer + 1000 <= Common.time()) {
                    timer = Common.time();  
                    motor_upper_mouth.set(ControlMode.PercentOutput, 0.9);
                    stateMouth=States.THROWING;                      // Uses THROWING to shoot second ball.
                }
                break;
            */
            
            case THROWING:
                if (timer + 1000 <= Common.time()) { // Originally 660
                    stateMouth=States.IDLE;
                }
                break;
            
            /*    
            case TWO_PREP1:
                timer = Common.time();
                rotateTo(0);
                stopHighMotor();
                motor_upper_mouth.set(ControlMode.PercentOutput, -.45); // Changed to -0.45 to counter added friction of 2nd ball in intake
                stateMouth = States.TWO_PREP2;
                break;
            
            case TWO_PREP2:
                if (timer + 200 <= Common.time()) {
                    motor_upper_mouth.set(ControlMode.PercentOutput, -0.1); // Might need to adjust to be a little higher, not sure
                    high_motor.set(1);                                      // Maxed because need power   
                    timer = Common.time();
                    stateMouth = States.TWO_PREP3;
                }
                break;

            case TWO_PREP3:
                if (timer + 600 <= Common.time()){
                    stateMouth=States.READY_TO_THROW;
                }
                break;
            */

        }
        debug();
    }

     //returns true if the elbow switch is pressed
     private boolean atUpperLimit(){
        return !upperNeckLimit.get();
    }


    /*
    private boolean atLowerLimit(){
        return !lowerNeckLimit.get();
    }
    */

    public void toggleIntake(){
        if(stateMouth == States.IDLE){
            stateMouth = States.EATING;
        } else if(stateMouth == States.EATING||stateMouth == States.PREP1||stateMouth == States.PREP2||stateMouth==States.PREP3||stateMouth == States.READY_TO_THROW) {
            stateMouth = States.IDLE;
        }
    }

    public void intakeDown() {
        rotateTo(60);
    }

    public void intakeUp() {
        rotateTo(10); // was 8
    }

    public void dumpIntake() {
        if(stateMouth == States.IDLE){
            spitTime=Common.time();
            stateMouth=States.SPITTING;
        }
    }
    
    //Neck has finished moving.
    public boolean isComplete() { 
        return (stateNeck == States.HOLDING || stateNeck == States.IDLE);     
    }


    public boolean isReady(){
        return (stateNeck != States.INIT_DOWN && stateNeck != States.INIT_UP);
    }

    public boolean isSpitting() {
        return stateMouth==States.SPITTING;
    }

    public boolean isReadyToThrow() {
        return stateMouth == States.READY_TO_THROW;
    }

    public void prepThrower(){
        if (stateMouth==States.IDLE||stateMouth==States.EATING){
            stateMouth=States.PREP1;
        }
    }

    /*
    public void prepThrower2() {
        if (stateMouth == States.IDLE || stateMouth == States.EATING) {
            stateMouth = States.TWO_PREP1;
        }
    }
    */

    public void throwBall(){
        if (stateMouth==States.READY_TO_THROW){
            stateMouth=States.START_THROW;
        }
    }

    public void throwTwoBalls() {
        if (stateMouth == States.READY_TO_THROW) {
            stateMouth = States.START_THROW_TWO1;
        }
    }

    public boolean throwIsComplete(){
        return stateMouth == States.IDLE;
    }


    private void resetEncoder(){
        motor_neck.setSelectedSensorPosition(0);
    }

    public double getRawEncoder(){
        return motor_neck.getSelectedSensorPosition(0);
    }
/*
    public void neckUp(){
        if (stateNeck != States.INIT){
            stateNeck = States.MOVING;
        }
    }

    public void neckDown(){
        if (stateNeck != States.INIT){
            stateNeck = States.MOVING;
        }
    }
*/
    
    public double getDegrees(){
        return motor_neck.getSelectedSensorPosition(0)/COUNTS_PER_DEGREE; //CHECK
    }

    //This will return inch per second
    public double getVelocity(){
        return motor_neck.getSelectedSensorVelocity(0)*10/COUNTS_PER_DEGREE;
    }

    //Enables the PID and sets target
    private void setTarget(double degrees){
        Target = Common.constrain(degrees, MIN_DEGREES, MAX_DEGREES);
        motor_neck.set(ControlMode.Position, degreesToEncoder(Target));
    }

    private double degreesToEncoder(double degrees){
        return degrees * COUNTS_PER_DEGREE;
    }
 

    //Force stops the motor. Takes it out of PID mode and into percentage mode.
    private void stopNeckMotor(){
        motor_neck.set(ControlMode.PercentOutput, 0);
        stallCycleCount = 0;
    }
   
    private void stopMouthMotor(){
        motor_upper_mouth.set(ControlMode.PercentOutput, 0);
    }

    private void stopHighMotor(){
        high_motor.set(0);
    }

    public void debug(){
        //Common.dashNum("Neck: Target", Target);
        Common.dashNum("Neck: Encoder Count", getRawEncoder());
        Common.dashNum("Neck: Degrees", getDegrees());
        //Common.dashNum("Neck: Velocity", getVelocity());
        //Common.dashStr("Mouth: State", stateMouth.toString());
        Common.dashStr("Neck: State", stateNeck.toString());
        Common.dashBool("Neck: At Upper Limit", atUpperLimit());
       // Common.dashNum("Neck: Stall Cycle Count", stallCycleCount);
    }
            
}
