package frc.robot;

import java.security.spec.EllipticCurve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;

public class MountainGoat{
    private enum States {INIT, IDLE, STEP1, STEP2, STEP2B, STEP3, STEP4, STEP5, STEP6, STEP7, STEP8, STEP9, STEP9B, STEP9C, STEP10, STEP11}
    private States state = States.INIT;

    private GrandmasArm arm = new GrandmasArm();
    private GrandmasElbow elbow = new GrandmasElbow();
    private Intake intake;

    private boolean SAFE_MODE = true; //When in safe mode, climber pauses between steps.
    private boolean pause = false;  //Controls pauses between steps

    
    private double HEIGHT_OFFSET = 0; //Test robot has arm mounted 12.5 inches to high, should be 0 for real robot
    private double FULL_RETRACTION = -1.25;  //Arm full retraction pos for lifting robot to latched position

    private int moveCounter = 1;  //Climb counter.  Need to do two passes for full climb.

    private long timer = 0; 
    
    public MountainGoat(Intake intake){
        this.intake = intake;
    }

    public void init(){
        state = States.INIT;
        elbow.init();
        arm.init();
        moveCounter = 1;
    }

    public void armMove(double inches){
        arm.moveTo(inches);
    }

    public void elbowMove(double degrees){
        elbow.rotateTo(degrees);
    }

    public void startClimb() {
        if (state == States.IDLE){
            if (SAFE_MODE == true){
                pause = true;
            }else{
                pause = false;
            } 
            state = States.STEP1;
        } else {
            if (state == States.STEP2 || state == States.STEP2B || state == States.STEP3){
                arm.armSpeed(.5);
                arm.moveTo(0);
                elbow.rotateTo(0); // was 0
                state = States.IDLE;
            }
        }
    
    }

    // If the climb failed to reach traversal, but we completed to Step 11, reattempt the climb to Traversal
    public void climbAgain() {
        if (state == States.STEP11) {
            state = States.STEP4;
        }
    }

    public boolean isClimbing() {
        if (state == States.IDLE || state == States.INIT){
            return false;
        } else {
            return true;
        }
    }

    public boolean isClimbComplete() {
        return state == States.STEP11; 
    }

    public void nextStep(){
        pause = false;
    }    

    public void update(){
        switch(state){
            case INIT:
                if (arm.isReady() && elbow.isReady()){
                    state = States.IDLE;
                }
                break;

            case IDLE:
                break;


            case STEP1: //MOVE TO BELOW THE LOW BAR
                arm.armSpeed(1);
                arm.moveTo(20.0 - HEIGHT_OFFSET);  //was 22
                elbow.rotateTo(0);
                intake.rotateTo(40);
                state = States.STEP2;
                break;

            case STEP2: //Move elbow ahead of bar
                if (arm.armIsComplete() && elbow.elbowIsComplete()){
                    if (!pause){
                        Common.debug("Starting STEP2B");
                        elbow.rotateTo(25);
                        state = States.STEP2B;
                    }
                }
                break;

            case STEP2B: //Extend above bar
                if (elbow.elbowIsComplete()){
                    if (!pause){
                        Common.debug("Starting STEP3");
                        arm.moveTo(31 - HEIGHT_OFFSET); //Needs to check
                        state = States.STEP3;
                    }
                }
                break;

            case STEP3: //Elbow into bar
                if (arm.armIsComplete() && elbow.elbowIsComplete()){
                    if (!pause){
                        Common.debug("Starting STEP4");
                        elbow.rotateTo(0);
                        arm.armSpeed(1);
                        state = States.STEP4;
                    }
                }
                break;
            
            case STEP4:  //Lift robot
                if (elbow.elbowIsComplete()){
                    if (!pause){
                        Common.debug("Starting STEP5");
                        arm.moveTo(FULL_RETRACTION);
                        state= States.STEP5;
                    }
                    
                }
                break;

            case STEP5:  //Release arm from bar
                if (arm.armIsComplete()){
                    if (!pause){
                        Common.debug("Starting STEP6");
                        arm.moveTo(6);
                        state = States.STEP6;
                    }
                }
                break;

            case STEP6:  //Reach way forward
                if (arm.armIsComplete() == true){
                    if (!pause){
                        Common.debug("Starting STEP7");
                        intake.rotateTo(40);
                        elbow.rotateTo(22);  //was 20
                        state = States.STEP7;
                    }
                }
                break;

            case STEP7:  //Extend arm
                if (elbow.elbowIsComplete() == true){
                    if (!pause){
                        Common.debug("Starting STEP8");
                        arm.moveTo(31); 
                        state = States.STEP8;
                    }
                }
                break;

            case STEP8:  //Elbow until stalled
                if (arm.armIsComplete() == true){
                    if (!pause){
                        Common.debug("Starting STEP9");
                        elbow.rotateTo(0);
                        state = States.STEP9;
                    }
                }
                break;

            case STEP9:  //Pull down and latch on 
                if (elbow.elbowIsComplete() == true){
                    if (!pause){
                        if (moveCounter == 1) { 
                            Common.debug("Starting STEP10 - Partial Retraction - High bar");
                            //arm.moveTo(24);
                            //state = States.STEP9B;
                            arm.moveTo(FULL_RETRACTION);
                            state = States.STEP10;
                        } else {
                            Common.debug("Starting STEP10 - Partial Retraction - Traverse");
                            arm.moveTo(15);  
                            state = States.STEP10;  
                        }
                        intake.intakeUp();
                    }
                }
                break;
            
            case STEP9B:    //Waiting for swinging to slow down
                if (arm.armIsComplete())  {
                    timer = Common.time();
                    state = States.STEP9C;
                }
                break;

            case STEP9C:    //Complete retraction
                if (Common.time() >= timer + 1500) {
                    arm.moveTo(FULL_RETRACTION);
                    state = States.STEP10;   
                }             
                break;

            case STEP10:
                if (arm.armIsComplete()){
                    if (moveCounter == 1){
                        Common.debug("Starting STEP5");
                        state = States.STEP5;
                        moveCounter = 2;
                    }else{
                        Common.debug("Starting STEP11");
                        state = States.STEP11;
                    }
                }
                break;

            case STEP11:
                arm.setToIdle();
                break;

        }//end of switch
        if (SAFE_MODE){
            pause = true;
        }
        arm.update();
        elbow.update();
        debug();
    }
    
    private void debug(){
        Common.dashStr("Climber: State", state.toString());
    }

}
