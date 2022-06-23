package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;


public class Auto {

    GrandmasWheelchair dt;
    Intake intake;

    public String currentAuto = "one";


    private enum autoStates { 
        INIT,
        DUMP_BALL,
        SHOOT_BALL,
        LEAVE_TARMAC,
        READY_INTAKE,
        DRIVE,
        HIGH_DRIVE,
        TURN,
        LIFT,
        TURN_BACK,
        APPROACH,
        SCORE,
        FACE_FIRST_BALL,
        BALL_1,
        TURN_TO,
        BALL_2,
        FACE_GOAL,
        READY_INTAKE_2,
        ADJUST,
        TELEOP_SETUP,
        ARM_DOWN,
        DRIVE_BACK,
        PREP_TWO,
        SHOOT_TWO,
        IDLE;
    }

    private enum autoBallAmount {
        DUMP_ONE,
        HIGH_ONE,
        HIGH_TWO,
        HIGH_THREE,
        LOW_ONE,
        LOW_TWO,
        LOW_THREE;
    }

    private long timer = 0;

    private autoStates autoState = autoStates.INIT;
    private autoBallAmount autoChooser = autoBallAmount.DUMP_ONE;
    
    public Auto(GrandmasWheelchair dt, Intake intake){
        this.dt = dt;
        this.intake = intake;
    }

    public void init() {
        autoState = autoStates.INIT;
    }

    public void setAuto(String auto) {

        if (auto == "DUMP_ONE") {
          autoChooser = autoBallAmount.DUMP_ONE;
        }

        if (auto == "LOW_ONE") {
            autoChooser = autoBallAmount.LOW_ONE;
        }

        else if (auto == "LOW_TWO") {
            autoChooser = autoBallAmount.LOW_TWO;
        }

        else if (auto == "LOW_THREE") {
            autoChooser = autoBallAmount.LOW_THREE;
        }

        else if (auto == "HIGH_ONE") {
          autoChooser = autoBallAmount.HIGH_ONE;
        }

        else if (auto == "HIGH_TWO") {
          autoChooser = autoBallAmount.HIGH_TWO;
        }

        else if (auto == "HIGH_THREE") {
          autoChooser = autoBallAmount.HIGH_THREE;
        }
    }

    public String selectedAuto() {
      return autoChooser.toString();
    }

    public void dumpAuto() {
      switch (autoState) {
          case INIT: 
          //Common.debug("oneBallAuto: INIT");
          if (intake.isReady()) {
            intake.intakeUp();      
            autoState = autoStates.DUMP_BALL;
            }
            break;

          case DUMP_BALL:
            if (intake.isComplete()) { 
              intake.dumpIntake();
              autoState = autoStates.TURN;
            }  
            break;

          case IDLE: 
              if (dt.isComplete()) {}
              break;

      }
  }

    public void lowOneBallAuto() {
        switch (autoState) {
            case INIT: 
            //Common.debug("oneBallAuto: INIT");
            if (intake.isReady()) {
              intake.intakeUp();      
              autoState = autoStates.DUMP_BALL;
              }
              break;

            case DUMP_BALL:
              if (intake.isComplete()) { 
                intake.dumpIntake();
                autoState = autoStates.TURN;
              }  
              break;


            case TURN:
                //intake.intakeUp();
                //give the bot time to be fully ready to turn
                if (intake.isSpitting() == false) {
                  dt.turnTo(180);
                  timer = Common.time();
                  autoState = autoStates.LEAVE_TARMAC;
                }
                break;

            case LEAVE_TARMAC:               
                if (timer + 10000 <= Common.time()) {
                  dt.driveFwd(61, 0.55, 0, 0);
                  autoState = autoStates.IDLE;
                }
                break;

            case IDLE: 
                if (dt.isComplete()) {}
                break;

        }
    }

    public void lowTwoBallAuto() {
        switch(autoState) {
            case INIT: 
            if (intake.isReady()) {
              intake.intakeUp();
              Common.debug("Starting twoBallDump DUMP_BALL");      
              autoState = autoStates.DUMP_BALL;
              }
              break;

            case DUMP_BALL:
              if (intake.isComplete()) { 
                Common.debug("Starting TURN");
                intake.dumpIntake();
                autoState = autoStates.TURN;
              }  
              break;


            case TURN:
                //intake.intakeUp();
                //give the bot time to be fully ready to turn
                if (intake.isSpitting() == false) {
                  dt.turnTo(180);
                  Common.debug("Starting READY_INTAKE");
                  autoState = autoStates.READY_INTAKE;
                }
                break;

            case READY_INTAKE:
              if (dt.isComplete()) {
                intake.intakeDown();
                Common.debug("Starting DRIVE");
                autoState = autoStates.DRIVE;
              }
              break;
            
            case DRIVE:
                if (intake.isComplete()) {
                    intake.toggleIntake();
                    dt.driveFwd(59, 0.60, 17, 0.45);
                    Common.debug("Starting LIFT");
                    autoState = autoStates.LIFT; 
                }
                break;

            
            case LIFT:
                if (dt.isComplete()) {
                  intake.intakeUp();
                  Common.debug("Starting LEAVE_TARMAC");
                  autoState = autoStates.LEAVE_TARMAC;
                }
                break;

            case LEAVE_TARMAC: 
                if (intake.isComplete()) {
                  intake.toggleIntake();
                  Common.debug("Starting TURN_BACK");
                  dt.driveFwd(0, 0.45, 5, 0.45);
                  autoState = autoStates.TURN_BACK;
                }
                break;


            case TURN_BACK:
                if (dt.isComplete()) {
                  dt.turnTo(0);
                  Common.debug("Starting APPROACH");
                  autoState = autoStates.APPROACH;
                }
                break;

            case APPROACH:
                if (dt.isComplete()) {
                    dt.driveFwd(70, 0.60, 21, 0.45);  
                    Common.debug("Starting SCORE");
                    autoState = autoStates.SCORE;
                }
                break;

            case SCORE:
                if (dt.isComplete()) {
                    intake.dumpIntake();
                    Common.debug("TELEOP_SETUP");
                    autoState = autoStates.TELEOP_SETUP;
                }
                break;

            case TELEOP_SETUP:
                if(intake.isSpitting() != true){
                  dt.turnTo(250);
                  autoState = autoStates.IDLE;
                }
                break;

            case IDLE:
                if (dt.isComplete()) {}
                break;
                
            }
    }

    public void lowThreeBallAuto() {
        switch(autoState) {
            case INIT:
              intake.dumpIntake();
              if (intake.isReady()) {
                autoState = autoStates.FACE_FIRST_BALL;
              }
              break;

            /*
              case DUMP_BALL:
              
              autoState = autoStates.FACE_FIRST_BALL;
            */
          
            case FACE_FIRST_BALL:
                dt.turnTo(164.5);  // was 162
                autoState = autoStates.READY_INTAKE;
              break;
              
            case READY_INTAKE:
              if (dt.isComplete()) {
                intake.intakeDown();
                autoState = autoStates.BALL_1;
              }  
              break;
      
            case BALL_1:
              if (intake.isComplete()) {
                intake.toggleIntake();
                dt.driveFwd(64, 0.60, 11, 0.45);  //was 59, slow inches went from 17 to 11
                autoState = autoStates.TURN_TO;
              }
              break;
       
            case TURN_TO:
            if (dt.isComplete()) {
              dt.turnTo(271);  //was 274, prev was 285
              autoState = autoStates.BALL_2;
            }
            break;
      
            case BALL_2:
              if (dt.isComplete()) {
                dt.driveFwd(78, 0.75, 18, 0.45);
                autoState = autoStates.FACE_GOAL;
              } 
      
            case FACE_GOAL:
              if (dt.isComplete()) {
                intake.intakeUp();
                dt.turnTo(409); //Was 410, was 411, prev was 413
                autoState = autoStates.APPROACH;
              }
              break;
      
            case APPROACH:
              if (dt.isComplete()) {
                intake.toggleIntake();
                dt.driveFwd(71, 0.65, 24, 0.45);  //Was 75, prev was 72
                autoState = autoStates.ADJUST;
              }
              break; 
      
            case ADJUST:
              if (dt.isComplete()) {
                dt.turnTo(370);     //Was 385
                autoState = autoStates.SCORE;
              }
              break;
      
            case SCORE:
              if (dt.isComplete()) {
                intake.dumpIntake();
                autoState = autoStates.TELEOP_SETUP;
              }
              break;

            case TELEOP_SETUP:
              if(intake.isSpitting() != true){
                dt.turnTo(540);
                autoState = autoStates.IDLE;
              }
              break;
          
            /*
            case ARM_DOWN:
              if(dt.isComplete()) {
                intake.intakeDown();
                autoState = autoStates.IDLE;
              }
              break;
            */
            case IDLE:
              break;          
          }
        
              
    }

    public void highOneBallAuto() {
      switch (autoState) {
          //Common.debug("oneBallAuto: INIT");
          case INIT: 
          if (intake.isReady()) {
            intake.prepThrower();
            Common.debug("Starting highTwoBallDump DUMP_BALL");      
            autoState = autoStates.SHOOT_BALL;
            }
            break;

          case SHOOT_BALL:
            if (intake.isReadyToThrow()) { 
              intake.throwBall();
              timer = Common.time();
              Common.debug("shooting");
              autoState = autoStates.LEAVE_TARMAC;
            }  
            break;

          case LEAVE_TARMAC:               
              if (timer + 10000 <= Common.time()) {
                dt.driveFwd(72, 0.65, 0, 0);
                autoState = autoStates.IDLE;
              }
              break;

          case IDLE: 
              if (dt.isComplete()) {}
              break;

      }
  }

    public void highTwoBallAuto() {
      switch(autoState) {
          case INIT: 
          if (intake.isReady()) {
            intake.prepThrower();
            Common.debug("Starting highTwoBallDump DUMP_BALL");      
            autoState = autoStates.SHOOT_BALL;
            }
            break;

          case SHOOT_BALL:
            if (intake.isReadyToThrow()) { 
              intake.throwBall();
              Common.debug("Shooting");
              autoState = autoStates.HIGH_DRIVE;
            }  
            break;


          case HIGH_DRIVE:
              if (intake.throwIsComplete()) {
                  intake.toggleIntake();
                  intake.intakeDown();
                  dt.driveFwd(60, 0.50, 23, 0.45); // Was 66, 0.6, 17, 0.45 
                  Common.debug("Starting Drive to Ball");
                  autoState = autoStates.READY_INTAKE; 
              }
              break;

          case READY_INTAKE:
            if (dt.isComplete()) {
              intake.toggleIntake();
              Common.debug("Finishing Ball Intake");
              autoState = autoStates.DRIVE_BACK;
            }
            break;
          
          case DRIVE_BACK:
            if (intake.isComplete()) {
              dt.driveBack(59, 0.60, 24, 0.45); // Was 66 fast, 17 slow
              intake.intakeUp();
              Common.debug("Starting DRIVE_BACK");
              autoState = autoStates.PREP_TWO;
            }

          case PREP_TWO:
            if (dt.isComplete()) {
              intake.prepThrower();
              Common.debug("Prepping for SHOOT_TWO");
              autoState = autoStates.SHOOT_TWO;
            }
          case SHOOT_TWO:
              if (intake.isReadyToThrow()) {
                intake.throwBall();
                Common.debug("Shooting 2nd ball");
                autoState = autoStates.IDLE;
              }
              break;

          case IDLE:
              break;
              
          }
  }

  public void highThreeBallAuto() {
    switch(autoState) {
        case INIT: 
          if (intake.isReady()) {
            intake.prepThrower();
            Common.debug("Starting highThreeBallDump DUMP_BALL");      
            autoState = autoStates.SHOOT_BALL;
            }
            break;

          case SHOOT_BALL:
            if (intake.isReadyToThrow()) { 
              intake.throwBall();
              Common.debug("shooting");
              autoState = autoStates.FACE_FIRST_BALL;
            }  
            break;
      
        case FACE_FIRST_BALL:
            if (intake.isComplete()) {
              dt.turnTo(-15); // Originally -11.2 
              autoState = autoStates.READY_INTAKE;
            }
            break;
          
        case READY_INTAKE:
          if (dt.isComplete()) {
            intake.intakeDown();
            autoState = autoStates.BALL_1;
          }  
          break;
  
        case BALL_1:
          if (intake.isComplete()) {
            intake.toggleIntake();
            dt.driveFwd(66, 0.60, 11, 0.45);  //was 64, slow inches went from 17 to 11
            autoState = autoStates.TURN_TO;
          }
          break;
   
        case TURN_TO:
        if (dt.isComplete()) {
          dt.turnTo(88);  //was 271, subtracted 180 for face and adjusted from 91
          autoState = autoStates.BALL_2;
        }
        break;
  
        case BALL_2:
          if (dt.isComplete()) {
            dt.driveFwd(78, 0.75, 18, 0.45);
            autoState = autoStates.FACE_GOAL;
          } 
  
        case FACE_GOAL:
          if (dt.isComplete()) {
            intake.intakeUp();
            dt.turnTo(219); //Was 410, was 411, prev was 413, reduced by 180 to account for face change
            autoState = autoStates.APPROACH;
          }
          break;
  
        case APPROACH:
          if (dt.isComplete()) {
            intake.toggleIntake();
            dt.driveFwd(65, 0.65, 30, 0.45);  //Was 75, prev was 72 + moved 6 inches of fast to slow
            autoState = autoStates.ADJUST;
          }
          break; 
  
        case ADJUST:
          if (dt.isComplete()) {
            dt.turnTo(185);     //Was 370, face change adjustment 190 > 185
            autoState = autoStates.SCORE;
          }
          break;
  
        case SCORE:
          if (dt.isComplete()) {
            intake.dumpIntake();
            autoState = autoStates.TELEOP_SETUP;
          }
          break;

        case TELEOP_SETUP:
          if(intake.isSpitting() != true){
            dt.turnTo(360); // was 540, face change adjustment
            autoState = autoStates.IDLE;
          }
          break;

        /*
        case ARM_DOWN:
          if(dt.isComplete()) {
            intake.intakeDown();
            autoState = autoStates.IDLE;
          }
          break;
        */

        case IDLE:
          break;          
      }
    
          
}

  
              
    

    public void update() {
        switch (autoChooser) {
            case DUMP_ONE:
              dumpAuto();
              break;

            case LOW_ONE:
                lowOneBallAuto();
                break;

            case LOW_TWO:
                lowTwoBallAuto();
                break;

            case LOW_THREE:
                lowThreeBallAuto();
                break;

            case HIGH_ONE:
                highOneBallAuto();
                break;

            case HIGH_TWO:
                highTwoBallAuto();
                break;

            case HIGH_THREE:
                highThreeBallAuto();
                break;
        }
        dt.update();
        intake.update();
        debug();
    }

    public void debug(){
        Common.dashStr("Auto: Chooser", autoChooser.toString());
        Common.dashStr("Auto: State", autoState.toString());
    }

}