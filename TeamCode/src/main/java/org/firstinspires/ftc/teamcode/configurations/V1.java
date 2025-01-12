/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the robot second version (18th of january)
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class V1 extends Configuration {

    protected void initialize(){

        /* Moving configuration : Positive power makes wheel go forward */
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",false));      // CH Motor 0
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",false));        // CH Motor 1
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",true));     // CH Motor 2
        mMotors.put("back-right-wheel",new ConfMotor("backRight",true));       // CH Motor 3

        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        mImus.put("otos", new ConfImu("sensor_otos"));                                     // EH I2C 3

        /* Intake configuration */
        mMotors.put("intake-slides-left",new ConfMotor("intakeSlidesLeft",true));         // EH Motor 3
        mMotors.put("intake-slides-right",new ConfMotor("intakeSlidesRight",true));        // EH Motor 2
        mMotors.get("intake-slides-left").addPosition("min",0 );
        mMotors.get("intake-slides-right").addPosition("min",0 );
        mMotors.get("intake-slides-left").addPosition("transfer",175 );
        mMotors.get("intake-slides-right").addPosition("transfer",177 );
        mMotors.get("intake-slides-left").addPosition("max",358 );
        mMotors.get("intake-slides-right").addPosition("max",357 );
        mServos.put("intake-arm-pitch", new ConfServo(
                "intakeArmPitchLeft", false,                                                  // CH Servo 5     4bSPL
                "intakeArmPitchRight", true                                                                      // EH Servo 1     4bSPR
        ));
        mServos.put("intake-elbow-pitch", new ConfServo("intakeElbowPitch", false));         // EH Servo 0     CSPIn
        mServos.put("intake-wrist-roll", new ConfServo("intakeWristRoll", false));           // CH Servo 4     CSRoIn
        mServos.put("intake-claw", new ConfServo("intakeClaw", false));                      // EH Servo 2     CSIn

        /* Outtake configuration */
        mMotors.put("outtake-slides-left",new ConfMotor("outtakeSlidesLeft",false,true));         // EH Motor 0
        mMotors.put("outtake-slides-right",new ConfMotor("outtakeSlidesRight",true,true));        // EH Motor 1
        mMotors.get("outtake-slides-left").addPosition("min",0 );
        mMotors.get("outtake-slides-right").addPosition("min",0 );
        mMotors.get("outtake-slides-left").addPosition("transfer",0 );
        mMotors.get("outtake-slides-right").addPosition("transfer",0 );
        mMotors.get("outtake-slides-left").addPosition("max",4156 );
        mMotors.get("outtake-slides-right").addPosition("max",4156 );
        mMotors.get("outtake-slides-left").addPosition("highBasket",10 );
        mMotors.get("outtake-slides-right").addPosition("highBasket",10 );
        mMotors.get("outtake-slides-left").addPosition("lowBasket",10 );
        mMotors.get("outtake-slides-right").addPosition("lowBasket",10 );
        mMotors.get("outtake-slides-left").addPosition("lowSubmersible",10 );
        mMotors.get("outtake-slides-right").addPosition("lowSubmersible",10 );
        mMotors.get("outtake-slides-left").addPosition("highSubmersible",10 );
        mMotors.get("outtake-slides-right").addPosition("highSubmersible",10 );


        mServos.put("outtake-wrist-roll", new ConfServo("outtakeWristRoll", false));         // CH Servo 0     CSRoOut
        mServos.put("outtake-claw", new ConfServo("outtakeClaw", false));                    // CH Servo 1     CSOut
        mServos.put("outtake-elbow-pitch", new ConfServo(
                "outtakeElbowPitchLeft", false,                                                // CH Servo 2     CSPROut
                "outtakeElbowPitchRight", false)                                                      // CH Servo 3     CSPLOut
        );


        /* Intake servos reference positions */
        mServos.get("intake-arm-pitch").addPosition("transfer", 0.97);
        mServos.get("intake-arm-pitch").addPosition("overSub", 0.55);
        mServos.get("intake-arm-pitch").addPosition("look", 0.44);
        mServos.get("intake-arm-pitch").addPosition("grab", 0.39);
        mServos.get("intake-arm-pitch").addPosition("off", 0.97);


        mServos.get("intake-elbow-pitch").addPosition("transfer", 0.15);
        mServos.get("intake-elbow-pitch").addPosition("grab", 0.68);
        mServos.get("intake-elbow-pitch").addPosition("look", 0.70);
        mServos.get("intake-elbow-pitch").addPosition("overSub", 0.73);
        mServos.get("intake-elbow-pitch").addPosition("off", 0.15);

        mServos.get("intake-wrist-roll").addPosition("-2", 0.27);
        mServos.get("intake-wrist-roll").addPosition("-1", 0.335);
        mServos.get("intake-wrist-roll").addPosition("0", 0.405);
        mServos.get("intake-wrist-roll").addPosition("1", 0.47);
        mServos.get("intake-wrist-roll").addPosition("2", 0.54);
        mServos.get("intake-wrist-roll").addPosition("3", 0.605);
        mServos.get("intake-wrist-roll").addPosition("4", 0.675);
        mServos.get("intake-wrist-roll").addPosition("5", 0.74);
        mServos.get("intake-wrist-roll").addPosition("6", 0.82);

        mServos.get("intake-claw").addPosition("closed", 1.0);
        mServos.get("intake-claw").addPosition("microrelease", 0.98);
        mServos.get("intake-claw").addPosition("open", 0.62);

        mServos.get("outtake-wrist-roll").addPosition("-2", 0.0);
        mServos.get("outtake-wrist-roll").addPosition("-1", 0.0675);
        mServos.get("outtake-wrist-roll").addPosition("0", 0.135);
        mServos.get("outtake-wrist-roll").addPosition("1", 0.2075);
        mServos.get("outtake-wrist-roll").addPosition("2", 0.28);
        mServos.get("outtake-wrist-roll").addPosition("3", 0.35);
        mServos.get("outtake-wrist-roll").addPosition("4", 0.42);
        mServos.get("outtake-wrist-roll").addPosition("5", 0.465);
        mServos.get("outtake-wrist-roll").addPosition("6",  0.51);

        mServos.get("outtake-claw").addPosition("closed", 0.67);
        mServos.get("outtake-claw").addPosition("microrelease", 0.65);
        mServos.get("outtake-claw").addPosition("open", 0.31);

        mServos.get("outtake-elbow-pitch").addPosition("transfer", 0.11);
        mServos.get("outtake-elbow-pitch").addPosition("drop", 0.026);
        mServos.get("outtake-elbow-pitch").addPosition("off", 0.05);

    }

    protected void initializeTuning() {
        mSingleServos.put("intake-arm-left-pitch", new ConfServo("intakeArmPitchLeft", false));
        mSingleServos.put("intake-arm-right-pitch", new ConfServo("intakeArmPitchRight", true));
        mSingleServos.put("intake-elbow-pitch", new ConfServo("intakeElbowPitch", false));
        mSingleServos.put("intake-wrist-roll", new ConfServo("intakeWristRoll", false));
        mSingleServos.put("intake-claw", new ConfServo("intakeClaw", false));

        mSingleServos.put("outtake-wrist-roll", new ConfServo("outtakeWristRoll", false));
        mSingleServos.put("outtake-claw", new ConfServo("outtakeClaw", false));
        mSingleServos.put("outtake-elbow-left-pitch", new ConfServo("outtakeElbowPitchLeft", false));
        mSingleServos.put("outtake-elbow-right-pitch", new ConfServo("outtakeElbowPitchRight", true));
    }
}