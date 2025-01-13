package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Intake includes */
import org.firstinspires.ftc.teamcode.intake.IntakeSlides;
import org.firstinspires.ftc.teamcode.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.intake.IntakeElbow;
import org.firstinspires.ftc.teamcode.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.intake.IntakeClaw;

/* Outtake includes */
import org.firstinspires.ftc.teamcode.outtake.OuttakeSlides;
import org.firstinspires.ftc.teamcode.outtake.OuttakeElbow;
import org.firstinspires.ftc.teamcode.outtake.OuttakeWrist;
import org.firstinspires.ftc.teamcode.outtake.OuttakeClaw;


public class Collecting {

    Telemetry       logger;

    IntakeSlides    intakeSlides;
    IntakeArm       intakeArm;
    IntakeElbow     intakeElbow;
    IntakeWrist     intakeWrist;
    IntakeClaw      intakeClaw;
    OuttakeSlides   outtakeSlides;
    OuttakeElbow    outtakeElbow;
    OuttakeWrist    outtakeWrist;
    OuttakeClaw     outtakeClaw;

    Gamepad         gamepad;
    boolean         wasXPressed;
    boolean         wasAPressed;
    boolean         wasYPressed;
    boolean         wasBPressed;
    boolean         wasDPadUpPressed;
    boolean         wasDPadDownPressed;
    boolean         wasDPadLeftPressed;
    boolean         wasDPadRightPressed;
    boolean         wasTriggerLeftXPositifPressed;
    boolean         wasTriggerLeftXNegatifPressed;
    boolean         wasTriggerRightXPositifPressed;
    boolean         wasTriggerRightXNegatifPressed;

    public Collecting() {

        intakeSlides = new IntakeSlides();
        intakeArm    = new IntakeArm();
        intakeElbow  = new IntakeElbow();
        intakeWrist  = new IntakeWrist();
        intakeClaw   = new IntakeClaw();

        outtakeSlides = new OuttakeSlides();
        outtakeElbow = new OuttakeElbow();
        outtakeWrist = new OuttakeWrist();
        outtakeClaw = new OuttakeClaw();

        wasXPressed = false;
        wasAPressed = false;
        wasYPressed = false;
        wasBPressed = false;

        wasDPadDownPressed = false;
        wasDPadUpPressed = false;
        wasDPadLeftPressed = false;
        wasDPadRightPressed = false;

        wasTriggerLeftXPositifPressed = false;
        wasTriggerLeftXNegatifPressed = false;
        wasTriggerRightXPositifPressed = false;
        wasTriggerRightXNegatifPressed = false;

    }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm, Gamepad gp) {

        logger = tm;
        logger.addLine("=== COLLECTING ===");

        intakeSlides.setHW(config, hwm, tm);
        intakeArm.setHW(config, hwm, tm);
        intakeElbow.setHW(config, hwm, tm);
        intakeWrist.setHW(config, hwm, tm);
        intakeClaw.setHW(config, hwm, tm);

        outtakeSlides.setHW(config, hwm, tm);
        outtakeElbow.setHW(config, hwm, tm);
        outtakeWrist.setHW(config, hwm, tm);
        outtakeClaw.setHW(config, hwm, tm);

        gamepad = gp;
    }

    public void move() {

        if (gamepad.left_bumper)       {
            logger.addLine("==> EXT OUT SLD");
            outtakeSlides.extend(0.1);
        }
        else if (gamepad.right_bumper) {
            logger.addLine("==> RLB OUT SLD");
            outtakeSlides.rollback(0.1);
        }
        else                            {
            outtakeSlides.stop();
        }

        if(gamepad.right_stick_button) {
            outtakeSlides.setPosition(OuttakeSlides.Position.TRANSFER );
        }

        if(gamepad.left_trigger > 0 )                {
            logger.addLine("==> EXT IN SLD");
            intakeSlides.extend(gamepad.left_trigger * 0.3);
        }
        else if (gamepad.right_trigger > 0)          {
            logger.addLine("==> RLB IN SLD");
            intakeSlides.rollback(gamepad.right_trigger * 0.3);
        }
        else                                         {
            intakeSlides.stop();
        }

        if(gamepad.left_stick_button) {
            intakeSlides.setPosition(IntakeSlides.Position.TRANSFER );
        }

        if(gamepad.x)                 {
            logger.addLine(String.format("==> SWT OUT CLW : " + outtakeClaw.getPosition()));
            if(!wasXPressed){ outtakeClaw.switchPosition(); }
            wasXPressed = true;
        }
        else {
            wasXPressed = false;
        }

        if(gamepad.y)     {
            logger.addLine(String.format("==> MDW OUT ARM : " + outtakeElbow.getPosition()));
            if(!wasYPressed){ outtakeElbow.moveDown(); }
            wasYPressed = true;
        }
        else { wasYPressed = false; }

        if(gamepad.a) {
            logger.addLine(String.format("==> MUP OUT ARM : " + outtakeElbow.getPosition()));
            if(!wasAPressed){ outtakeElbow.moveUp();}
            wasAPressed = true;
        }
        else { wasAPressed = false; }

        if(gamepad.b) {
            logger.addLine(String.format("==> CENTER OUT WRS : " + outtakeWrist.getPosition()));
            if(!wasBPressed){ this.transition() ;}
            wasBPressed = true;
        }
        else { wasBPressed = false; }

        if(gamepad.dpad_left) {
            logger.addLine(String.format("==> SWT IN CLW : " + intakeClaw.getPosition()));
            if(!wasDPadLeftPressed){  intakeClaw.switchPosition(); }
            wasDPadLeftPressed = true;
        }
        else { wasDPadLeftPressed = false; }

        if(gamepad.dpad_up)     {
            logger.addLine(String.format("==> MDW IN ARM : " + intakeArm.getPosition()));
            if(!wasDPadUpPressed){ intakeElbow.moveDown(); intakeArm.moveDown(); }
            wasDPadUpPressed = true;
        }
        else { wasDPadUpPressed = false; }

        if(gamepad.dpad_down) {
            logger.addLine(String.format("==> MUP IN ARM : " + intakeArm.getPosition()));
            if(!wasDPadDownPressed){ intakeArm.moveUp(); intakeElbow.moveUp();}
            wasDPadDownPressed = true;
        }
        else { wasDPadDownPressed = false; }

        if(gamepad.dpad_right) {
            logger.addLine(String.format("==> CENTER IN WRS : " + intakeWrist.getPosition()));
            if(!wasDPadRightPressed){ this.retract() ;}
            wasDPadRightPressed = true;
        }
        else { wasDPadRightPressed = false; }



        if(gamepad.left_stick_x < 0) {
            logger.addLine(String.format("==> RDW IN WRS : " + intakeWrist.getPosition()));
            if(!wasTriggerLeftXNegatifPressed){ intakeWrist.rotateDown(); }
            wasTriggerLeftXNegatifPressed = true;
        }
        else { wasTriggerLeftXNegatifPressed = false; }

        if(gamepad.left_stick_x > 0) {
            logger.addLine(String.format("==> RUP IN WRS : " + intakeWrist.getPosition()));
            if(!wasTriggerLeftXPositifPressed){ intakeWrist.rotateUp(); }
            wasTriggerLeftXPositifPressed = true;
        }
        else { wasTriggerLeftXPositifPressed = false; }

        if(gamepad.right_stick_x < 0) {
            logger.addLine(String.format("==> RDW OUT WRS : " + outtakeWrist.getPosition()));
            if(!wasTriggerRightXNegatifPressed){ outtakeWrist.rotateDown(); }
            wasTriggerRightXNegatifPressed = true;
        }
        else { wasTriggerRightXNegatifPressed = false; }

        if(gamepad.right_stick_x > 0) {
            logger.addLine(String.format("==> RUP OUT WRS : " + outtakeWrist.getPosition()));
            if(!wasTriggerRightXPositifPressed){ outtakeWrist.rotateUp(); }
            wasTriggerRightXPositifPressed = true;
        }
        else { wasTriggerRightXPositifPressed = false; }

        logger.addLine(intakeSlides.getPositions());
        logger.addLine(outtakeSlides.getPositions());
    }

    public void transition (){

        outtakeClaw.setPosition(OuttakeClaw.Position.OPEN);
        outtakeWrist.setPosition(OuttakeWrist.Position.NULL);
        //outtakeSlides.setPosition(OuttakeSlides.Position.TRANSFER);
        //intakeWrist.setPosition(IntakeWrist.Position.NULL );
        //intakeElbow.setPosition(IntakeElbow.Position.TRANSFER );
        //intakeArm.setPosition(IntakeArm.Position.TRANSFER ) ;
        //intakeSlides.setPosition(IntakeSlides.Position.TRANSFER);
        //intakeClaw.setPosition(IntakeClaw.Position.MICRORELEASED);
        //outtakeClaw.setPosition(OuttakeClaw.Position.CLOSED);
        //intakeClaw.setPosition(IntakeClaw.Position.OPEN);
        //outtakeElbow.setPosition(OuttakeElbow.Position.DROP);

    }

    public void retract (){
        //intakeSlides.setPosition(IntakeSlides.Position.TRANSFER );
        //intakeClaw.setPosition(IntakeClaw.Position.CLOSED );
//        //intakeWrist.setPosition(IntakeWrist.Position.NULL );
//        intakeElbow.setPosition(IntakeElbow.Position.OFF );
//        intakeArm.setPosition(IntakeArm.Position.OFF );
//        outtakeClaw.setPosition(OuttakeClaw.Position.CLOSED );
//        outtakeWrist.setPosition(OuttakeWrist.Position.NULL  );
//        outtakeElbow.setPosition(OuttakeElbow.Position.OFF );
//        outtakeSlides.setPosition(OuttakeSlides.Position.MIN );
//        intakeSlides.setPosition(IntakeSlides.Position.MIN );


    }

}

