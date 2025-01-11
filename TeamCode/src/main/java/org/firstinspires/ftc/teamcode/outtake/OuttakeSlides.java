package org.firstinspires.ftc.teamcode.outtake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configurations includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.intake.IntakeSlides;

import java.util.LinkedHashMap;
import java.util.Map;

public class OuttakeSlides {
    public enum Position {
        MIN,
        TRANSITION,
        MAX
    };
    private static final Map<String, Position> sConfToPosition = Map.of(
            "min",  Position.MIN,
            "transition", Position.TRANSITION,
            "max", Position.MAX
    );
    Telemetry            mLogger;

    boolean              mReady;

    MotorComponent       mMotorRight;
    MotorComponent       mMotorLeft;

    TouchSensor          mTouchSensorRight;
    TouchSensor          mTouchSensorLeft;

    Map<Position, Integer> mPositionsLeft = new LinkedHashMap<>();
    Map<Position, Integer> mPositionsRight = new LinkedHashMap<>();


    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;
        String status = "";

        mTouchSensorRight = hwm.touchSensor.get("outtakeSlidesRightTouch");
        mTouchSensorLeft = hwm.touchSensor.get("outtakeSlidesLeftTouch");

        ConfMotor slides = config.getMotor("outtake-slides-left");
        if(slides == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (slides.shallMock()) { mMotorLeft = new MotorMock("outtake-slides-left"); }
            else if (slides.getHw().size() == 1) { mMotorLeft = new MotorSingle(slides, hwm, "outtake-slides-left", logger); }
            else if (slides.getHw().size() == 2) { mMotorLeft = new MotorCoupled(slides, hwm, "outtake-slides-left", logger); }

            if (!mMotorLeft.isReady()) { mReady = false; status += " HW";}
            else {
                mMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER  );


                mPositionsLeft.clear();
                Map<String, Integer> confPosition = slides.getPositions();
                for (Map.Entry<String, Integer> pos : confPosition.entrySet()) {
                    if(sConfToPosition.containsKey(pos.getKey())) {
                        mPositionsLeft.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                    }
                }
            }
        }

        slides = config.getMotor("outtake-slides-right");
        if(slides == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (slides.shallMock()) { mMotorRight = new MotorMock("outtake-slides-right"); }
            else if (slides.getHw().size() == 1) { mMotorRight = new MotorSingle(slides, hwm, "outtake-slides-right", logger); }
            else if (slides.getHw().size() == 2) { mMotorRight = new MotorCoupled(slides, hwm, "outtake-slides-right", logger); }

            if (!mMotorRight.isReady()) { mReady = false; status += " HW";}
            else {
                mMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER  );
                mPositionsRight.clear();
                Map<String, Integer> confPosition = slides.getPositions() ;
                for (Map.Entry<String, Integer> pos : confPosition.entrySet()) {
                    if(sConfToPosition.containsKey(pos.getKey())) {
                        mPositionsRight.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                    }
                }
            }
        }


        if (!mPositionsLeft.containsKey(Position.MIN)) { mReady = false; }
        if (!mPositionsRight.containsKey(Position.MIN)) { mReady = false; }
        if (!mPositionsRight.containsKey(Position.MAX)) { mReady = false; }
        if (!mPositionsLeft.containsKey(Position.MAX)) { mReady = false; }

        // Log status
        if (mReady) { logger.addLine("==>  OUT SLD : OK"); }
        else        { logger.addLine("==>  OUT SLD : KO : " + status); }

    }

    public void extend(double Power)   {
        if(mReady) {
            
            if(mMotorLeft.getCurrentPosition() < mPositionsLeft.get(Position.MAX)){
                mMotorLeft.setPower(Power);
            }
            if(mMotorRight.getCurrentPosition() < mPositionsRight.get(Position.MAX)){
                mMotorRight.setPower(Power);
            }
            if(mMotorRight.getCurrentPosition() > mPositionsRight.get(Position.MAX)) {
                mMotorRight.setPower(0);
            }
            if(mMotorLeft.getCurrentPosition() > mPositionsLeft.get(Position.MAX)) {
                mMotorLeft.setPower(0);
            }
        }

    }

    public void stop() {
        if (mReady) {
            mMotorLeft.setPower(0);
            mMotorRight.setPower(0);
              }
    }

    public void rollback(double Power) {
        if(mReady && !mTouchSensorLeft.isPressed()) {
            mMotorLeft.setPower(-Power);

        }
        if(mReady && !mTouchSensorRight.isPressed()) {
            mMotorRight.setPower(-Power);
        }


        if(mReady && mTouchSensorRight.isPressed()) {
            mMotorRight.setPower(0);
        }
        if(mReady && mTouchSensorLeft.isPressed()) {
            mMotorLeft.setPower(0);
        }
    }
    public String getPositions()
    {
        return " Outake L : " + mMotorLeft.getCurrentPosition() + "Outake R : " + mMotorRight.getCurrentPosition();
    }

    public void setPosition(Position position)
    {
        if(mPositionsLeft.containsKey(position) && mPositionsRight.containsKey(position)) {
        mMotorLeft.setTargetPosition(mPositionsLeft.get(position));
        mMotorRight.setTargetPosition(mPositionsRight.get(position));
        mMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION   );
        mMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION   );


        }
    }



}


