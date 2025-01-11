package org.firstinspires.ftc.teamcode.intake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configurations includes */
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

import java.util.LinkedHashMap;
import java.util.Map;

public class IntakeSlides {
    public enum Position {
        MIN,
        TRANSFER,
        MAX
    };
    private static final Map<String, Position> sConfToPosition = Map.of(
            "min",  Position.MIN,
            "transfer", Position.TRANSFER,
            "max", Position.MAX
    );
    Telemetry            mLogger;

    boolean              mReady;


    Position             mPosition;

    MotorComponent       mMotorRight;
    MotorComponent       mMotorLeft;

    Map<Position, Integer> mPositionsLeft = new LinkedHashMap<>();
    Map<Position, Integer> mPositionsRight = new LinkedHashMap<>();


    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;
        String status = "";

        ConfMotor slides = config.getMotor("intake-slides-left");
        if(slides == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (slides.shallMock()) { mMotorLeft = new MotorMock("intake-slides-left"); }
            else if (slides.getHw().size() == 1) { mMotorLeft = new MotorSingle(slides, hwm, "intake-slides-left", logger); }
            else if (slides.getHw().size() == 2) { mMotorLeft = new MotorCoupled(slides, hwm, "intake-slides-left", logger); }

            if (!mMotorLeft.isReady()) { mReady = false; status += " HW";}
            else {
                mMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mPositionsLeft.clear();
                Map<String, Integer> confPosition = slides.getPositions();
                for (Map.Entry<String, Integer> pos : confPosition.entrySet()) {
                    if(sConfToPosition.containsKey(pos.getKey())) {
                        mPositionsLeft.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                    }
                }

            }
        }

        slides = config.getMotor("intake-slides-right");
        if(slides == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (slides.shallMock()) { mMotorRight = new MotorMock("intake-slides-right"); }
            else if (slides.getHw().size() == 1) { mMotorRight = new MotorSingle(slides, hwm, "intake-slides-right", logger); }
            else if (slides.getHw().size() == 2) { mMotorRight = new MotorCoupled(slides, hwm, "intake-slides-right", logger); }

            if (!mMotorRight.isReady()) { mReady = false; status += " HW";}
            else {
                mMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (mReady) { logger.addLine("==>  IN SLD : OK"); }
        else        { logger.addLine("==>  IN SLD : KO : " + status); }

    }

    public void extend(double Power)   {
        if(mReady) {
            if(mMotorLeft.getTargetPosition() < mPositionsRight.get(Position.MAX)){
                mMotorLeft.setPower(Power);
            }
            if(mMotorRight.getTargetPosition() < mPositionsRight.get(Position.MAX)){
                mMotorRight.setPower(Power);
            }
            if(mMotorRight.getTargetPosition() > mPositionsRight.get(Position.MAX)) {
                mMotorRight.setPower(0);
            }
            if(mMotorLeft.getTargetPosition() > mPositionsRight.get(Position.MAX)) {
                mMotorLeft.setPower(0);
            }
        }

    }

    public void stop() {
        if (mReady) {
            mMotorRight.setPower(0);
            mMotorLeft.setPower(0);
        }
    }

    public void rollback(double Power) {
        if(mReady) {
            if(mMotorLeft.getTargetPosition() > mPositionsRight.get(Position.MIN)){
                mMotorLeft.setPower(Power);
            }
            if(mMotorRight.getTargetPosition() > mPositionsRight.get(Position.MIN)){
                mMotorRight.setPower(Power);
            }
            if(mMotorRight.getTargetPosition() < mPositionsRight.get(Position.MIN)) {
                mMotorRight.setPower(0);
            }
            if(mMotorLeft.getTargetPosition() < mPositionsRight.get(Position.MIN)) {
                mMotorLeft.setPower(0);
            }
        }

    }
    public void setPosition(Position position) {
    }

    public String getPositions()
    {
        return "L : " + mMotorLeft.getCurrentPosition() + " R : " + mMotorRight.getCurrentPosition();
    }

}


