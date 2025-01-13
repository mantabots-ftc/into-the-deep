package org.firstinspires.ftc.teamcode.intake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.outtake.OuttakeSlides;

import java.util.LinkedHashMap;
import java.util.Map;

public class IntakeSlides {
    public enum Position {
        MIN,
        TRANSFER,
        MAX,
        UNKNOWN
    };
    private static final Map<String, Position> sConfToPosition = Map.of(
            "min",  Position.MIN,
            "transfer", Position.TRANSFER,
            "max", Position.MAX
    );
    Telemetry            mLogger;

    boolean              mReady;
    ElapsedTime          mTimer;

    Position             mPosition;

    MotorComponent       mMotorRight;
    MotorComponent       mMotorLeft;

    Map<Position, Integer> mPositionsLeft = new LinkedHashMap<>();
    Map<Position, Integer> mPositionsRight = new LinkedHashMap<>();
    
    public boolean isBusy() { return (mMotorRight.isBusy() || mMotorLeft.isBusy());}


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
        if(mReady && !this.isBusy())
        {
            mMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mPosition = Position.UNKNOWN;

            boolean shall_work = (
                    (mMotorLeft.getCurrentPosition() < mPositionsLeft.get(Position.MAX)) &&
                    (mMotorRight.getCurrentPosition() < mPositionsRight.get(Position.MAX)));

            if(shall_work){
                mMotorLeft.setPower(Power);
                mMotorRight.setPower(Power);
            }
            if(!shall_work) {
                mMotorRight.setPower(0);
                mMotorLeft.setPower(0);
            }

        }

    }

    public void stop() {
        if(mReady && !this.isBusy()) {
            mMotorRight.setPower(0);
            mMotorLeft.setPower(0);
        }
    }

    public void rollback(double Power) {
        if(mReady && !this.isBusy()) {
            mMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mPosition = Position.UNKNOWN;

            boolean shall_work = (
                    (mMotorLeft.getCurrentPosition() > mPositionsLeft.get(Position.MIN)) &&
                    (mMotorRight.getCurrentPosition() > mPositionsLeft.get(Position.MIN)));

            if(shall_work){
                mMotorLeft.setPower(-Power);
                mMotorRight.setPower(-Power);
            }
            if(!shall_work) {
                mMotorRight.setPower(0);
                mMotorLeft.setPower(0);
            }

        }

    }

    public void setPosition(Position position)
    {
        if(mPositionsLeft.containsKey(position) && mPositionsRight.containsKey(position)) {
            mMotorLeft.setTargetPosition(mPositionsLeft.get(position));
            mMotorRight.setTargetPosition(mPositionsRight.get(position));

            mMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION   );
            mMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION   );

            while(mMotorRight.isBusy() || mMotorLeft.isBusy()) {
                mLogger.addLine("" + mMotorLeft.getCurrentPosition());
                mLogger.addLine("" + mMotorLeft.getTargetPosition());
                mLogger.addLine("" + mMotorRight.getCurrentPosition());
                mLogger.addLine("" + mMotorRight.getTargetPosition());
                mLogger.update();
                mMotorRight.setPower(0.35);
                mMotorLeft.setPower(0.35);
            }
            mPosition = position;
        }
    }

    public String getPositions()
    {
        return "POS IN SLD L : " + mMotorLeft.getCurrentPosition() + " R : " + mMotorRight.getCurrentPosition();
    }

}


