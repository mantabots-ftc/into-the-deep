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
import org.firstinspires.ftc.teamcode.configurations.ConfSensor;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.components.TouchSensorComponent;
import org.firstinspires.ftc.teamcode.components.TouchSensorMock;
import org.firstinspires.ftc.teamcode.components.TouchSensorCoupled;
import org.firstinspires.ftc.teamcode.components.TouchSensorSingle;

public class OuttakeSlides {

    Telemetry            mLogger;

    boolean              mReady;

    MotorComponent       mLeftMotor;
    MotorComponent       mRightMotor;
    TouchSensorComponent mLeftTouch;
    TouchSensorComponent mRightTouch;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;
        String status = "";

        ConfMotor slides = config.getMotor("outtake-slides-left");
        if(slides == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (slides.shallMock()) { mLeftMotor = new MotorMock("outtake-slides-left"); }
            else if (slides.getHw().size() == 1) { mLeftMotor = new MotorSingle(slides, hwm, "outtake-slides-left", logger); }
            else if (slides.getHw().size() == 2) { mLeftMotor = new MotorCoupled(slides, hwm, "outtake-slides-left", logger); }

            if (!mLeftMotor.isReady()) { mReady = false; status += " HW ML";}
            else {
                mLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        slides = config.getMotor("outtake-slides-right");
        if(slides == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (slides.shallMock()) { mRightMotor = new MotorMock("outtake-slides-right"); }
            else if (slides.getHw().size() == 1) { mRightMotor = new MotorSingle(slides, hwm, "outtake-slides-right", logger); }
            else if (slides.getHw().size() == 2) { mRightMotor = new MotorCoupled(slides, hwm, "outtake-slides-right", logger); }

            if (!mRightMotor.isReady()) { mReady = false; status += " HW MR";}
            else {
                mRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        ConfSensor sensor = config.getSensor("outtake-slides-left");
        if(sensor == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (sensor.shallMock()) { mLeftTouch = new TouchSensorMock("outtake-slides-left-touch"); }
            else if (sensor.getHw().size() == 1) { mLeftTouch = new TouchSensorSingle(sensor, hwm, "outtake-slides-left-touch", logger); }
            else if (sensor.getHw().size() == 2) { mLeftTouch = new TouchSensorCoupled(sensor, hwm, "outtake-slides-left-touch", logger); }

            if (!mLeftTouch.isReady()) { mReady = false; status += " HW TL";}
        }

        sensor = config.getSensor("outtake-slides-right");
        if(sensor == null)  { mReady = false; status += " CONF";}
        else {

            // Configure motor
            if (sensor.shallMock()) { mRightTouch = new TouchSensorMock("outtake-slides-right-touch"); }
            else if (sensor.getHw().size() == 1) { mRightTouch = new TouchSensorSingle(sensor, hwm, "outtake-slides-right-touch", logger); }
            else if (sensor.getHw().size() == 2) { mRightTouch = new TouchSensorCoupled(sensor, hwm, "outtake-slides-right-touch", logger); }

            if (!mLeftTouch.isReady()) { mReady = false; status += " HW TR";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  OUT SLD : OK"); }
        else        { logger.addLine("==>  OUT SLD : KO : " + status); }

    }

    public void extend(double Power)   {
        if(mReady) {
            mLeftMotor.setPower(Power);
            mRightMotor.setPower(Power);
        }
    }

    public void stop() {
        if (mReady) {
            mLeftMotor.setPower(0);
            mRightMotor.setPower(0);
        }
    }

    public void rollback(double Power) {
        mLogger.addLine(" ==> OUT SLD TCH L : " + mLeftTouch.isPressed());
        mLogger.addLine(" ==> OUT SLD TCH R : " + mRightTouch.isPressed());
        if(mReady && !mLeftTouch.isPressed()) {
            mLeftMotor.setPower(-Power);
        }
        else if(mReady && mLeftTouch.isPressed()) {
            mLeftMotor.setPower(0);
        }
        if(mReady && !mRightTouch.isPressed()) {
            mRightMotor.setPower(-Power);
        }
        else if(mReady && mRightTouch.isPressed()) {
            mRightMotor.setPower(0);
        }
    }

}