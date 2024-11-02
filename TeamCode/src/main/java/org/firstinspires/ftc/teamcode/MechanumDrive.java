

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class MechanumDrive extends OpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    double botHeading ;
    IMU imu;
    boolean aIsPressed = false;
    double imuTarget;
    double replace;
    double difference;
    private DistanceSensor distanceSensor;
    @Override
    public void init (){
       frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
       backLeft = hardwareMap.get(DcMotor.class,"backLeft");
       frontRight = hardwareMap.get(DcMotor.class,"frontRight");
       backRight = hardwareMap.get(DcMotor.class,"backRight");
       backRight.setDirection(DcMotorSimple.Direction.REVERSE);
       backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       distanceSensor = hardwareMap.get(DistanceSensor.class,"sensor_color_distance");
       double multiplier;



         imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD ));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

         botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }
    @Override
    public void loop(){

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double multiplier = 0.45;
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        difference = imuTarget-botHeading;
        while (difference < 0) { difference += 2 * Math.PI; }
        while (difference >= 2 * Math.PI) { difference -= 2 * Math.PI; }
        if (botHeading < 0){
            botHeading += Math.PI*2;
        }


        if (gamepad1.left_bumper) {
            multiplier = 0.9;
        }
        if(gamepad1.right_bumper){
            multiplier=0.25;
        }
        if(gamepad1.a){
            aIsPressed = true;

             imuTarget =  (botHeading + Math.PI)%(Math.PI*2);
             if (imuTarget > 2*Math.PI){
                 imuTarget -= Math.PI*2;
             }


        }


        if (aIsPressed){



            if (Math.abs(difference) >0.1) {

                telemetry.addData("ImuTarget", imuTarget);
                telemetry.addData("botHeading", botHeading);
                telemetry.addData("aIsPressed",aIsPressed);
                telemetry.addData("difference",difference);

                frontLeft.setPower(-0.6);
                backLeft.setPower(-0.6);
                frontRight.setPower(0.6);
                backRight.setPower(0.6);
                telemetry.update();
            }else aIsPressed = false;
        }


        frontLeft.setPower(((y+x+rotation)/denominator)* multiplier*1.1);
        backLeft.setPower(((y-x+rotation)/denominator)* multiplier*1.1);
        frontRight.setPower(((y-x-rotation)/denominator)* multiplier);
        backRight.setPower(((y+x-rotation)/denominator)* multiplier);


  }
}
