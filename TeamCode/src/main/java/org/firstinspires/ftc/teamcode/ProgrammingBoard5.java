package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;




public class ProgrammingBoard5 {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRotation;
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;




    public void init(HardwareMap hwMap) {
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
        servo1 = hwMap.get(Servo.class,"servo1");
        servo2 = hwMap.get(Servo.class,"servo2");
        servo3 = hwMap.get(Servo.class,"servo3");
    }
    public boolean isTouchSensorPressed() {
        return !touchSensor.getState();
    }

    public void setMotorSpeed(double speed) {
        motor.setPower(speed);
    }

    public String isTouchSensorReleased() {
        if (!touchSensor.getState() == false) {
            return "Released";
        }
        return "Pressed";
    }

    public double getMotorRotations() {
        return motor.getCurrentPosition() / ticksPerRotation;
    }
    public void setServoPosition(double position, Servo servo){
        servo.setPosition(position);
    }
}
