package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTesting extends OpMode {
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;



    @Override
    public void init (){
        servo1 = hardwareMap.get(Servo.class,"servo1");
       servo2 = hardwareMap.get(Servo.class,"servo2");
       servo3= hardwareMap.get(Servo.class,"servo3");
    }
    @Override
    public void loop () {
        if (gamepad1.b){
           servo1.setPosition(1.0);
        }
        if (gamepad1.a){
            servo2.setPosition(1.0);
        }
        if(gamepad1.x){
            servo3.setPosition(1.0);
        }
        if(gamepad1.y){
           servo1.setPosition(0.0);
            servo2.setPosition(0.0);
            servo3.setPosition(0.0);
        }
    }
}
