/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   CoupledMotor class overloads the FTC motor class to manage
   A couple of motors both turning the same hardware.

   Note that this is a dangerous situation which can result in
   motor destruction if not correctly tuned. The coupled motors
   shall be the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* System includes */
import static java.lang.Math.max;

import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Map;
import java.util.List;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfSensor;

public class TouchSensorCoupled extends TouchSensorComponent {

    Telemetry                      mLogger;

    TouchSensor                    mFirst;
    TouchSensor                    mSecond;

    /* -------------- Constructors --------------- */
    public TouchSensorCoupled(ConfSensor conf, HardwareMap hwMap, String name, Telemetry logger)
    {
        mReady  = true;
        mLogger = logger;
        mName   = name;

        Map<String, ConfSensor.Type> hw = conf.getHw();
        if((hw.size() == 2) && !conf.shallMock()) {

            List<Map.Entry<String, ConfSensor.Type>> sensors = new ArrayList<>(hw.entrySet());
            ListIterator<Map.Entry<String, ConfSensor.Type>> iterator = sensors.listIterator();

            Map.Entry<String,ConfSensor.Type> sensor = iterator.next();
            if(sensor.getValue() == ConfSensor.Type.TOUCH)
            {
                mFirst = hwMap.tryGet(TouchSensor.class, sensor.getKey());
            }
            sensor = iterator.next();
            if(sensor.getValue() == ConfSensor.Type.TOUCH)
            {
                mSecond = hwMap.tryGet(TouchSensor.class, sensor.getKey());
            }
        }

        if(mFirst  == null) { mReady = false; }
        if(mSecond == null) { mReady = false; }
    }

    @Override
    public double	                    getValue()
    {
        double result = 1;
        if(mReady) {
            result = max(mFirst.getValue(),mSecond.getValue());
        }
        return result;
    }

    @Override
    public boolean                     isPressed()
    {
        boolean result = false;
        if(mReady) {
            if(mFirst.isPressed()) { result = true; }
            if(mSecond.isPressed()) { result = true; }
        }
        return result;
    }

}
