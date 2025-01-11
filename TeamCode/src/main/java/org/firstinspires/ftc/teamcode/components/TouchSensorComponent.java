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


/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.TouchSensor;

public abstract class TouchSensorComponent {

    protected boolean  mReady;
    protected String   mName;

    public boolean                              isReady() { return mReady;}
    public String                               getName() { return mName; }

    public abstract double	                    getValue();
    public abstract boolean                     isPressed();


}
