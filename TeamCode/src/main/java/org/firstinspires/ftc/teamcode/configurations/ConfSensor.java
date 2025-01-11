/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Sensor configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.util.LinkedHashMap;
import java.util.Map;

public class ConfSensor {

    public enum Type {
        TOUCH
    };

    private       boolean           mShallMock   = false;

    private final Map<String, Type> mHw    = new LinkedHashMap<>();


    public ConfSensor(String name, Type type) {
        mHw.clear();
        mHw.put(name,type);
        mShallMock    = false;
    }
    public ConfSensor(String name1, Type type1, String name2, Type type2) {
        mHw.clear();
        mHw.put(name1,type1);
        mHw.put(name2,type2);
        mShallMock    = false;
    }

    public void addHw(String name, Type type) { mHw.put(name,type);  }

    public Map<String, Type   >       getHw()                  { return mHw;}
    public boolean                    shallMock()              { return mShallMock; }
    public Map.Entry<String, Type> getHw(int index)            {
        Map.Entry<String, Type> result = null;
        int iHw = 0;
        for (Map.Entry<String, Type> pos : mHw.entrySet()) {
            if(iHw == index) { result = pos; }
            iHw ++;
        }
        return result;
    }

}