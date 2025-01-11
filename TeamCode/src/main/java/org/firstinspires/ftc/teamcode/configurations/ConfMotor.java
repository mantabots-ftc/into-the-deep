/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Motor configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

public class ConfMotor {

    private       boolean               mShallMock   = false;

    private final Map<String, Boolean>  mHw    = new LinkedHashMap<>();
    private final Map<String, Boolean>  mEncoder    = new LinkedHashMap<>();
    private final Map<String, Integer > mPositions   = new LinkedHashMap<>();

    public ConfMotor(String Name, boolean ShallReverse)
    {
        mHw.clear();
        mHw.put(Name,ShallReverse);
        mEncoder.put(Name,false);
        mShallMock    = false;
    }
    public ConfMotor(String Name, boolean ShallReverse, boolean ShallInvertEncoder)
    {
        mHw.clear();
        mHw.put(Name,ShallReverse);
        mEncoder.put(Name,ShallInvertEncoder);
        mShallMock    = false;
    }

    public ConfMotor(String Name1, boolean ShallReverse1, String Name2, boolean ShallReverse2)
    {
        mHw.clear();
        mHw.put(Name1,ShallReverse1);
        mHw.put(Name2,ShallReverse2);
        mEncoder.put(Name1,false);
        mEncoder.put(Name2,false);
        mShallMock    = false;
    }

    public ConfMotor(String Name1, boolean ShallReverse1, boolean ShallInvertEncoder1, String Name2, boolean ShallReverse2, boolean ShallInvertEncoder2)
    {
        mHw.clear();
        mHw.put(Name1,ShallReverse1);
        mHw.put(Name2,ShallReverse2);
        mEncoder.put(Name1,ShallInvertEncoder1);
        mEncoder.put(Name2,ShallInvertEncoder2);
        mEncoder.put(Name1,false);
        mEncoder.put(Name2,false);
        mShallMock    = false;
    }

    public void addHw(String Name, boolean ShallReverse) { mHw.put(Name,ShallReverse);  }
    public void addPosition(String Name, Integer Value)   { mPositions.put(Name, Value); }

    public Map<String, Boolean>       getHw()                  { return mHw;}
    public Map<String, Boolean>       getEncoder()             { return mEncoder;}
    public boolean                    shallMock()              { return mShallMock; }
    public Map<String, Integer>        getPositions()          { return mPositions; }
    public Map.Entry<String, Boolean> getHw(int index)         {
        Map.Entry<String, Boolean> result = null;
        int iHw = 0;
        for (Map.Entry<String, Boolean> pos : mHw.entrySet()) {
            if(iHw == index) { result = pos; }
            iHw ++;
        }
        return result;

    }
    public Map.Entry<String, Boolean> getEncoder(int index)         {
        Map.Entry<String, Boolean> result = null;
        int iHw = 0;
        for (Map.Entry<String, Boolean> pos : mEncoder.entrySet()) {
            if(iHw == index) { result = pos; }
            iHw ++;
        }
        return result;

    }
    public Integer              getPosition(String Name) {
        if(mPositions.containsKey(Name)) {
            return mPositions.get(Name);
        }
        return -100000;
    }


}