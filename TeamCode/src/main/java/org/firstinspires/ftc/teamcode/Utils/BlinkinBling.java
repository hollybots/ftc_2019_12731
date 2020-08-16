package org.firstinspires.ftc.teamcode.Utils;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlinkinBling  {

    private Servo blinkinModule           = null;


    public BlinkinBling(HardwareMap hardwareMap)
    {
        try {
            blinkinModule = hardwareMap.get(Servo.class, "bling");
        } catch (Exception e) {
            blinkinModule = null;
            Log.d("BOTBASE: ", "Cannot intialize Bling");
        }
    }

    public boolean isActive() {
        return ( blinkinModule != null );
    }


    public void  setBlinkinPattern(double patternNo) {
        if ( blinkinModule != null ) {
            Log.d("BOTBASE: ", "Cannot setPattern");
            blinkinModule.setPosition(patternNo);
        }
    }
}