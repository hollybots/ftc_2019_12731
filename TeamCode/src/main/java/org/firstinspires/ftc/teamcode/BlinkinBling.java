package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlinkinBling  {

    private Servo blinkinModule           = null;


    public BlinkinBling(HardwareMap hardwareMap)
    {
//        try {
            blinkinModule = hardwareMap.get(Servo.class, "blinkin");
//        } catch (Exception e) {
//            blinkinModule = null;
//            Log.d("TELEOP: ", "not null!!!");
//        }
    }

    public boolean isActive() {
        return ( blinkinModule != null );
    }


    public void  setBlinkinPattern(double patternNo) {
        if ( blinkinModule != null ) {
            Log.d("TELEOP: ", "not null!!!");
            blinkinModule.setPosition(patternNo);
        }
    }
}