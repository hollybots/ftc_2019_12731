package org.firstinspires.ftc.teamcode.Components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class CollisionAlert {

    static final boolean DEBUG = true;


    /**
     * HARDWARE
     */
    // Collision limit switches
    DigitalChannel limitSwitch                      = null;

    /**
     * STATE VARIABLES
     */

    // Position variables used for storage and calculations
    private Boolean limitSwichtState                    = false;
    private Boolean limitSwichtPreviousState            = false;


    /****
     *
     * @param hardwareMap
     */
    public CollisionAlert(HardwareMap hardwareMap, String limitSwitchHardwareName)
    {
        try {
            limitSwitch = hardwareMap.get(DigitalChannel.class, limitSwitchHardwareName);
        } catch (Exception e) {
            limitSwitch = null;
            Log.d("BOTBASE: ", "Cannot intialize " + limitSwitchHardwareName);
        }
    }


    public Boolean exists() {
        return (limitSwitch != null);
    }

    public void updateLimitSwitchState() {
        if (!exists()) {
            return;
        }
        limitSwichtPreviousState = limitSwichtState;
        limitSwichtState = limitSwitch.getState();
    }

    public Boolean isColliding() {
        return (limitSwichtState == true);
    }


    void dbugThis(String s) {
        if (DEBUG == true) {
            Log.d("COLLISION: ", s);
        }
    }
}
