package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;

@Autonomous(name="Red Alliance Tray - 12731", group="1")
public class Autonomous_12731_red_tray extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
        currentState = STATE_moveToTray;
    }

    protected void moveToTrayState() {

        moveBackward(4, 0.4);
        moveXInchesFromLeftObject(17.0, 6000, 0.5);
        moveBackwardByTime(10000, 0.3);
        moveBackwardByTime(10000, 0.1);
        justWait(500);
        currentState = STATE_clampTray;

    }


    protected void moveTrayBackState() {

        moveXInchesFromFrontObject(4.0, 10000, 0.3);
        turn(-90.0);
        botTop.clampRelease();
        justWait(500);
        currentState = STATE_parkUnderBridge;
    }


    protected void parkUnderBridgeState() {

        gotoHeading(-90);
        moveForward(3.0, 0.3);
        moveXInchesFromLeftObject(3.0, 10000, 0.3);
        moveForwardToColor(Color.RED, 0.3);
        currentState = STATE_done;
    }

}
