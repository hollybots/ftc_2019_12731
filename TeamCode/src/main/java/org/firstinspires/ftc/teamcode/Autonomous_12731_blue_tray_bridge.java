package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Alliance Tray Bridge - 12731", group="1")
public class Autonomous_12731_blue_tray_bridge extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
        currentState = STATE_moveToTray;
    }

    protected void moveToTrayState() {

        moveBackward(4, 0.4);
        moveXInchesFromRightObject(17.0, 6000, 0.5);
        moveBackwardByTime(10000, 0.3);
        justWait(500);
        currentState = STATE_clampTray;

    }


    protected void moveTrayBackState() {

        moveXInchesFromFrontObject(4.0, 10000, 0.3);
        turn(90.0);
        botTop.clampRelease();
        justWait(500);
        currentState = STATE_parkUnderBridge;
    }


    protected void parkUnderBridgeState() {

        gotoHeading(90);
        moveForward(3.0, 0.3);
        moveLeft(18.0,  0.3);
        moveForwardToColor(Color.BLUE, 0.3);
        currentState = STATE_done;
    }

}
