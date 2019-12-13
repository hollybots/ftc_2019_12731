package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;


@Autonomous(name="Blue Alliance Tray - 12731", group="1")
public class Autonomous_12731_blue_tray extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
        currentState = STATE_moveToTray;
    }

    protected void moveToTrayState() {

        moveBackward(4, 0.5);
        moveXInchesFromRightObject(17.0, 6000, 0.5);
        moveBackwardByTime(10000, 0.3);
        justWait(500);
        currentState = STATE_clampTray;

    }


    protected void moveTrayBackState() {

        moveXInchesFromFrontObject(3.0, 10000, 0.3);
        turn(90.0);
        botTop.clampRelease();
        justWait(500);
        currentState = STATE_parkUnderBridge;
    }


    protected void parkUnderBridgeState() {

        botTop.clampRelease();
        justWait(3000);
        gotoHeading(90);
        moveForwardToColor(Color.BLUE, 0.3);
        moveXInchesFromRightObject(3.0, 10000, 0.3);
        currentState = STATE_done;
    }

}
