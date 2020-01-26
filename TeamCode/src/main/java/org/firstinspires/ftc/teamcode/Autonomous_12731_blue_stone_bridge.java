package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Alliance Stone Bridge - 12731", group="1")
public class Autonomous_12731_blue_stone_bridge extends Autonomous_12731_blue_stone {

    protected void dropOffStoneState() {
        botTop.openClaw();
        moveForward(4.0, 0.6);
        justWait(1000);
        moveForward(4.0, 0.4);
//        moveXInchesFromBackObject(4.0, 10000, 0.5);
        currentState = STATE_parkUnderBridge;
        return;
    }

    // From the build zone
    protected void parkUnderBridgeState() {
        gotoHeading(0);
        moveRightToColor(Color.BLUE, 0.4);
        currentState = STATE_done;
        return;
    }
}

