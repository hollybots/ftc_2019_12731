package org.firstinspires.ftc.teamcode;


import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Alliance Tray Bridge - 12731", group="1")
public class Autonomous_12731_red_tray_bridge extends Autonomous_12731_red_tray {

    protected void parkUnderBridgeState() {

        gotoHeading(-90);
        moveForward(3.0, 0.3);
        moveRight(18.0,  0.3);
        moveForwardToColor(Color.RED, 0.3);
        currentState = STATE_done;
    }
}
