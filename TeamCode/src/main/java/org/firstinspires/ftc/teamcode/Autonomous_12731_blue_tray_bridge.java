package org.firstinspires.ftc.teamcode;


import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Alliance Tray Bridge - 12731", group="1")
public class Autonomous_12731_blue_tray_bridge extends Autonomous_12731_blue_tray {

    protected void parkUnderBridgeState() {

        gotoHeading(90);
        moveForward(3.0, 0.3);
        moveLeft(18.0,  0.3);
        moveForwardToColor(Color.BLUE, 0.3);
        currentState = STATE_done;
    }
}
