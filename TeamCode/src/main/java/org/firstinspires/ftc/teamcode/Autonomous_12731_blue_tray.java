package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Alliance Tray - 12731", group="1")
public class Autonomous_12731_blue_tray extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
        currentState = STATE_moveToTray;
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }

    protected void moveToTrayState() {

        moveLeftByTime(1850, 0.9);
        moveForwardByTime(2312, 0.9);
        currentState = STATE_clampTray;
    }


    protected void parkUnderBridgeState() {

        gotoHeading(0);
        moveRightByTime(2000, 0.9);
        currentState = STATE_done;
    }
}
