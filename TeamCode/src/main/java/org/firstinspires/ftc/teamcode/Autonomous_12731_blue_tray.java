package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Blue Alliance Tray - 12731", group="1")
public class Autonomous_12731_blue_tray extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
        currentState = STATE_moveToTray;
    }

    protected void moveToTrayState() {

        moveBackward(4, 0.5);
        moveXInchesFromRightObject(24.0 - DISTANCE_RIGHT_SENSORS, 6000, 0.5);
        moveBackwardByTime(10000, 0.5);
        currentState = STATE_clampTray;
    }


    protected void parkUnderBridgeState() {

        gotoHeading(0);
        double currentDistanceRight = getValidDistance(distanceRight);
        moveLeft(68.0 - currentDistanceRight + DISTANCE_RIGHT_SENSORS, 0.5);
        currentState = STATE_done;
    }

}
