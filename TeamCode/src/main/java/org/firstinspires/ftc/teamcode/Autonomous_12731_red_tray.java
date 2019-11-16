package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Red Alliance Tray - 12731", group="1")
public class Autonomous_12731_red_tray extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
        currentState = STATE_moveToTray;
    }

    protected void moveToTrayState() {

        moveBackward(4, 0.5);
        moveXInchesFromLeftObject(24.0 - DISTANCE_LEFT_SENSORS, 6000, 0.5);
        moveBackwardByTime(10000, 0.5);
        currentState = STATE_clampTray;
    }


    protected void parkUnderBridgeState() {

        gotoHeading(0);
        double currentDistanceLeft = distanceLeft.getDistance(DistanceUnit.INCH);
        moveRight(68.0 - currentDistanceLeft + DISTANCE_LEFT_SENSORS, 0.5);
        currentState = STATE_done;
    }

}
