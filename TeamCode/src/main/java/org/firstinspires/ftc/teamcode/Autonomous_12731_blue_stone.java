package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Blue Alliance Stone - 12731", group="1")
public class Autonomous_12731_blue_stone extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
//        currentState = STATE_getCloseEnoughToPickup;
        currentState = STATE_moveToStones;
    }


    protected void travelToBuildSiteState() {
        botBase.setBling(LED_TEAM_COLORS3);
        moveXInchesFromBackObject(12.0, 100000, 0.4);
        gotoHeading(0);
        justWait(500);
        double toGo = 100.0 - getValidDistance(distanceRight) - DISTANCE_RIGHT_SENSORS;
        moveLeft(toGo, 0.4);
        currentState = STATE_dropOffStone;
        return;
    }


    protected void travelHomeState() {
        gotoHeading(0);
        moveXInchesFromRightObject(9.0, 5000,0.5);
        gotoHeading(0);
        currentState = STATE_moveToStones;
        return;
    }


    protected void parkUnderBridgeState() {
        gotoHeading(0);
        moveXInchesFromBackObject(12.0, 5000, 0.5);
        moveRight(20.0, 0.5);
        currentState = STATE_done;
        return;
    }

}
