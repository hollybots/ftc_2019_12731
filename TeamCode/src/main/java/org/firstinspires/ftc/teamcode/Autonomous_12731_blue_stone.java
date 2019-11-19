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

    /**
     * Here we scan left an right to find a stone.  One the stone is found
     * we go to the next state
     */
    protected void scanForStoneState() {

        stoneRelativePlacement = vuMark.find();
        if (stoneRelativePlacement != null ) {
            stopMoving();
            currentState = STATE_alignWithStone;
            dbugThis(String.format("Found the stone at Pos (in)  : {X, Y} = %.1f, %.1f",
                    stoneRelativePlacement.x, stoneRelativePlacement.y));
            return;
        }

        moveXInchesFromRightObject(9.0, 10000, 0.4);
        boolean goingRight = true;

        while (opModeIsActive() && stoneRelativePlacement == null) {

            stoneRelativePlacement = vuMark.find();
            if (stoneRelativePlacement != null) {
                currentState = STATE_alignWithStone;
                dbugThis(String.format("Found the stone at Pos (in)  : {X, Y} = %.1f, %.1f",
                        stoneRelativePlacement.x, stoneRelativePlacement.y));
                return;
            }
            if ( goingRight ) {
                gotoHeading(0);
                moveLeftByTime(6000, 0.2);
            } else {
                gotoHeading(0);
                moveRightByTime(6000, 0.2);
            }

            goingRight = !goingRight;
            telemetry.update();
        }

        return;
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
        moveRight(20.0, 0.5);
        currentState = STATE_done;
        return;
    }

}
