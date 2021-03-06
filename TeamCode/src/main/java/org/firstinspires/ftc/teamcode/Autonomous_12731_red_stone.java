package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;

@Autonomous(name="Red Alliance Stone - 12731", group="1")
public class Autonomous_12731_red_stone extends Autonomous_12731 {


    @Override
    public void initAutonomous() {
        super.initAutonomous();
        currentState = STATE_moveToStones;
    }

    /**
     * In the Red Stone Autonomous mode,
     * We start our scan from the the Left wall and we strafe right to the end of the wall and back using
     * the left and front sensors to guide us.
     * if we have NOT detected the Skystone by the end of MAX_CYCLES_FOR_FINDING_STONE cycles, we go park under the the Bridge
     */
    protected void scanForStoneState() {

//        moveXInchesFromLeftObject(4.50, 10000, 0.5);
        boolean goingRight          = false;
        boolean endOfStoneWall      = false;
        boolean hittingSideWall     = false;
        int retries                 = 0;

        powerPropulsion(TravelDirection.LEFT, 0.2);

        while (opModeIsActive() && retries < MAX_CYCLES_FOR_FINDING_STONE ) {

            stoneRelativePlacement = vuMark.find();
            botTop.checkAllLimitSwitches();
            endOfStoneWall = getValidDistance(distanceFront) > (DISTANCE_TO_STONEWALL + 4.0);
            hittingSideWall = getValidDistance(distanceLeft) < 8.0;


            if (stoneRelativePlacement == null) {

                // we dont have the targe, we keep csanning for it
                dbugThis("scanForStoneState : Not Found the stone yet");
                dbugThis("Hitting end : " + endOfStoneWall);
                dbugThis("Hitting wall  : " + hittingSideWall);

                if (endOfStoneWall && goingRight ) {
                    powerPropulsion(TravelDirection.LEFT, 0.3);
                    goingRight = false;
                    gotoHeading(0);
                    retries++;
                }

                if (hittingSideWall && !goingRight) {
                    powerPropulsion(TravelDirection.RIGHT, 0.3);
                    goingRight = true;
                    gotoHeading(0);
                    retries++;
                }
            }

            else {

                // We have the target, now we need delta to be able to refine our position
                dbugThis(String.format("scanForStoneState : Found the stone at Pos (in)  : {X, Y} = %.1f, %.1f",
                        stoneRelativePlacement.x, stoneRelativePlacement.y));

                double delta = stoneRelativePlacement.y - CAMERA_TO_CENTER;
                double absDelta = Math.abs(delta);

                if (absDelta <= 0.8) {
                    dbugThis("ALIGNED");
                    currentState = STATE_getCloseEnoughToPickup;
                    return;
                }

                //  Then we slow down to find the middle of the stone
                dbugThis("Slowly looking for middle of Stone");
                dbugThis(String.format("Delta: %.2f", delta));

                // In the robot centric space, a negative delta means that the robot must move left, TOWARD the side wall
                if (delta < 0 && !hittingSideWall) {
                    dbugThis("SITUATION A: Negative delta, going left");
                    powerPropulsion(TravelDirection.LEFT, 0.2);
                    goingRight = false;

                }

                // we are in this situation where we should abandon this stone
                if (delta < 0 && hittingSideWall) {
                    dbugThis("SITUATION B: Negative delta, should go left but are hitting the side wall");
                    powerPropulsion(TravelDirection.RIGHT, 0.3);
                    goingRight = true;
                    justWait(2000);
                }

                // In the robot centric space, a positive delta means that the robot must move right AWAY from the side wall
                if (delta > 0 && !hittingSideWall){
                    dbugThis("SITUATION C: Positive delta, going right");
                    powerPropulsion(TravelDirection.RIGHT, 0.2);
                    goingRight = true;
                }

                // we are in this predicament where we totally ignore delta and start scanning away
                if (delta > 0 && endOfStoneWall){
                    dbugThis("SITUATION D: Positive delta: Shoult go right, but passed the edge of the wall");
                    powerPropulsion(TravelDirection.LEFT, 0.3);
                    goingRight = false;
                    justWait(1000);
                }
            }

            // Abandon the SkyStone, just park under bridge
            if (retries == MAX_CYCLES_FOR_FINDING_STONE) {

                dbugThis("Missed the whole thing, going to bridge");
                moveXInchesFromBackObject(24.0, 10000, 0.4);
//                double toGo = 72.0 - getValidDistance(distanceLeft) - DISTANCE_LEFT_SENSORS;
//                moveRight(toGo, 0.4);
                moveRightToColor(Color.RED, 0.3);
                currentState = STATE_idle;
                return;
            }
        }
        currentState = STATE_idle;
        return;
    }

    protected void travelToBuildSiteState() {
        botBase.setBling(LED_TEAM_COLORS3);
        moveXInchesFromBackObject(12.0, 100000, 0.4);
        gotoHeading(0);
        justWait(500);
        double toGo = 100.0 - getValidDistance(distanceLeft) - DISTANCE_LEFT_SENSORS;
        moveRight(toGo, 0.6);
        currentState = STATE_dropOffStone;
        return;
    }

    protected void dropOffStoneState() {
        botTop.openClaw();
        moveForward(4.0, 0.6);
        moveXInchesFromBackObject(4.0, 10000, 0.5);
        currentState = STATE_parkUnderBridge;
        return;
    }


    protected void travelHomeState() {
        gotoHeading(0);
        moveXInchesFromLeftObject(9.0, 5000, 0.5);
        gotoHeading(0);
        currentState = STATE_moveToStones;
        return;
    }


    // From the build zone
    protected void parkUnderBridgeState() {
        gotoHeading(0);
        moveLeftToColor(Color.RED, 0.4);
        currentState = STATE_done;
        return;
    }
}

