package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Red Alliance Stone - 12731", group="1")
public class Autonomous_12731_red_stone extends Autonomous_12731 {

    @Override
    public void initAutonomous() {

        super.initAutonomous();
        currentState = STATE_moveToStones;
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }


    /**
     * Here we scan left an right to find a stone.  One the stone is found
     * we go to the next state
     */
    protected void scanForStoneState() {

        stoneRelativePlacement = navigation.getSkyStone("Stone Target");
        if (stoneRelativePlacement != null ) {
            dbugThis("Stopping all motion");
            stopMoving();
            currentState = STATE_alignWithStone;
            dbugThis(String.format("Pos (in)  : {X, Y} = %.1f, %.1f",
                    stoneRelativePlacement.x, stoneRelativePlacement.y));
            return;
        }

        moveXInchesFromLeftObject(9.0, 10000, 0.5);
        boolean goingRight = false;

        while (opModeIsActive() && stoneRelativePlacement == null) {

            stoneRelativePlacement = navigation.getSkyStone("Stone Target");
            if (stoneRelativePlacement != null) {
                stopMoving();
                currentState = STATE_alignWithStone;
                dbugThis(String.format("Pos (in)  : {X, Y} = %.1f, %.1f",
                        stoneRelativePlacement.x, stoneRelativePlacement.y));
                return;
            }
            if ( goingRight ) {
                gotoHeading(0);
                moveLeftByTime(6000, 0.4);
            } else {
                gotoHeading(0);
                moveRightByTime(6000, 0.4);
            }

            goingRight = !goingRight;
            telemetry.update();
        }

        return;
    }


    protected void travelToBuildSiteState() {
        setCameraVerticalPosition(0.55);
        moveXInchesFromBackObject(8.0, 100000, 0.8);
        gotoHeading(0);
//        moveRightByTime(6000, 0.8);
        moveXInchesFromRightObject(30.0, 100000, 0.9);
//        moveRightByTime(6000, 0.8);
        currentState = STATE_dropOffStone;
        return;
    }


    protected void travelHomeState() {
        gotoHeading(0);
        moveXInchesFromLeftObject(9.0, 5000,0.8);
        gotoHeading(0);
        currentState = STATE_moveToStones;
        return;
    }



    protected void parkUnderBridgeState() {
        gotoHeading(0);
        moveXInchesFromBackObject(12.0, 5000,0.9);
        moveLeftByTime(2000, 0.9);
        currentState = STATE_done;
        return;
    }

}
