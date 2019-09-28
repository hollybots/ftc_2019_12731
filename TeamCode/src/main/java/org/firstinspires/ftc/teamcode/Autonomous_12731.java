package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Alliance Team 12731", group="1")
public class Autonomous_12731 extends AutonomousOpModesBase {

    protected static final int STATE_idle                       = 0;

    protected static final int STATE_moveToStones               = 1;
    protected static final int STATE_scanForStone               = 2;
    protected static final int STATE_alignWithStone             = 3;
    protected static final int STATE_pickUpStone                = 4;
    protected static final int STATE_travelToBuildSite          = 5;
    protected static final int STATE_parkUnderBridge            = 6;
    protected static final int STATE_done                       = 10;
    protected static final int STATE_getCloseEnoughToPickup     = 7;
    protected static final int STATE_dropOffStone               = 8;
    protected static final int STATE_travelHome                 = 9;

    private int currentState                                    = STATE_moveToStones;
    FieldPlacement BRIDGE_PLACEMENT                             = null;


    FieldPlacement stoneRelativePlacement = null;


    // Sounds
    BotSounds botSounds = null;

    @Override
    public void initAutonomous() {

        DEBUG = true;
        BRIDGE_PLACEMENT       = new FieldPlacement(0, -46);

        super.initAutonomous();
        justWait(2000);

        /**
         * Add everything that is NOT Sensors or Propulsion motors
         */
        /**
         * SOUNDS
         */
        botSounds = new BotSounds(hardwareMap);

    }

    @Override
    public void runOpMode() {

        initAutonomous();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Enable navigation system
        navigation.activate();

        /*********************************************
         * WAIT FOR START
         * *******************************************/

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Started!");
        telemetry.update();


        /*********************************************
         * GAME IS ON !!
         * *******************************************/

        FieldPlacement initialPosition = new FieldPlacement(0,0);
        setCameraVerticalPosition(0.65);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if ( currentState == STATE_done ) {
                 break;
            }

            switch (currentState) {

                case STATE_idle:
                    stopMoving();
                    idle();
                    break;

                case STATE_moveToStones:
                    moveToStoneState();
                    break;

                case STATE_scanForStone:
                    scanForStoneState();
                    break;

                case STATE_alignWithStone:
                    alignWithStoneState();
                    break;

                case STATE_pickUpStone:
                    pickUpStoneState();
                    break;

                case STATE_getCloseEnoughToPickup:
                    getCloseEnoughToPickUpState();
                    break;

                case STATE_travelToBuildSite:
                    travelToBuildSiteState();
                    break;

                case STATE_dropOffStone:
                    dropOffStoneState();
                    break;

                case STATE_travelHome:
                    travelHomeState();
                    break;

                case STATE_parkUnderBridge:
                    parkUnderBridge();
                    break;

            }
            telemetry.update();
        }

        navigation.stop();
        stopMoving();
    }

    /**
     * We need to move close enough to be able to see the stone
     */
    private void moveToStoneState() {

        setCameraVerticalPosition(0.55);
        setCameraHorizontalPosition(0.5);
        moveXInchesFromFrontObject(14.0, 10000, 0.9);
        currentState = STATE_scanForStone;
        return;
    }


    /**
     * Here we scan left an right to find a stone.  One the stone is found
     * we go to the next state
     */
    private void scanForStoneState() {


        stoneRelativePlacement = navigation.getSkyStone("Stone Target");
        if (stoneRelativePlacement != null ) {
            gotoHeading(0);
            currentState = STATE_alignWithStone;
            return;
        }

        moveXInchesFromLeftObject(9.0, 10000, 0.5);

        boolean goingRight = false;

        while (opModeIsActive() && stoneRelativePlacement == null) {

            stoneRelativePlacement = navigation.getSkyStone("Stone Target");
            if (stoneRelativePlacement != null) {
                stopMoving();
                break;
            }
            if ( goingRight ) {
                gotoHeading(0);
                moveLeftByTime(4000, 0.4);
            } else {
                gotoHeading(0);
                moveRightByTime(4000, 0.4);
            }

            goingRight = !goingRight;
            telemetry.update();
        }
        gotoHeading(0);
        currentState = STATE_alignWithStone;
        return;
    }



    private void alignWithStoneState() {

        stoneRelativePlacement = navigation.getSkyStone("Stone Target");

        while (stoneRelativePlacement != null && Math.abs(stoneRelativePlacement.y) > 1.0 && opModeIsActive() ) {
            if (stoneRelativePlacement.y < 35.0) {
                if (stoneRelativePlacement.y > 0) {
                    dbugThis("Position Y : " + stoneRelativePlacement.y + ", going LEFT");
                    powerPropulsion(TravelDirection.LEFT, 0.2);
                } else if (stoneRelativePlacement.y < 0) {
                    dbugThis("Position Y : " + stoneRelativePlacement.y + ", going RIGHT");
                    powerPropulsion(TravelDirection.RIGHT, 0.2);
                }
            }
            stoneRelativePlacement = navigation.getSkyStone("Stone Target");
        }
        stopMoving();
        gotoHeading(0);

        if (stoneRelativePlacement == null) {
            currentState = STATE_scanForStone;
            return;
        }

        currentState = STATE_getCloseEnoughToPickup;
        return;
    }


    private void getCloseEnoughToPickUpState() {
        moveXInchesFromFrontObject(6.0, 10000, 0.5);
        currentState = STATE_pickUpStone;
        return;
    }


    private void pickUpStoneState() {
        justWait(5000);
        currentState = STATE_travelToBuildSite;
        return;
    }


    private void travelToBuildSiteState() {
        setCameraHorizontalPosition(0);
        setCameraVerticalPosition(0.65);
        moveXInchesFromBackObject(7.0, 100000, 0.8);
        gotoHeading(0);
        moveRightByTime(6000, 0.8);
        currentState = STATE_dropOffStone;
        return;
    }


    private void dropOffStoneState() {
        gotoHeading(0);
        justWait(5000);
        currentState = STATE_travelHome;
        return;
    }

    private void travelHomeState() {
        gotoHeading(0);
        moveLeftByTime(6000, 0.9);
        moveXInchesFromLeftObject(6.0, 5000,0.8);
        gotoHeading(0);
        currentState = STATE_moveToStones;
        return;
    }


    private void parkUnderBridge() {

    }
}
