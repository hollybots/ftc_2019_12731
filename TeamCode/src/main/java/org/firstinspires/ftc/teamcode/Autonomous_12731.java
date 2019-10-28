package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Base Team 12731", group="1")
@Disabled

public class Autonomous_12731 extends AutonomousOpModesBase {

    protected static final int STATE_idle                       = 0;

    protected static final int STATE_moveToStones               = 1;
    protected static final int STATE_scanForStone               = 2;
    protected static final int STATE_alignWithStone             = 3;
    protected static final int STATE_pickUpStone                = 4;
    protected static final int STATE_travelToBuildSite          = 5;
    protected static final int STATE_parkUnderBridge            = 6;

    protected static final int STATE_getCloseEnoughToPickup     = 7;
    protected static final int STATE_dropOffStone               = 8;
    protected static final int STATE_travelHome                 = 9;

    protected static final int STATE_moveToTray                 = 20;
    protected static final int STATE_clampTray                  = 21;
    protected static final int STATE_moveTrayBack               = 22;



    protected static final int STATE_done                       = 50;

    protected int currentState                                  = STATE_idle;

    protected FieldPlacement stoneRelativePlacement             = null;



//    protected CRServo slide                               = null;
//    protected Servo claw                                  = null;
//    protected Servo rightPin                              = null;
//    protected Servo leftPin                               = null;

    // Sounds
    protected BotSounds botSounds = null;

    // alignment
    protected static final double CAMERA_TO_CENTER               = 0.0;

    protected static final double BLING_MODE_CLAMP               = 0.6545;


    @Override
    public void initAutonomous() {

        DEBUG = true;
        super.initAutonomous();


        /* **********************************
           LIGHTS
        */
        botBase.setBling(0.7745);
    }

    @Override
    public void runOpMode() {

        initAutonomous();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        setCameraVerticalPosition(0.35);


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

                case STATE_getCloseEnoughToPickup:
                    getCloseEnoughToPickUpState();
                    break;

                case STATE_pickUpStone:
                    pickUpStoneState();
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
                    parkUnderBridgeState();
                    break;

                case STATE_moveToTray:
                    moveToTrayState();
                    break;

                case STATE_clampTray:
                    clampTrayState();
                    break;

                case STATE_moveTrayBack:
                    moveTrayBackState();
                    break;

            }
            telemetry.update();
        }

        vuMark.stop();
        stopMoving();
    }

    /**
     * We need to move close enough to be able to see the stone
     */
    protected void moveToStoneState() {

        botTop.slideUp();
        moveXInchesFromFrontObject(11.0, 10000, 1.0);
        currentState = STATE_scanForStone;
        return;
    }


    /**
     * Here we scan left an right to find a stone.  One the stone is found
     * we go to the next state
     */
    protected void scanForStoneState() {
        // implement in alliance specific
    }


    protected void alignWithStoneState() {

        if ( stoneRelativePlacement == null ) {
            currentState = STATE_done;
            dbugThis("Failed");
            return;
        }
//
//        else if (true) {
//            currentState = STATE_done;
//            dbugThis("Last known position : "  +  stoneRelativePlacement.y );
//            return;
//        }

        // This is the distance from the camera to the actual part of the robot that must align with the center of the Vumark
        double offset = CAMERA_TO_CENTER;


        double delta = stoneRelativePlacement.y - offset;
        double absDelta = Math.abs(delta);

        if (absDelta < 1.0) {
            currentState = STATE_getCloseEnoughToPickup;
            dbugThis("Found the bugger!!");
            return;
        }

        // First we do a gross movement just to get closer
        if (delta < 0) {
            moveLeftByTime(timeToMoveInMs(absDelta), 0.9);
        } else {
            moveRightByTime(timeToMoveInMs(absDelta), 0.9);
        }
        stopMoving();

        //
        //  Then we slow down to find the middle of the stone
        dbugThis("Slowly looking for middle of Stone");
        stoneRelativePlacement = vuMark.find();
        if (stoneRelativePlacement == null) {
            currentState = STATE_done;
            dbugThis("Failed");
            return;
        }

        delta = stoneRelativePlacement.y - offset;
        absDelta = Math.abs(delta);
        dbugThis("New Delta: " + delta);

        while (absDelta > 1.0 && opModeIsActive()) {

            if (delta < 0) {
                powerPropulsion(TravelDirection.LEFT, 0.3);
            } else {
                powerPropulsion(TravelDirection.RIGHT, 0.3);
            }
            justWait(1000);
            stoneRelativePlacement = vuMark.find();
            if (stoneRelativePlacement == null) {
                currentState = STATE_done;
                dbugThis("Failed");
                return;
            }
            delta = stoneRelativePlacement.y - offset;
            absDelta = Math.abs(delta);

            dbugThis("New Delta: " + delta);
        }
        currentState = STATE_getCloseEnoughToPickup;
        return;
    }


    protected void getCloseEnoughToPickUpState() {
        moveXInchesFromFrontObject(2.0, 10000, 0.5);
        currentState = STATE_pickUpStone;

        return;
    }

    protected void pickUpStoneState() {
        botTop.stopSlide();
        botTop.slideUp();
        botTop.closeClaw();
        justWait(500);
        botTop.stopSlide();
        currentState = STATE_travelToBuildSite;
//        currentState = STATE_idle;
        return;
    }

    protected void travelToBuildSiteState() {
        // implement in alliance specific
    }


    protected void dropOffStoneState() {
        gotoHeading(0);
        botTop.openClaw();
        moveXInchesFromBackObject(12.0, 100000, 1.0);
        botTop.closeClaw();
        botTop.openClaw();
        slideByTime(2000, 1.0);
        currentState = STATE_parkUnderBridge;
        return;
    }

    protected void travelHomeState() {
        // implement in alliance specific
    }


    protected void moveToTrayState() {
        // implement in alliance specific
    }

    protected void clampTrayState() {

        botTop.clampOn();
        botBase.setBling(BLING_MODE_CLAMP);
        currentState = STATE_moveTrayBack;
    }


    protected void moveTrayBackState() {

        moveXInchesFromBackObject(7.0, 100000, 0.8);
        botTop.clampRelease();
        currentState = STATE_parkUnderBridge;
    }

    protected void parkUnderBridgeState() {
        // implement in alliance specific
    }


    public void slideByTime(int ms, double power) {

        double limit = runtime.milliseconds() + ms;

        botTop.getSlide().setPower(power);
        while (opModeIsActive() &&  runtime.milliseconds() < limit) {
            idle();
        }
        botTop.getSlide().setPower(0.0);
    }
}
