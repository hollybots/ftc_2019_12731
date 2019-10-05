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

    protected FieldPlacement stoneRelativePlacement       = null;
    protected CRServo slide                               = null;
    protected Servo claw                                  = null;
    protected Servo rightPin                              = null;
    protected Servo leftPin                              = null;

    // Sounds
    protected BotSounds botSounds = null;

    @Override
    public void initAutonomous() {

        DEBUG = true;
        super.initAutonomous();

        /**
         * Add everything that is NOT Sensors or Propulsion motors
         */

        /**
         * SOUNDS
         */
        botSounds = new BotSounds(hardwareMap);

        /**
         * CLAW Mechanism
         */
        slide               = hardwareMap.get(CRServo.class, "slide");
        claw                = hardwareMap.get(Servo.class, "claw");

        /**
         * TRAY HOOK Mechanism
         */
        rightPin                = hardwareMap.get(Servo.class, "claw");
        leftPin                = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void runOpMode() {

        initAutonomous();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Enable navigation system
        navigation.activate();

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

        navigation.stop();
        stopMoving();
    }

    /**
     * We need to move close enough to be able to see the stone
     */
    protected void moveToStoneState() {

        openClaw();
        slideByTime(1000, 0.8);
        moveXInchesFromFrontObject(13.0, 10000, 0.9);
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

        double offset = 0.0;
        stopMoving();
        dbugThis("Slowly looking for Stone");

        int x = 0;

        while (opModeIsActive()) {

            x++;
            stoneRelativePlacement = navigation.getSkyStone("Stone Target");
            if ( stoneRelativePlacement == null ) {
                continue;
            }

            if ( Math.abs(stoneRelativePlacement.y - offset) < 2.0  ) {
                break;
            }

            if (Math.abs(stoneRelativePlacement.y - offset) > 35.0) {
                break;
            }

            if (stoneRelativePlacement.y - offset < 0.0) {
               dbugThis("Position Y : " + stoneRelativePlacement.y + ", going LEFT");
                powerPropulsion(TravelDirection.LEFT, 0.25);
            } else if (stoneRelativePlacement.y - offset > 0.0) {
                dbugThis("Position Y : " + stoneRelativePlacement.y + ", going RIGHT");
                powerPropulsion(TravelDirection.RIGHT, 0.25);
            }
            justWait(200);
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


    protected void getCloseEnoughToPickUpState() {
        moveXInchesFromFrontObject(4.0, 10000, 0.5);
        currentState = STATE_pickUpStone;
        return;
    }

    protected void pickUpStoneState() {
        slideByTime(2000, -0.8);
        closeClaw();
        slideByTime(1000, 0.8);

        currentState = STATE_travelToBuildSite;
//        currentState = STATE_idle;
        return;
    }

    protected void travelToBuildSiteState() {
        // implement in alliance specific
    }


    protected void dropOffStoneState() {
        gotoHeading(0);
        openClaw();
        moveXInchesFromBackObject(7.0, 100000, 0.8);
        closeClaw();
        openClaw();
        slideByTime(1000, 0.8);
        currentState = STATE_parkUnderBridge;
        return;
    }

    protected void travelHomeState() {
        // implement in alliance specific
    }



    protected void openClaw() {
        claw.setPosition(0);
    }


    protected void moveToTrayState() {
        // implement in alliance specific
    }

    protected void clampTrayState() {

        rightPin.setPosition(1.0);
        leftPin.setPosition(1.0);
        currentState = STATE_moveTrayBack;
    }


    protected void moveTrayBackState() {

        moveXInchesFromBackObject(7.0, 100000, 0.8);
        rightPin.setPosition(0.0);
        leftPin.setPosition(0.0);
        currentState = STATE_parkUnderBridge;
    }

    protected void parkUnderBridgeState() {
        // implement in alliance specific
    }


    protected void closeClaw() {
        claw.setPosition(0.9);
    }

    protected void slideByTime(int ms, double power) {

        double limit = runtime.milliseconds() + ms;

        slide.setPower(power);
        while (opModeIsActive() &&  runtime.milliseconds() < limit) {
            idle();
        }
        slide.setPower(0.0);
    }


}
