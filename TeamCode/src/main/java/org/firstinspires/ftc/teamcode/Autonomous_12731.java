package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;

@Autonomous(name="Base Team 12731", group="none")
@Disabled

public class Autonomous_12731 extends AutonomousOpModesBase {

    protected static final int STATE_idle                       = 0;

    protected static final int STATE_moveToStones               = 1;
    protected static final int STATE_scanForStone               = 2;
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

    // Sounds
    protected BotSounds botSounds = null;

    // alignment camera from center toward the the left is negative (with the selfie side forward)
    protected static final double CAMERA_TO_CENTER               = -3;

    protected static final int MAX_CYCLES_FOR_FINDING_STONE      = 3;

    protected static final double BLING_MODE_CLAMP               = LED_TEAM_COLORS4;
    protected static final double DISTANCE_TO_STONEWALL          = 12.0;


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

        /*********************************************
         * WAIT FOR START
         * *******************************************/

        waitForStart();
        runtime.reset();

        /*********************************************
         * GAME IS ON !!
         * *******************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            autonomousIdleTasks();

            if ( currentState == STATE_done ) {
                 break;
            }

            switch (currentState) {

                case STATE_idle:
                    stopMoving();
                    autonomousIdleTasks();
                    break;

                case STATE_moveToStones:
                    moveToStoneState();
                    break;

                case STATE_scanForStone:
                    scanForStoneState();
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

        dbugThis("================================= NEW TRY ====================================================");
        botTop.swing(BotTop.SWING_UP_COMMAND, false);
        autonomousIdleTasks();
        moveXInchesFromFrontObject(DISTANCE_TO_STONEWALL, 10000, 0.2);
        autonomousIdleTasks();
        botTop.openClaw();
//        autonomousIdleTasks();
//        justWait(1000);
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



    protected void getCloseEnoughToPickUpState() {
        botTop.slideDown();
        dbugThis("getCloseEnoughToPickUpState");
//        moveXInchesFromFrontObject(0, 4000, 0.2);
        moveForwardByTime(2000, 0.2);
        currentState = STATE_pickUpStone;
//        currentState = STATE_idle;
        return;
    }

    protected void pickUpStoneState() {
        dbugThis("pickUpStoneState");
        botTop.stopSlide();
        botTop.closeClaw();
        justWait(500);
        // slide up
        slideByTime(500,  botTop.POWER_SLIDE);
        botTop.swing(BotTop.SWING_DOWN_COMMAND, false);
        currentState = STATE_travelToBuildSite;
        return;
    }

    protected void travelToBuildSiteState() {
        // implement in alliance specific
    }


    protected void dropOffStoneState() {

    }

    protected void travelHomeState() {
        // implement in alliance specific
    }


    protected void moveToTrayState() {
        // implement in alliance specific
    }

    protected void clampTrayState() {

        botTop.clampOn();
        justWait(500);
        botBase.setBling(BLING_MODE_CLAMP);
        currentState = STATE_moveTrayBack;
    }


    protected void moveTrayBackState() {
        // implement in alliance specific
    }


    protected void parkUnderBridgeState() {
        // implement in alliance specific
    }

}
