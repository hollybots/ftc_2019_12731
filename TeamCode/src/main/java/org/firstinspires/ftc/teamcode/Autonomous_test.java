package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Test Suite", group="1")
//@Disabled

public class Autonomous_test extends AutonomousOpModesBase {

    protected static final int TEST_idle                       = 0;
    protected static final int TEST_done                       = 1;

    protected static final int TEST_moveLaterally               = 5;


    protected int currentTest                                   = TEST_moveLaterally;

    protected FieldPlacement stoneRelativePlacement             = null;

    // Sounds
    protected BotSounds botSounds = null;

    // alignment camera from center toward the the left is negative (with the selfie side forward)
    protected static final double CAMERA_TO_CENTER               = -1.8;

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

            if ( currentTest == TEST_done ) {
                 break;
            }

            switch (currentTest) {

                case TEST_done:
                    stopMoving();
                    autonomousIdleTasks();
                    break;

                case TEST_moveLaterally:
                    moveFrontAndLaterally();
                    autonomousIdleTasks();
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
    protected void moveFrontAndLaterally() {
    }

}
