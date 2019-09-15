package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Alliance Team 12731", group="1")
public class Autonomous_12731 extends AutonomousOpModesBase {

    // Sounds
    BotSounds botSounds = null;

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
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initAutonomous();

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

        // Enable navigation system
//        navigation.activate();

        FieldPlacement initialPosition = new FieldPlacement(0,0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            dbugThis(isStalled() + "");

            moveForwardByTime(3000);
            justWait(1000);
            moveBackwardByTime(3000);
            botSounds.play("ss_bb8_up");
            justWait(1000);
            moveLeftByTime(3000);
            justWait(1000);
            moveRightByTime(3000);
            botSounds.play("ss_r2d2_down");
            justWait(1000);
            break;
        }

        // Disable navigation system
        navigation.stop();
        stopMoving();
    }
}
