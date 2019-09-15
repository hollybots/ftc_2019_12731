package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous | Sensor Navigation", group="1")
//@Disabled
public class Autonomous_12731 extends AutonomousOpModesBase {


    @Override
    public void initAutonomous() {
        super.initAutonomous();
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
        navigation.activate();

        FieldPlacement initialPosition = new FieldPlacement(0,0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            gotoPlacement(initialPosition);
            break;
        }

        // Disable navigation system
        navigation.stop();
        stopMoving();
    }
}
