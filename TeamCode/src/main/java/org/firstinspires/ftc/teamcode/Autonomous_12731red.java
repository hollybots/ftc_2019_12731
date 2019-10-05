package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Red Alliance Team 12731", group="1")
public class Autonomous_12731red extends AutonomousOpModesBase {
    private Servo servo = null;
    private DcMotor Coil = null;
    Servo servoClaw = null;
    CRServo servoLinear = null;
    Servo servoCamera = null;
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
        //servos
        servoLinear  = hardwareMap.get(CRServo.class, "servoLinear");
        servoClaw  = hardwareMap.get(Servo.class, "servoClaw");
        servoCamera = hardwareMap.get(Servo.class,"camera_pan");

        servoClaw.setPosition(0);
        servoCamera.setPosition(4);
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
            moveForwardByTime(100);
            moveRightByTime(1850);
            break;
        }

        // Disable navigation system
        navigation.stop();
        stopMoving();
    }
}


