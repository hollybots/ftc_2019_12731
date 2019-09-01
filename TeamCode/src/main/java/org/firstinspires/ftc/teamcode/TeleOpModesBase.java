package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Teleop mode.
 */
@TeleOp(name="TeleOpModes Base", group="TeleOpModes")
@Disabled
public class TeleOpModesBase extends OpMode implements AllOpModesUtilities {

    private BotBase botBase;

    /**
     * Initializes the robot.
     * Called once before the match when the "Init" button is pressed.
     */
    @Override
    public void init() {

        botBase.init(hardwareMap);

        botBase.getFrontLeftDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        botBase.getFrontRightDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getFrontRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        botBase.getRearLeftDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getRearLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        botBase.getRearRightDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getRearRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Main loop function.
     * Called repeatedly during the match after the "Play" button is pressed.
     */
    @Override
    public void loop() {
    }


    /**
     * Stops the robot.
     * Called once at the end of the match when time runs out.
     */
    @Override
    public void stop() {

        botBase.stop();
    }

}
