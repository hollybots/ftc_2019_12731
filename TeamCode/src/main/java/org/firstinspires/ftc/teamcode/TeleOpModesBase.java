package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Teleop mode.
 */
@TeleOp(name="TeleOp Base class", group="none")
@Disabled
public class TeleOpModesBase extends OpMode {

    protected BotBase botBase;
    protected BotTop  botTop;

    Servo camera_pan_vertical = null;
    double cameraPanVerticalPosition        = 0;

    protected boolean DEBUG = true;


    // Timekeeper OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();



    /**
     * Initializes the robot.
     * Called once before the match when the "Init" button is pressed.
     */
    @Override
    public void init() {

        botBase     = new BotBase(hardwareMap);
        botTop      = new BotTop(hardwareMap);

        try {
            camera_pan_vertical = hardwareMap.get(Servo.class, "camera_pan_vertical");
        } catch (Exception $e) {
            camera_pan_vertical = null;
        }

        setCameraVerticalPosition(0.6);
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

    void dbugThis(String s) {

        if ( DEBUG == true ) {
            Log.d("TELEOP: ", s);
        }
    }


    public void setCameraVerticalPosition(double position) {
        if (camera_pan_vertical == null) {
            return;
        }
        position = Math.min(Math.max(0, position), 1.0);
        cameraPanVerticalPosition = position;
        camera_pan_vertical.setPosition(position);
    }



    // Computes the current battery voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}
