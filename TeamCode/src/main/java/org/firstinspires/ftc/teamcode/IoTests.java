/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.WriteXMLFileHandler;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.ftccommon.R;

import java.util.ArrayList;
import java.util.List;


/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="IO Tests", group="1")
//@Disabled
public class IoTests extends OpMode
{
    /***
     * OUTPUTS (will be connected to the gamepad)
     */

    // DC Motors
    List<DcMotor> motors;

    // Servo
    List<Servo> servos;

    /**
     * INPUTS
     */

    // Range Sensors
    List<ModernRoboticsI2cRangeSensor> i2cs;

    // Digital Inputs
    List<DigitalChannel> dinputs;


    /**
     * State variables
     */
    private int selectingServoMode = 0;
    private Boolean left_bumper_was_down = false;
    private Boolean right_bumper_was_down = false;
    private double servo_current_value = 0.0;
    private int current_servo_selection = 0;

    private int selectingI2cMode = 0;
    private int current_i2c_selection = 0;
    public double current_distance = -1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        servos       = new ArrayList();
        /*********************************************
         * Configure Servos
         */
        for (int i=0; i<=5; i++) {
            try {
                servos.add(hardwareMap.get(Servo.class, "servo" + i));
            }
            catch (Exception e) {
                Log.d("TEST_IO: ", "Cannot intialize servo" + i);
            }
        }

        motors       = new ArrayList();
        for (int i=0; i<=3; i++) {
            try {
                motors.add(hardwareMap.get(DcMotor.class, "motor" + i));
                motors.get(i).setDirection(DcMotor.Direction.FORWARD);
                motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            catch (Exception e) {
                Log.d("TEST_IO: ", "Cannot intialize motor" + i);
            }
        }

        // Map all the I2C - Distance Sensors
        i2cs       = new ArrayList();
        for (int i=0; i<=3; i++) {
            try {
                i2cs.add(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "i2c" + i));
            } catch (Exception e) {
                Log.d("TEST_IO: ", "Cannot intialize i2c" + i);
            }
        }

    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /**
         * INPUT SENSORS
         */
        i2cSelection();
        if (selectingI2cMode == 0 ) {
            double distance = i2cs.get(current_i2c_selection).getDistance(DistanceUnit.INCH);
            if (distance != current_distance) {
                Log.d("TEST_IO: ", String.format("Distance sensor %d: %.4f", current_i2c_selection, distance));
                current_distance = distance;
            }
        }

        /**
         * INPUT GAMEPAD
         */
        double motor0_command                  = -gamepad1.right_stick_y;
        double motor1_command                   = gamepad1.right_stick_x;
        double motor2_command                  = -gamepad1.left_stick_y;
        double motor3_command                   = gamepad1.left_stick_x;

        servoSelection();
        if (selectingServoMode == 0) {

            // servo are incremented by 0.1 each time the bumper button is pressed right -> + left -> -
            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                left_bumper_was_down = true;
            }
            else if (left_bumper_was_down && !gamepad1.left_bumper) {
                servo_current_value = Math.max(0.0, servo_current_value - 0.1);
                servos.get(current_servo_selection).setPosition(servo_current_value);
                Log.d("TEST_IO: ", String.format("Setting Servo %d to %.2f", current_servo_selection, servo_current_value ));
                left_bumper_was_down = false;
            }
            else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                right_bumper_was_down = true;
            }
            else if (right_bumper_was_down && !gamepad1.right_bumper) {
                servo_current_value = Math.min(0.0, servo_current_value + 0.1);
                servos.get(current_servo_selection).setPosition(servo_current_value);
                Log.d("TEST_IO: ", String.format("Setting Servo %d to %.2f", current_servo_selection, servo_current_value ));
                right_bumper_was_down = false;
            }
        }


        /**
         * OUTPUT PROPULSION
         */
        if (selectingI2cMode == 0) {
            // Send calculated power to wheels
            motors.get(0).setPower(motor0_command);
            motors.get(1).setPower(motor1_command);
            motors.get(2).setPower(motor2_command);
            motors.get(3).setPower(motor3_command);
        }

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }


    private void servoSelection()
    {
        if (selectingServoMode == 0) {
            // to enter servo selection mode, press 2 bumpers at the same time
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                Log.d("TEST_IO: ", "================= Entering SELECTING servo mode ============");
                selectingServoMode = 1;
            }
            return;
        }

        if (selectingServoMode == 1) {
            // To start selecting, release both bumpers
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                selectingServoMode = 2;
            }
            return;
        }

        if (selectingServoMode == 2) {
            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                left_bumper_was_down = true;
                return;
            }
            if (!gamepad1.left_bumper && left_bumper_was_down ) {
                current_servo_selection = Math.max(current_servo_selection - 1, 0);
                Log.d("TEST_IO: ", "Current Servo: " + current_servo_selection);
                left_bumper_was_down = false;
                return;
            }
            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                right_bumper_was_down = true;
                return;
            }
            if (right_bumper_was_down && !gamepad1.right_bumper) {
                current_servo_selection = Math.max(current_servo_selection + 1, 0);
                Log.d("TEST_IO: ", "Current Servo: " + current_servo_selection);
                right_bumper_was_down = false;
                return;
            }

            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                selectingServoMode = 3;
                return;
            }
        }

        if (selectingServoMode == 3) {
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                Log.d("TEST_IO: ", "================= Leaving SELECTING servo mode ============");
                selectingServoMode = 4;
                servo_current_value = 0;
                servos.get(current_servo_selection).setPosition(servo_current_value);
            }
            return;
        }

        if (selectingServoMode == 4) {
            // Just we can skip a loop
            selectingServoMode = 0;
            return;
        }
    }


    private void i2cSelection()
    {
        if (selectingI2cMode == 0) {
            // to enter servo selection mode, press 2 bumpers at the same time
            if (gamepad1.left_stick_button) {
                Log.d("TEST_IO: ", "================= Entering SELECTING i2c mode ============");
                selectingI2cMode = 1;
            }
            return;
        }

        if (selectingI2cMode == 1) {
            // To start selecting, release stick button
            if (!gamepad1.left_stick_button) {
                selectingI2cMode = 2;
            }
            return;
        }

        if (selectingI2cMode == 2) {
            if (gamepad1.left_stick_y > 0) {
                current_i2c_selection = 0;
                return;
            }
            if (gamepad1.left_stick_y < 0) {
                current_i2c_selection = 1;
                return;
            }
            if (gamepad1.left_stick_x < 0) {
                current_i2c_selection = 2;
                return;
            }
            if (gamepad1.left_stick_x > 0) {
                current_i2c_selection = 3;
                return;
            }
            if (gamepad1.left_stick_button) {
                selectingI2cMode = 3;
            }
            return;
        }

        if (selectingI2cMode == 3) {
            if (!gamepad1.left_stick_button) {
                Log.d("TEST_IO: ", "================= Leaving SELECTING i2c mode ============");
                selectingI2cMode = 4;
            }
            return;
        }

        if (selectingI2cMode == 4) {
            // Just we can skip a loop
            selectingI2cMode = 0;
            return;
        }
    }

//
//    private void loadTestConfigurationFile() {
//
//        RobotConfigFileManager cfgFileMgr = new RobotConfigFileManager();
//        AppUtil appUtil = AppUtil.getInstance();
//        this.context = appUtil.getApplication();
//        this.preferences = PreferenceManager.getDefaultSharedPreferences(context);
//        RobotConfigFile file;
//
//
////        String key = context.getString(R.string.pref_hardware_config_filename);
////        String objSerialized;
//
//
////        Log.d("TEST_IO: ", "Key " + key );
////        objSerialized = preferences.getString(key, null);
////        Log.d("TEST_IO: ", "objSerialized " + objSerialized );
//
//        file = RobotConfigFile.fromString(cfgFileMgr, "{\"isDirty\":false,\"location\":\"LOCAL_STORAGE\",\"name\":\"io_test\",\"resourceId\":0}");
//        cfgFileMgr.setActiveConfig(file);
//    }
}
