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

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: All OpModes 14877", group="Linear Opmode")
@Disabled
public class OpModesBase extends LinearOpMode {

    public enum TravelDirection {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
    }

    /** Injected values : See the child Red, Blue Extended OpModes **/

    // Will dump debug information in the LogCat if true
    protected boolean DEBUG                             = false;


    /**
     * DC Motors Constants
     */
    protected static final double TORQUENADO_COUNTS_PER_MOTOR_REV        = 1440;                 // eg: REV Motor Encoder
    protected static final double NEVEREST40_COUNTS_PER_MOTOR_REV        = 1120;                 // eg: REV Motor Encoder

    protected static final int TURN_DIRECTION_LEFT                = 1;  // this is a direction for
    protected static final int TURN_DIRECTION_RIGHT               = -1;
    protected static final int ERROR_POSITION_COUNT               = 10;

    protected static final double DRIVE_SPEED                     = 0.9;

    protected static final double TURNING_SPEED                   = 0.3;

    protected static final double PROPULSION_DRIVE_GEAR_REDUCTION     = 1.3;                  // This is < 1.0 if geared UP > 1 we are gering down (the small drives the big)
    protected static final double WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;        // For figuring circumference
    protected static final double PROPULSION_ENCODER_COUNTS_PER_INCH  = (TORQUENADO_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;


    protected static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
     P_TURN_COEFF * TURNING_SPEED must be > 0.1
     ************************************************************************* */
    protected static final double     P_TURN_COEFF            = 0.5;     // Larger is more responsive, but also less stable
    protected static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    // Timekeeper OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();

    // Robot Hardware
    protected DcMotor frontLeftDrive     = null;
    protected DcMotor frontRightDrive    = null;
    protected DcMotor rearLeftDrive     = null;
    protected DcMotor rearRightDrive    = null;



    @Override
    public void runOpMode() {}


    /*********************************************
     * INIT
     * *******************************************/

    /**
     * initRobot()
     *
     *    Configure the Hardware according to the Team Hardware Spreadsheet.
     */
    protected void initRobot() {


        /* ************************************
            DC MOTORS
        */
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive  = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);


        /* ***********************************
            SERVOS
         */



    }


    /**
     * justWait()
     *
     * Just like that.  This function does nothing but wait while allowing to other processes to run
     *
     * @param ms       : any number of milliseconds.  Can be less than 1
     *
     */
    protected void justWait(double ms) {

        ms = Math.abs(ms);
        double limit = runtime.milliseconds() + ms;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && runtime.milliseconds() < limit  ) {
            idle();
            telemetry.update();
        }
    }



    /**
     * dbugThis()
     *
     * Writes a string in the LogCat windows under the "FIRST" tag
     *
     * @param s             : String to write
     */
    protected void dbugThis(String s) {

        if ( this.DEBUG == true ) {
            Log.d("OpModesBaseClass: ", s);
        }
    }
}

