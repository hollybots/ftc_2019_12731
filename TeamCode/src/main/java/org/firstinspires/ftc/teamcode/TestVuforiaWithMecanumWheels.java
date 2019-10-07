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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Test Mecanum & Vuforia", group="3")
//@Disabled
public class TestVuforiaWithMecanumWheels extends TeleOpModesBase
{
    static final double     AUTONOMOUS_SPEED            = 0.6;
    static final double     K                           = 0.2;
    private double          theta                       = 0;   // gyro angle.  For field centric autonomous mode we will use this to orient the robot


    // VuForia Key, register online
    protected static final String VUFORIA_KEY = "AXINfYT/////AAAAGfcLttUpcU8GheQqMMZAtnFDz/qRJOlHnxEna51521+PFcmEWc02gUQ1s4DchmXk+fFvt+afRNF+2UoUgoAyQNtfVjRNS0u4f5o4kka/jERVEtKlJ27pO4euCEjE1DQ+l8ecADKTd1aWu641OheSf/RqDJ7BSvDct/PYRfRLfShAfBUxaFT3+Ud+6EL31VTmZKiylukvCnHaaQZxDmB2cCDdYFeK2CDwNIWoMx2VvweehNARttNvSR3cp4AepbtWnadsEnDQaStDv8jN09iE7CRWmMY8rrP8ba/O/eVlz0vzU7Fhtf2jXpSvCJn0qDw+1UK/bHsD/vslhdp+CBNcW7bT3gNHgTOrnIcldX2YhgZS";

    // Vuforia translation from the the robot center where x -> front, y -> left and  z -> up
    protected static final int CAMERA_FORWARD_DISPLACEMENT        = 150;   // eg: Camera is 150 mm in front of robot center
    protected static final int CAMERA_VERTICAL_DISPLACEMENT       = 110;   // eg: Camera is 110 mm above ground
    protected static final int CAMERA_LEFT_DISPLACEMENT           = 40;     // eg: Camera is 40 mm to the left of center line

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE       = BACK;
    protected static final boolean PHONE_IS_IN_PORTRAIT                         = false;

    // We delegate navigation to this object
    protected NavigationInterface navigation;


    // Robot placement at all time in Field Coordinates [x:{-F/2, F/2} y: {-F/2,F/2}]
    FieldPlacement botCurrentPlacement          = null;

    // Sounds
    BotSounds botSounds = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing the base...");
        super.init();
        telemetry.update();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing Vuforia");
        telemetry.update();

        /* ************************************
            NAVIGATION
        */
        navigation  = new VuforiaNavigation(
                botBase,
                hardwareMap,
                telemetry,
                VUFORIA_KEY,
                CAMERA_CHOICE,
                CAMERA_FORWARD_DISPLACEMENT,
                CAMERA_VERTICAL_DISPLACEMENT,
                CAMERA_LEFT_DISPLACEMENT,
                PHONE_IS_IN_PORTRAIT,
                this.DEBUG
        );

        // Tell the driver that initialization is complete.
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Status", "Make sure motors are free to move");
        telemetry.update();

        /**
         * SOUNDS
         */
        botSounds = new BotSounds(hardwareMap);
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

        botSounds.play("ss_r2d2_up");

        navigation.activate();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        botCurrentPlacement = navigation.getPlacement();
        if (botCurrentPlacement != null) {
            telemetry.addData("Navigation:", "Pos: (%.1f, %.1f) - Heading : %.1f rad.",
                    botCurrentPlacement.x,
                    botCurrentPlacement.y,
                    botCurrentPlacement.orientation);
        }
        else {
            telemetry.addData("No Targets:", "");
        }


        FieldPlacement stoneRelativeplacement = navigation.getSkyStone("Stone Target");
        if (stoneRelativeplacement != null ) {
            botSounds.play("ss_roger_roger");
            telemetry.addData("Stone:", "Travel: (%.1f, %.1f) - Heading : %.1f rad.",
                    stoneRelativeplacement.x,
                    stoneRelativeplacement.y,
                    stoneRelativeplacement.orientation);
        }
        else {
            telemetry.addData("No Stone:", "");
        }

        telemetry.update();

        /*
        Read gamepad value
         */
        // ... or for two 2-axis joysticks do this (Halo):
        // push joystick1 forward to go forward
        // push joystick1 to the right to strafe right
        // push joystick2 to the right to rotate clockwise
        double forward      = -gamepad1.right_stick_y;
        double right        = gamepad1.right_stick_x;
        double clockwise    = gamepad1.left_stick_x;


        // Now add a tuning constant K for the “rotate” axis sensitivity.
        // Start with K=0, and increase it very slowly (do not exceed K=1)
        // to find the right value after you’ve got fwd/rev and strafe working:
        clockwise = K*clockwise;

        // if "theta" is measured CLOCKWISE from the zero reference:
        double temp = forward* Math.cos(theta) + right* Math.sin(theta);
        right = -forward* Math.sin(theta) + right* Math.cos(theta);
        forward = temp;

// if "theta" is measured COUNTER-CLOCKWISE from the zero reference:
//        temp = forward*Math.cos(theta) - right*Math.sin(theta);
//        right = forward*Math.sin(theta) + right*Math.cos(theta);
//        forward = temp;


        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed commands:
        double front_left = forward + clockwise + right;
        double front_right = forward - clockwise - right;
        double rear_left = forward + clockwise - right;
        double rear_right = forward - clockwise + right;

        // Finally, normalize the wheel speed commands
        // so that no wheel speed command exceeds magnitude of 1:
        double max = Math.abs(front_left);
        if (Math.abs(front_right)>max) {
            max = Math.abs(front_right);
        }
        if (Math.abs(rear_left)>max){
            max = Math.abs(rear_left);
        }
        if (Math.abs(rear_right)>max) {
            max = Math.abs(rear_right);
        }
        if ( max > 1.0 ) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;
        }


        // Send calculated power to wheels
        botBase.getFrontLeftDrive().setPower(front_left);
        botBase.getFrontRightDrive().setPower(front_right);
        botBase.getRearLeftDrive().setPower(rear_left);
        botBase.getRearRightDrive().setPower(rear_right);

        // Show the elapsed game time and wheel power.
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), rear left (%.2f), rear right (%.2f)", front_left, front_right, rear_left, rear_right);
//
//        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
        navigation.activate();
    }
}
