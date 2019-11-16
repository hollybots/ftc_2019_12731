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
import com.qualcomm.robotcore.util.ElapsedTime;




/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="TeleOp Main", group="1")
//@Disabled
public class TeleOpMode_12731 extends TeleOpModesBase
{

    private class WheelPower {
        double front_left;
        double front_right;
        double rear_left;
        double rear_right;
    };
    static final int CONTROL_FORWARD = 0;
    static final int CONTROL_RIGHT = 1;
    private double[] ramp = {
            -1,
            -0.86,
            -0.73,
            -0.61,
            -0.51,
            -0.42,
            -0.34,
            -0.27,
            -0.22,
            -0.17,
            -0.13,
            -0.09,
            -0.06,
            -0.04,
            -0.03,
            -0.02,
            -0.01,
            0,
            0.01,
            0.02,
            0.03,
            0.04,
            0.06,
            0.09,
            0.13,
            0.17,
            0.22,
            0.27,
            0.34,
            0.42,
            0.51,
            0.61,
            0.73,
            0.86,
            1
    };

    // This limits the power change to an 0.1 increment every 200ms 0,00005 power/s^2
    static final double  DELTA_T                        = 200; // in ms ) {
    static final double  MAX_CHANGE_IN_POWER_IN_DELTA_T = 0.1;

    static final double  LED_OFF                        = 0.7745;   // off
    static final double  LED_TEAM_COLORS1               = 0.6545;  // Sinelon, Color 1 and 2
    static final double  LED_TEAM_COLORS2               = 0.6295;  // End to End Blend
    static final double  LED_TEAM_COLORS3               = 0.6045;  // Sparkle, Color 1 on Color 2
    static final double  LED_TEAM_COLORS4               = 0.6195;  // Beats per Minute, Color 1 and 2

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    static final double     K                           = 0.6;
    private double          theta                       = 0;   // gyro angle.  For field centric autonomous mode we will use this to orient the robot

    boolean use2Controllers                             = true;


    // State variables
    boolean isClamping                                  = false;
    boolean waitForClampingButtonRelease                = false;




    double lastTimeWeCheckedSpeed                       = 0.0;
    int currentRampNumberForward                        = 20;
    int currentRampNumberRight                          = 20;
    double previousPowerForward                             = 0;
    double previousPowerRight                               = 0;


    int resetState                                      = 0;
    int readyState                                      = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing the base...");
        telemetry.update();

        // Init Botbase and Bottop
        super.init();

        botBase.setBling(LED_OFF);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        // Send telemetry message to indicate successful Encoder reset
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /*
        Read gamepad value
         */

        /**
         * INPUT GAMEPAD
         */
        // ... or for two 2-axis joysticks do this (Halo):
        // push joystick1 forward to go forward
        // push joystick1 to the right to strafe right
        // push joystick2 to the right to rotate clockwise


        double forward                  = -gamepad1.left_stick_y;
        double right                    = gamepad1.left_stick_x;
        WheelPower wheels               = null;

        double now                      = runtime.milliseconds();
        double deltaT                   = now - lastTimeWeCheckedSpeed;

        double clockwise                = gamepad1.right_stick_x;

        double slideUp          = !use2Controllers ? gamepad1.right_trigger : gamepad2.right_trigger ;
        double slideDown        = !use2Controllers ? gamepad1.left_trigger : gamepad2.left_trigger;
        boolean clawDown        = !use2Controllers ? gamepad1.right_bumper : gamepad2.right_bumper;
        boolean clawUp          = !use2Controllers ? gamepad1.left_bumper : gamepad2.left_bumper;

        double arm                  = -gamepad2.right_stick_y;
        double linearMotion         = -gamepad2.left_stick_y;

        boolean isPressedClampingButton     = gamepad2.right_stick_button;
        boolean toggledClamp                = false;

        boolean blinkinOff                  =  gamepad1.dpad_up;
        boolean blinkin1                    =  gamepad1.dpad_down;
        boolean blinkin2                    =  gamepad1.dpad_left;
        boolean blinkin3                    =  gamepad1.dpad_right;

        boolean reset                       =  gamepad2.x;
        boolean ready                       =  gamepad2.y;

        wheels                              = calcWheelPower(K, clockwise, forward, right);

        botTop.checkAllLimitSwitches();

        // don't allow any other command aside  of propulsion while robot is resetting
        if (resetState > 0) {
            maybeEndResetSequence();
        }

        if (readyState > 0) {
            maybeEndReadySequence();
        }

        // If we aske for reset, we start the sequence here
        if (resetState == 0 && reset ) {
            startResetSequence();
        }

        if (resetState == 0) {
            // If we aske for reset, we start the sequence here
            if (readyState == 0 && ready ) {
                startReadySequence();
            }
        }

        if ( resetState == 0 && readyState == 0) {

            /**
             * OUTPUT LINEAR MOTION
             */
            botTop.coil(linearMotion);

            /**
             * OUTPUT SWIVEL ARM
             */
            botTop.swing(arm);


            /**
             * OUTPUT CLAW
             */
            if (clawDown) {
                botTop.closeClaw();
            } else if (clawUp) {
                botTop.openClaw();
            }


            /**
             * OUTPUT SLIDE
             */
            if (slideUp > 0) {
                botTop.slideUp();
            } else if (slideDown > 0) {
                botTop.slideDown();
            } else {
                botTop.stopSlide();
            }


            /**
             * OUTPUT CLAMP
             */
            // Check for a toggle state -> needs to be Clamp button must be pressed and released
            if (isPressedClampingButton) {
                waitForClampingButtonRelease = true;
            } else if (waitForClampingButtonRelease) {
                waitForClampingButtonRelease = false;
                toggledClamp = true;
            }

            if (toggledClamp && isClamping) {
                isClamping = false;
            } else if (toggledClamp && !isClamping) {
                isClamping = true;
            }
            if (isClamping) {

                botTop.clampOn();
            } else {
                botTop.clampRelease();
            }
        }


        /**
         * OUTPUT PROPULSION
         */
        // Send calculated power to wheels
        botBase.getFrontLeftDrive().setPower(wheels.front_left);
        botBase.getFrontRightDrive().setPower(wheels.front_right);
        botBase.getRearLeftDrive().setPower(wheels.rear_left);
        botBase.getRearRightDrive().setPower(wheels.rear_right);

        dbugThis(String.format("%.02f,%.02f,%.02f,%.02f", wheels.front_left,wheels.front_right,wheels.rear_left,wheels.rear_right));


        /**
         * OUTPUT BLING
         */
        if (blinkinOff) {
            botBase.setBling(LED_OFF);
        }
        else if (blinkin1) {
            botBase.setBling(LED_TEAM_COLORS1);
        }
        else if (blinkin2) {
            botBase.setBling(LED_TEAM_COLORS4);
        }
        else if (blinkin3) {
            botBase.setBling(LED_TEAM_COLORS3);
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
        botBase.setBling(LED_OFF);
    }

    private void maybeEndResetSequence(){

        // If the coil is completely down, start moving down the arm
        if ( botTop.isCoilLimitDown() && resetState == 1 ) {
            botTop.swing(BotTop.SWING_DOWN_COMMAND);
            resetState = 2;
            return;
        }

        // If the arm limit switch is met, then we are done
        if ( botTop.isSwingLimitDown() && resetState == 2 ) {
            resetState = 0;
        }
    }

    private void startResetSequence() {

        if (botTop.isSwingLimitDown()) {
            return;
        }
        resetState = 1;
        botTop.coil(BotTop.COIL_DOWN_COMMAND);
        botTop.slideDown();
        botTop.clampRelease();
        botTop.openClaw();
    }


    private void maybeEndReadySequence(){

        // If the coil is completely down, start moving down the arm
        if ( botTop.isSwingLimitUp() && readyState == 1 ) {
            readyState = 0;
        }
    }

    private void startReadySequence() {

        botTop.swing(BotTop.SWING_UP_COMMAND);
        readyState = 1;
        botTop.slideDown();
        botTop.clampRelease();
        botTop.openClaw();
    }


    private WheelPower calcWheelPower(double K, double clockwise, double forward, double right) {

        WheelPower wheels                    = new WheelPower();

        // Now add a tuning constant K for the “rotate” axis sensitivity.
        // Start with K=0, and increase it very slowly (do not exceed K=1)
        // to find the right value after you’ve got fwd/rev and strafe working:
        clockwise = K * clockwise;

        // if "theta" is measured CLOCKWISE from the zero reference:
        double temp = forward * Math.cos(theta) + right * Math.sin(theta);
        right = -forward * Math.sin(theta) + right * Math.cos(theta);
        forward = temp;

        // if "theta" is measured COUNTER-CLOCKWISE from the zero reference:
//        temp = forward*Math.cos(theta) - right*Math.sin(theta);
//        right = forward*Math.sin(theta) + right*Math.cos(theta);
//        forward = temp;

        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed command:
        wheels.front_left = forward + clockwise + right;
        wheels.front_right = forward - clockwise - right;
        wheels.rear_left = forward + clockwise - right;
        wheels.rear_right = forward - clockwise + right;

        // Finally, normalize and limit acceleration for the wheel speed command
        // so that no wheel speed command exceeds magnitude of 1 and acceleration is kept under limit:
        double calculatedPropulsionCommand = Math.abs(wheels.front_left);
        if (Math.abs(wheels.front_right) > calculatedPropulsionCommand) {
            calculatedPropulsionCommand = Math.abs(wheels.front_right);
        }
        if (Math.abs(wheels.rear_left) > calculatedPropulsionCommand) {
            calculatedPropulsionCommand = Math.abs(wheels.rear_left);
        }
        if (Math.abs(wheels.rear_right) > calculatedPropulsionCommand) {
            calculatedPropulsionCommand = Math.abs(wheels.rear_right);
        }

        if (calculatedPropulsionCommand > 1.0) {
            wheels.front_left /= calculatedPropulsionCommand;
            wheels.front_right /= calculatedPropulsionCommand;
            wheels.rear_left /= calculatedPropulsionCommand;
            wheels.rear_right /= calculatedPropulsionCommand;
        }

        return wheels;
    }



    private double rampUp(int which, double setPoint) {


        if (which == CONTROL_FORWARD) {

            if (setPoint > previousPowerForward && setPoint > 0) {

                previousPowerForward = Math.min(setPoint, previousPowerForward + MAX_CHANGE_IN_POWER_IN_DELTA_T);
                return previousPowerForward;
            }


            if (setPoint < previousPowerForward && setPoint < 0) {

                previousPowerForward = Math.max(setPoint, previousPowerForward - MAX_CHANGE_IN_POWER_IN_DELTA_T);
                return previousPowerForward;
            }

            previousPowerForward = setPoint;
            return previousPowerForward;
        }
//        else if (which == CONTROL_RIGHT) {
//
//            if ( setPoint < previousPowerRight) {
//                currentRampNumberRight = Math.min(Math.max(0, currentRampNumberRight - 1), 40);
//            }
//            else if (setPoint > previousPowerRight) {
//                currentRampNumberRight = Math.min(Math.max(0, currentRampNumberRight + 1), 40);
//            }
//
//            previousPowerRight = ramp[currentRampNumberForward];
//            return ramp[currentRampNumberForward];
//        }

        return 0;
    }


    double findNextSetPointUp(double under) {

        int i = 0;
        while (ramp[i] < under) {
            i++;
        }
        i = Math.min(Math.max(0, i), 40);

        return ramp[i];
    }
}
