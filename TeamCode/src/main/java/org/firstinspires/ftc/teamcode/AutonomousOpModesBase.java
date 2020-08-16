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

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Utils.FieldPlacement;
import org.firstinspires.ftc.teamcode.Utils.TravelDirection;
import org.firstinspires.ftc.teamcode.Utils.VuMarkIdentification;


import java.util.Random;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;



/**
 * This file contains the base class definition for all Autonomous OpModes.
 */

@Autonomous(name="Autonomous Base Class", group="none")
@Disabled
public class AutonomousOpModesBase extends LinearOpMode {

    protected static final double CLOSE_ENOUGH_X                = 1.0;
    protected static final double CLOSE_ENOUGH_Y                = 1.0;

    // Will dump debug information in the LogCat if true
    protected boolean DEBUG                               = false;

    // Will iniatiate VUFORIA
    protected Boolean USE_VUFORIA                         = true;
    protected String TRACKABLE_ASSET_NAME                 = "Skystone";

    /**
     * FIELD CONSTANT
     */
    static final double FIELD_WIDTH                     = 48.0;
    static final double FIELD_LENGTH                    = 96.0;

    /**
     * PROPULSION CONSTANTS
     */
    static final int TURN_DIRECTION_LEFT                = 1;  // this is a direction for
    static final int TURN_DIRECTION_RIGHT               = -1;
    static final int ERROR_POSITION_COUNT               = 10;
    static final double DRIVE_SPEED                     = 0.8;
    static final double TURNING_SPEED                   = 0.3;


    static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
     P_TURN_COEFF * TURNING_SPEED must be > 0.1
     ************************************************************************* */
    static final double     P_TURN_COEFF            = 0.5;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    static final double  LED_TEAM_COLORS1               = 0.6545;  // Sinelon, Color 1 and 2
    static final double  LED_TEAM_COLORS2               = 0.6295;  // End to End Blend
    static final double  LED_TEAM_COLORS3               = 0.6045;  // Sparkle, Color 1 on Color 2
    static final double  LED_TEAM_COLORS4               = 0.6195;  // Beats per Minute, Color 1 and 2

    static final double K                               = 1.17396293; // constant that maps change in voltage to change in RPM


    /**
     * NAVIGATION CONSTANTS
     */



    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;


    // Vuforia translation from the the robot center where x -> front, y -> left and  z -> up

    final float CAMERA_FORWARD_DISPLACEMENT  = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;    // eg: Camera is ON the left side


//
//    protected static final int CAMERA_FORWARD_DISPLACEMENT        = 150;   // eg: Camera is 150 mm in front of robot center
//    protected static final int CAMERA_VERTICAL_DISPLACEMENT       = 110;   // eg: Camera is 110 mm above ground
//    protected static final int CAMERA_LEFT_DISPLACEMENT           = 40;     // eg: Camera is 40 mm to the left of center line

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE       = FRONT;
    protected static final boolean PHONE_IS_IN_PORTRAIT                         = false;

    protected static final int FIND_NAVIGATION_BEACON_MAX_RETRY              = 10;


    /**
     * SENSOR CONSTANTS
     */


    // Number of sensors, they all have to be the same type
    protected static final int NB_FRONT_SENSORS                 = 1;
    protected static final int NB_REAR_SENSORS                  = 1;
    protected static final int NB_LEFT_SENSORS                  = 1;
    protected static final int NB_RIGHT_SENSORS                 = 1;

    // Distance of each sensors measured in inches from the center of the robot
    protected static final double DISTANCE_FRONT_SENSORS        = 9;
    protected static final double DISTANCE_REAR_SENSORS         = 9;
    protected static final double DISTANCE_LEFT_SENSORS         = 6;
    protected static final double DISTANCE_RIGHT_SENSORS        = 6;

    // Every 2 seconds we check if the delta position in either direction has changed by this amount
    protected static final double DELTA_CHECK_IF_STALLED_X      = 2.0;
    protected static final double DELTA_CHECK_IF_STALLED_Y      = 2.0;

    // After this amount of failed delta check we declare a stall
    protected static final int ATTEMPTS_BEFORE_STALL_DETECTED   = 5;


    /**
     * Class Objects      */

    /* Propulsion and basic hardware
     */
    public BotBase botBase              = null;
    public BotTop botTop                = null;



    /* VuMark detection
     */
    protected VuMarkIdentification vuMark  = null;

    /**
     * HARDWARE classes
     */

    // IMU
    protected BNO055IMU gyro = null;

    // Range Sensors
    ModernRoboticsI2cRangeSensor distanceFront = null;
    ModernRoboticsI2cRangeSensor distanceBack = null;
    ModernRoboticsI2cRangeSensor distanceLeft = null;
    ModernRoboticsI2cRangeSensor distanceRight = null;

    // Color sensors
    ColorSensor bottomColor                 = null;


    // navigation servo
    Servo camera_pan_horizontal = null;
    double cameraPanHorizontalPosition        = 0;
    int cameraPanHorizontalDirection          = 1; // 1: clockwise -1: counterclockwise

    // navigation servo
    Servo camera_pan_vertical = null;
    double cameraPanVerticalPosition        = 0;
    int cameraPanVerticalDirection          = 1; // 1: up -1: down


    // limit switches
    DigitalChannel backCollision       = null;

    /**
     * Class valiables for persistence
     */
    // stall probability (0-5 5:stalled)
    int stallProbability                        = 0;
    // Robot placement at all time in Field Coordinates [x:{-F/2, F/2} y: {-F/2,F/2}]
    FieldPlacement botCurrentPlacement          = null;
    // Previous Robot placement in Field Coordinates [x:{-F/2, F/2} y: {-F/2,F/2}]
    FieldPlacement botPreviousPlacement         = null;

    // current direction of displacement
    TravelDirection propulsionDirection         = TravelDirection.IDLE;

    // Timekeeper OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();


//    @Override
    public void runOpMode() {}

    protected void initAutonomous() {

        botBase     = new BotBase(hardwareMap);
        botTop      = new BotTop(hardwareMap);

        /*********************       GYRO        *********************** */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParameters);


        /* ************************************
            VUMARK
         */
        if (USE_VUFORIA) {
            vuMark  = new VuMarkIdentification(
                    hardwareMap,
                    telemetry,
                    TRACKABLE_ASSET_NAME,
                    CAMERA_CHOICE,
                    this.DEBUG
            );
        }


        /* ***********************************
            RANGE SENSORS
         */
        try {
            distanceFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range_1");
        }
        catch (Exception e){
            distanceFront = null;
            dbugThis("Unable to initialize front_range_1");
        }
        try {
            distanceBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rear_range_1");
        }
        catch (Exception e){
            distanceBack = null;
            dbugThis("Unable to initialize rear_range_1");
        }
        try {
            distanceLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range_1");
        }
        catch (Exception e){
            distanceLeft = null;
            dbugThis("Unable to initialize left_range_1");
        }
        try {
            distanceRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range_1");
        }
        catch (Exception e){
            distanceRight = null;
            dbugThis("Unable to initialize right_range_1");
        }


        /** LINE DETECTORS
         *
         */
        // get a reference to the color sensor.
        try {
            bottomColor = hardwareMap.get(ColorSensor.class, "bottom_color");
        }
        catch (Exception e){
            bottomColor = null;
            dbugThis("Unable to initialize bottom_color");
        }

        /**
         *  COLLISION
         */
        try {
            backCollision = hardwareMap.get(DigitalChannel.class, "back_collision");
        }
        catch (Exception e){
            backCollision = null;
            dbugThis("Unable to initialize back collision limit switch");
        }

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

    }


    /**
     * This free the resources and objects created for this opmode
     */
    protected void terminateAutonomous()
    {
        if (vuMark != null) {
            vuMark.stop();
        }
        stopMoving();
    }

    /**
     * This function will add the angle passed in parameter to the current heading
     *
     * @param angle:    degrees to add to the current heading
     */
    protected void turn(double angle)
    {

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        double finalTheta = actualAngle + angle;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, finalTheta, P_TURN_COEFF)) {

            autonomousIdleTasks();
        }

        justWait(0.5);
        stopMoving();
    }


    /**
     * This function stops all propulstion motors simultaneously
     */
    protected void stopMoving() {
        botBase.stop();
        propulsionDirection = TravelDirection.IDLE;
        return;
    }


    /**
     * Use the Gyro to turn until heading is equal to the angle passed in parameter
     *
     * @param angle         : Final heading in degrees
     */
    protected void gotoHeading(double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, angle, P_TURN_COEFF)) {

            autonomousIdleTasks();
        }

        justWait(0.5);

        stopMoving();
        return;
    }


    /**
     * This function moves the robot at the given angle until it reaches the final Destination.
     * angleInRadians is ALWAYS the angle between the desired direction and the direction the robot is facing.
     *
     * WARNING ***************  This method MUST be called INSIDE a control loop. ********************
     *
     * @param angleInRadians
     */
    protected void moveAtAngle(double angleInRadians, double power)
    {
        double temp = -angleInRadians + Math.PI/4.0;

        double front_left = power * Math.cos(temp);
        double front_right = power * Math.sin(temp);
        double rear_left = power * Math.sin(temp);
        double rear_right = power * Math.cos(temp);

        // normalize the wheel speed so we don't exceed 1
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

        botBase.getFrontLeftDrive().setPower(front_left);
        botBase.getFrontRightDrive().setPower(front_right);
        botBase.getRearLeftDrive().setPower(rear_left);
        botBase.getRearRightDrive().setPower(rear_right);

    }

    /**
     * Moves Right for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     */
    protected void moveRight(double distance, double power)
    {
        move(TravelDirection.RIGHT, timeToMoveInMs(TravelDirection.RIGHT, power, distance), power, false);
    }

    /**
     * Moves Left for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     */
    protected void moveLeft(double distance, double power)
    {

        move(TravelDirection.LEFT, timeToMoveInMs(TravelDirection.LEFT, power, distance), power, false);
    }

    /**
     * Moves Backward for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     */
    protected void moveBackward(double distance, double power)
    {
        move(TravelDirection.BACKWARD, timeToMoveInMs(TravelDirection.BACKWARD, power, distance), power,false);
    }

    /**
     * Moves Forward for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     *
     * @param distance: in inches
     */
    protected void moveForward(double distance, double power)
    {

        dbugThis("moving forward");
        move(TravelDirection.FORWARD, timeToMoveInMs(TravelDirection.FORWARD, power, distance), power,false);
    }


    /**
     * Powers up the propulsion to move Right for a period of time, and at a given power
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveRightByTime(int ms, double power)
    {
        move(TravelDirection.RIGHT, ms, power, false);
    }

    /**
     * Powers up the propulsion to move Left for a period of time, and at a given power
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveLeftByTime(int ms, double power)
    {
        move(TravelDirection.LEFT, ms, power, false);
    }

    /**
     * Powers up the propulsion to move Back for a period of time, and at a given power
     *
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveBackwardByTime(int ms, double power)
    {
        move(TravelDirection.BACKWARD, ms, power,false);
    }

    /**
     * Powers up the propulsion to move Forward for a period of time, and at a given power
     *
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveForwardByTime(int ms, double power)
    {
        move(TravelDirection.FORWARD, ms, power,false);
    }



    /**
     * Powers up the propulsion to move Left until it is x inches from an object
     *
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromLeftObject(double x, double ms, double power) {

        if (distanceLeft != null && getValidDistance(distanceLeft) < x ) {
            return;
        }

        powerPropulsion(TravelDirection.LEFT, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.LEFT) &&
            runtime.milliseconds() < limit &&
                    distanceLeft != null && getValidDistance(distanceLeft) > x
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Powers up the propulsion to move Right until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveRightToColor(int color, double power) {

        if ( getValidColor(bottomColor) == color ) {
            dbugThis("Getting out of here");
            return;
        }

        powerPropulsion(TravelDirection.RIGHT, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.RIGHT) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }



    /**
     * Powers up the propulsion to move Left until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveLeftToColor(int color, double power) {

        if ( getValidColor(bottomColor) == color ) {
            dbugThis("Getting out of here");
            return;
        }

        powerPropulsion(TravelDirection.LEFT, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.LEFT) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }



    /**
     * Powers up the propulsion to move Forward until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveForwardToColor(int color, double power) {

        if ( getValidColor(bottomColor) == color ) {
            dbugThis("Found the color");
            return;
        }

        powerPropulsion(TravelDirection.FORWARD, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.FORWARD) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }



    /**
     * Powers up the propulsion to move Backward until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveBackwardToColor(int color, double power) {

        if (getValidColor(bottomColor) == color ) {
            dbugThis("Getting out of here");
            return;
        }

        powerPropulsion(TravelDirection.BACKWARD, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.BACKWARD) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Powers up the propulsion to move Right until it is x inches from an object
     *
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromRightObject(double x, double ms, double power) {


        dbugThis("Moving To right object" + x);
        if (distanceRight != null && getValidDistance(distanceRight) < x ) {
            dbugThis("Getting out of here");
            return;
        }

        powerPropulsion(TravelDirection.RIGHT, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.RIGHT) &&
            runtime.milliseconds() < limit &&
                    distanceRight != null && getValidDistance(distanceRight) > x
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Powers up the propulsion to move Forward until it is x inches from an object
     *
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromFrontObject(double x, double ms, double power) {

        dbugThis("Moving X to FRONT object " + x);

        if (distanceFront != null && getValidDistance(distanceFront)  < x ) {
            return;
        }

        powerPropulsion(TravelDirection.FORWARD, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.FORWARD) &&
            runtime.milliseconds() < limit &&
            distanceFront != null && getValidDistance(distanceFront) > x
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Powers up the propulsion to move Back until it is x inches from an object
     *
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromBackObject(double x, double ms, double power) {

        dbugThis("Moving X to BACK object " + x);

        if (distanceBack != null && getValidDistance(distanceBack) < x ) {
            return;
        }

        powerPropulsion(TravelDirection.BACKWARD, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.BACKWARD) &&
            !isCollidingWithBackObject() &&
            runtime.milliseconds() < limit &&
            distanceBack != null && getValidDistance(distanceBack)  > x
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     *  Moves the robot in either 4 direction for ms amount of time OR until oriented whichever comes first
     *  if the untilOriented flag is set to true.  If untilOriented is set to false, motor will move according
     *  to time limit only.
     *
     * @param direction             : FORWARD,BACKWARD,LEFT,RIGHT
     * @param ms                    : Limit of time the motos will be in motion
     * @param power                 : Motor power
     * @param untilRealigned         : The robot will move until oriented
     */
    private void move(TravelDirection direction, double ms, double power, boolean untilRealigned) {


        if (power == 0) {
            power = DRIVE_SPEED;
        }
        boolean bumped = false;
        powerPropulsion(direction, power);

        double limit = runtime.milliseconds() + ms;
        double limitToSlowDown = runtime.milliseconds() + 0.85 * ms;
        double now;

        while (
            opModeIsActive() &&
            (now = runtime.milliseconds()) < limit &&
            (!untilRealigned || untilRealigned && botCurrentPlacement != null)
        ) {
            if (isCollidingBack() && (direction == TravelDirection.BACKWARD) && !bumped ) {
                bumped = true;
                powerPropulsion(direction, 0.1);
                limit = now + 2000;
            }
//            if ( now > limitToSlowDown && power > 0.4 ) {
//                powerPropulsion(direction, power / 2.0);
//            }
//            else {
//                powerPropulsion(direction, power);
//            }

            autonomousIdleTasks();
        }
        stopMoving();
        return;
    }


    /**
     * Powers the propulsion to go in either 4 directions
     *
     * @param direction
     * @param power
     *
     * WARNING ***************  This method MUST be called INSIDE a control loop. ********************
     *
     */
    protected void powerPropulsion(TravelDirection direction, double power) {

        if (power == 0) {
            power = DRIVE_SPEED;
        }

        double multiplierFL = 0;
        double multiplierFR = 0;
        double multiplierRL = 0;
        double multiplierRR = 0;

        switch (direction) {
            case FORWARD:
                multiplierFL = 1;
                multiplierFR = 1;
                multiplierRL = 0.97;
                multiplierRR = 0.97;
                propulsionDirection = TravelDirection.FORWARD;
                break;
            case BACKWARD:
                multiplierFL = -1;
                multiplierFR = -1;
                multiplierRL = -0.97;
                multiplierRR = -0.97;
                propulsionDirection = TravelDirection.BACKWARD;
                break;
            case LEFT:
                multiplierFL = -1;
                multiplierFR = 1;
                multiplierRL = 0.97;
                multiplierRR = -0.97;
                propulsionDirection = TravelDirection.LEFT;
                break;
            case RIGHT:
                multiplierFL = 1;
                multiplierFR = -1;
                multiplierRL = -0.97;
                multiplierRR = 0.97;
                propulsionDirection = TravelDirection.RIGHT;
                break;
            default:
                return;

        }

        botBase.getFrontRightDrive().setPower(power * multiplierFR);
        botBase.getRearRightDrive().setPower(power * multiplierRR);
        botBase.getFrontLeftDrive().setPower(power * multiplierFL);
        botBase.getRearLeftDrive().setPower(power * multiplierRL);
    }



    public void setCameraVerticalPosition(double position) {
        if (camera_pan_vertical == null) {
            return;
        }
        position = Math.min(Math.max(0, position), 1.0);
        cameraPanVerticalPosition = position;
        camera_pan_vertical.setPosition(position);
    }


    public void setCameraHorizontalPosition(double position) {
        if (camera_pan_horizontal == null) {
            return;
        }
        position = Math.min(Math.max(0, position), 1.0);
        cameraPanHorizontalPosition = position;
        camera_pan_horizontal.setPosition(position);
    }


    /**
     *
     * @return
     */
     public void moveCamera() {
         if ( camera_pan_vertical == null ) {
             return;
         }

         double position = cameraPanHorizontalPosition;
         int direction  = cameraPanHorizontalDirection;

         dbugThis(position + "");

         // pan right
         if ( direction > 0 ) {
             position = Math.min(position + 0.05, 1.0);
             if (position == 1.0) {
                 cameraPanHorizontalDirection = -cameraPanHorizontalDirection;
             }
         }

         else if ( direction < 0 ) {
             position = Math.max(position - 0.05, 0.0);
             if (position == 0.0) {
                 cameraPanHorizontalDirection = -cameraPanHorizontalDirection;
             }
         }
         setCameraHorizontalPosition(position);
     }


    /**
     * Determines if the robot is stalled by checking every 2 seconds if is has moved significantly.  It uses
     * class properties to keep track of current and previous positions, and it accrues the stallProbability each time it failed
     * to detect movement.  stallProbability is reset whenever movement is detected.
     *
     * NOTE:  This method is used as a condition to break a movement loop.
     *
     * @return true|false
     */
    private boolean isStalled()
    {
        return false;
    }


    /**
     *
     * Returns true if the robot has moved given its previous and last positions.  This function is used to determine if the bot is
     * stalled.
     *
     * @param previousPlacement
     * @param currentPlacement
     * @return true|false
     */
    private boolean hasMovedSignificantly(FieldPlacement previousPlacement, FieldPlacement currentPlacement)
    {

        if (previousPlacement == null) return false;
        if (currentPlacement == null) return false;
        if ( Math.abs(previousPlacement.x - currentPlacement.x) > DELTA_CHECK_IF_STALLED_X ) return true;
        if ( Math.abs(previousPlacement.y - currentPlacement.y) > DELTA_CHECK_IF_STALLED_Y ) return true;

        return false;
    }


    /**
     *
     * @param currentPlacement
     * @param finalDestination
     * @return              : true if position is within target range according to the rule ERROR_POSITION_COUNT
     *
     */
    public boolean isPositionWithinAcceptableTargetRange(FieldPlacement currentPlacement, FieldPlacement finalDestination)
    {
        return ( (Math.abs(currentPlacement.x - finalDestination.x) <= CLOSE_ENOUGH_X) && (Math.abs(currentPlacement.y - finalDestination.y) <= CLOSE_ENOUGH_Y));
    }




    /**
     * Performs one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {

        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getHeadingError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        botBase.getFrontLeftDrive().setPower(leftSpeed);
        botBase.getRearLeftDrive().setPower(leftSpeed);
        botBase.getFrontRightDrive().setPower(rightSpeed);
        botBase.getRearRightDrive().setPower(rightSpeed);

        return onTarget;
    }


    /**
     * Determines the error between the target angle and the robot's current heading
     *
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     *
     */
    public double getHeadingError(double targetAngle) {

        double robotError;
        double actualAngle;

        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - actualAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;

        return robotError;
    }


    /**
     * Determines the error between the target angle and the robot's current heading
     *
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     *
     */
    public double getHeadingErrorRadians(double targetAngle) {

        double robotError;
        double actualAngle;

        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        actualAngle = AngleUnit.RADIANS.normalize(AngleUnit.RADIANS.fromUnit(angles.angleUnit, angles.firstAngle));

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - actualAngle;
        while (robotError > Math.PI)  robotError -= Math.PI*2;
        while (robotError <= -Math.PI) robotError += Math.PI*2;

        return robotError;
    }


    /**
     * Returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return steer
     *
     */
    public double getSteer(double error, double PCoeff) {

        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     *  This method picks a random value inside the given enum
     *
     * @param clazz :
     * @param <T>
     * @return value
     */

    public static <T extends Enum<?>> T randomEnum(Class<T> clazz){

        Random random = new Random();
        int x = random.nextInt(clazz.getEnumConstants().length);
        return clazz.getEnumConstants()[x];
    }


    /**
     *
     * This function does nothing but wait while allowing to other processes to run
     *
     * @param ms       : any number of milliseconds.  Can be less than 1
     *
     */
    protected void justWait(double ms) {

        ms = Math.abs(ms);
        double limit = runtime.milliseconds() + ms;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && runtime.milliseconds() < limit  ) {

            autonomousIdleTasks();
        }
    }


    /**
     * Logs a string to the LogCat Window
     * @param s
     */
    void dbugThis(String s) {

        if ( DEBUG == true ) {
            Log.d("AUTONOMOUS: ", s);
        }
    }

    /**
     *
     * Given a direction, returns true if the robot is too close
     *
     * @param direction
     * @return bool
     *
     */
    public boolean isHittingSomething(TravelDirection direction) {
//
//
//        switch (direction) {
//
//            case FORWARD:
//                if ( getValidDistance(distanceFront) < 2.0 ) {
//                    return true;
//                }
//                return false;
//
//            case BACKWARD:
//                if ( getValidDistance(distanceBack) < 2.0 ) {
//                    return true;
//                }
//                return false;
//
//            case LEFT:
//                if ( getValidDistance(distanceLeft) < 2.0 ) {
//                    return true;
//                }
//                return false;
//
//            case RIGHT:
//                if ( getValidDistance(distanceRight) < 2.0 ) {
//                    return true;
//                }
//                return false;
//        }

        return false;
    }


    /**
     * Always insert this function inside control loop as it checks the emergency situations related to limit switches and collisions and acts
     * consequently.
     */
    protected void autonomousIdleTasks() {
        idle();
        botTop.checkAllLimitSwitches();
    }

    /**
     * Given a direction, and the power level of the motor, this function will return the amount
     * of time required to power the propulsion to travel the required displacement in inches
     *
     * @param direction
     * @param power
     * @param displacementInInches
     * @return amount of milliseconds in inces
     *
     */
    private int timeToMoveInMs(TravelDirection direction, double power, double displacementInInches) {

        if (direction == TravelDirection.LEFT || direction == TravelDirection.RIGHT ) {
            if (power <= 0.2) {
                return (int) adjustForVoltageDrop(displacementInInches * 166.67);
            }
            if (power <= 0.3) {
                return (int) adjustForVoltageDrop(displacementInInches * 93.80);
            }
            if (power <= 0.4) {
                return (int) adjustForVoltageDrop(displacementInInches * 71.42);
            }
            if (power <= 0.5) {
                return (int) adjustForVoltageDrop(displacementInInches * 60.00);
            }
            if (power <= 0.6) {
                return (int) adjustForVoltageDrop(displacementInInches * 45.45);
            }
            if (power <= 0.7) {
                return (int) adjustForVoltageDrop(displacementInInches * 37.5);
            }

            return (int) adjustForVoltageDrop(displacementInInches * 20.00);
        }
        if (direction == TravelDirection.FORWARD || direction == TravelDirection.BACKWARD ) {
            if (power <= 0.2) {
                return (int) adjustForVoltageDrop(displacementInInches * 125.0);
            }
            if (power <= 0.3) {
                return (int) adjustForVoltageDrop(displacementInInches * 89.29);
            }
            if (power <= 0.4) {
                return (int) adjustForVoltageDrop(displacementInInches * 58.82);
            }
            if (power <= 0.5) {
                return (int) adjustForVoltageDrop(displacementInInches * 53.57);
            }
            if (power <= 0.6) {
                return (int) adjustForVoltageDrop(displacementInInches * 40.92);
            }
            if (power <= 0.7) {
                return (int)adjustForVoltageDrop(displacementInInches * 33.33);
            }
            return (int) adjustForVoltageDrop(displacementInInches * 28.57);
        }

        return 0;
    }


    private double adjustForVoltageDrop(double ms) {
        return ms / (1 - K + (K*getBatteryVoltage()/13.0));
    }



    /**
     * Checks the state of the back limit switch and the direction of the movement
     * as kept in the state variable _propulsionDirection_  It returns true if
     * the limit condition is met.
     *
     * If the limit condition is met, the motor is stopped
     * @return
     */
    public boolean isCollidingWithBackObject() {

        if (propulsionDirection == TravelDirection.BACKWARD && isCollidingBack()) {
            stopMoving();
            return true;
        }
        return false;
    }


    /**
     * Checks the state of the limit switch for backCollision limit switch.
     * If there is no such limit switch, it returns false.
     *
     * @return
     */
    public boolean isCollidingBack() {

        if (backCollision == null) {
            return false;
        }

        return !(backCollision.getState() == true );
    }


    /**
     * MR distance sensors tend to return bogus values once in a while.  This function makes sure filter out these values
     * by waiting for the next valid value.
     *
     * @param sensor
     * @return
     *
     */
    public double getValidDistance(ModernRoboticsI2cRangeSensor sensor) {

        if (sensor == null) {
            return 255;
        }

        double limit = runtime.milliseconds() + 1000;
        double validDistance = sensor.getDistance(DistanceUnit.INCH);
        while ( opModeIsActive() &&
                runtime.milliseconds() < limit &&
                ((validDistance = sensor.getDistance(DistanceUnit.INCH))) == DistanceSensor.distanceOutOfRange )  {
            autonomousIdleTasks();
        }

        dbugThis("From get Valid distance" + validDistance);

        return validDistance == DistanceSensor.distanceOutOfRange ? 255 : validDistance;
    }


    public int getValidColor(ColorSensor sensor) {


        if (sensor == null ) {
            dbugThis("Valid color is null");
            return Color.BLACK;
        }

        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        float hsvValues[] = {0F, 0F, 0F};
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensor.red() * 255),
                (int) (sensor.green() * 255),
                (int) (sensor.blue() * 255),
                hsvValues);

        if ( red > 80 && red > green &&  red > blue  ) {
            dbugThis("Valid color is red");
            return Color.RED;
        }

        if ( blue > 80 && blue > green &&  blue > red ) {
            dbugThis("Valid color is blue");
            return Color.BLUE;
        }
        dbugThis("Valid color is default");
        return Color.BLACK;
    }



    // Computes the current battery voltage
    double getBatteryVoltage() {
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