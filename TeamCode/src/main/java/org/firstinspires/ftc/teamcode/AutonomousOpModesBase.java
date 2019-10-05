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

//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;
import java.util.Random;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;



/**
 * This file contains the base class definition for all Autonomous OpModes.
 */

@Autonomous(name="Autonomous Base Class", group="none")
//@Disabled
public class AutonomousOpModesBase extends LinearOpMode {

    protected static final double CLOSE_ENOUGH_X                = 1.0;
    protected static final double CLOSE_ENOUGH_Y                = 1.0;

    // Will dump debug information in the LogCat if true
    boolean DEBUG                             = false;

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

    static final double DRIVE_SPEED                     = 0.9;
    static final double TURNING_SPEED                   = 0.3;


    static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
     P_TURN_COEFF * TURNING_SPEED must be > 0.1
     ************************************************************************* */
    static final double     P_TURN_COEFF            = 0.5;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    /**
     * NAVIGATION CONSTANTS
     */

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

    protected static final int FIND_NAVIGATION_BEACON_MAX_RETRY              = 10;


    // Number of sensors, they all have to be the same type
    protected static final int NB_FRONT_SENSORS                 = 1;
    protected static final int NB_REAR_SENSORS                  = 1;
    protected static final int NB_LEFT_SENSORS                  = 1;
    protected static final int NB_RIGHT_SENSORS                 = 1;

    // Distance of each sensors measured in inches from the center of the robot
    protected static final double DISTANCE_FRONT_SENSORS        = 8;
    protected static final double DISTANCE_REAR_SENSORS         = -8;
    protected static final double DISTANCE_LEFT_SENSORS         = -8;
    protected static final double DISTANCE_RIGHT_SENSORS        = 8;

    // Every 2 seconds we check if the delta position in either direction has changed by this amount
    protected static final double DELTA_CHECK_IF_STALLED_X      = 2.0;
    protected static final double DELTA_CHECK_IF_STALLED_Y      = 2.0;

    // After this amount of failed delta check we declare a stall
    protected static final int ATTEMPTS_BEFORE_STALL_DETECTED   = 5;


    /**
     * Class Objects
     */

    /* Propulsion and basic hardware
     */
    protected BotBase botBase = new BotBase();

    // We delegate navigation to this object
    protected NavigationInterface navigation;


    /**
     * Hardware classes
     */

    // integrated IMU
    protected BNO055IMU gyro = null;

    // Range Sensors
    ModernRoboticsI2cRangeSensor distanceFront = null;
    ModernRoboticsI2cRangeSensor distanceBack = null;
    ModernRoboticsI2cRangeSensor distanceLeft = null;
    ModernRoboticsI2cRangeSensor distanceRight = null;


    // navigation servo
    Servo camera_pan = null;

    /**
     * Class valiables for persistence
     */
    // stall probability (0-5 5:stalled)
    int stallProbability                        = 0;
    // Robot placement at all time in Field Coordinates [x:{-F/2, F/2} y: {-F/2,F/2}]
    FieldPlacement botCurrentPlacement          = null;
    // Previous Robot placement in Field Coordinates [x:{-F/2, F/2} y: {-F/2,F/2}]
    FieldPlacement botPreviousPlacement         = null;

    // Timekeeper OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {}


    protected void initAutonomous() {

        botBase.init(hardwareMap);

        /* DC MOtors with mecanum wheels, we are not using the encoders at all */
        botBase.getFrontRightDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getFrontRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botBase.getFrontRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        botBase.getFrontLeftDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getFrontLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botBase.getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        botBase.getRearRightDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getRearRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botBase.getRearRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        botBase.getRearLeftDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botBase.getRearLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botBase.getRearLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
            NAVIGATION
         */
        navigation  = new VuforiaNavigation(
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

        /* ***********************************
            RANGE SENSORS
         */
        distanceFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range_1");
        distanceBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rear_range_1");


        /* **********************************
            SERVO
         */
        camera_pan = hardwareMap.get(Servo.class, "camera_pan");
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

    }


    /*****************************************************************
     * HIGH LEVEL AUTONOMOUS MOVEMENT
     */


    /**
     * gotoPlacement()
     *
     * This function takes in a FieldPlacement object and position the robot according the x,y,orientation values
     * in that object.
     *
     * @param  destination                      : x,y, orientation values (see the FieldPlacement class)
     */
    protected void gotoPlacement(FieldPlacement destination) {

        double translation_x;
        double translation_y;
        double demandedTheta;
        double theta;

        realign();

        // Keeping the robot oriented north, we are going to make it to our destination by travelling at angle

        dbugThis("** Entering gotoPlacement **");
        dbugThis(String.format("Current position X: %2.2f", botCurrentPlacement.x));
        dbugThis(String.format("Current position y: %2.2f", botCurrentPlacement.y));

        dbugThis(String.format("Target position X: %2.2f", destination.x));
        dbugThis(String.format("Target position y: %2.2f", destination.y));

        // pid for approaching position
        while ( opModeIsActive() && !isPositionWithinAcceptableTargetRange(botCurrentPlacement, destination) ) {

            // if robot gets stalled moving in the given direction, we have to try to release it
            if ( isStalled() ) {
                nudgeRobot();
            }

            translation_x = destination.x - botCurrentPlacement.x;
            translation_y = destination.y - botCurrentPlacement.y;
            demandedTheta = Math.atan2(translation_y, translation_x);
            theta = (Math.PI / 2.0 - demandedTheta) - botCurrentPlacement.orientation;

            dbugThis(String.format("Translation Y: %2.2f",translation_y));
            dbugThis(String.format("Translation X: %2.2f",translation_x));
            dbugThis(String.format("Diagonal Displacement: %2.2f", demandedTheta));

            moveAtAngle(theta);
            botCurrentPlacement = navigation.getPlacement();
        }
        stopMoving();

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }




    /******************************
     * PROPULSION HELPERS
     */


    /**
     * realign()
     */
    protected void realign() {

        int tries = 0;

        botCurrentPlacement = navigation.getPlacement();
        while ( botCurrentPlacement == null && tries++ < FIND_NAVIGATION_BEACON_MAX_RETRY  &&  opModeIsActive() ) {

            // if we were not able to orient ourselves, we have to move a bit
            nudgeRobot();
            justWait(500);
        }
    }


    /**
     * turn()
     *
     * This function will add the angle passed in parameter to the current heading
     *
     * @param angle:    degrees to add to the current heading
     */
    protected void turn(double angle) {

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        double finalTheta = actualAngle + angle;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, finalTheta, P_TURN_COEFF)) {
            idle();
        }

        justWait(0.5);
        stopMoving();
    }


    /**
     * stopMoving()
     *
     * This function stops all motors simultaneously
     */
    protected void stopMoving() {

        botBase.stop();
        return;
    }


    /**
     * gotoHeading()
     *
     * Use the Guro to turn until heading is equal to the angle passed in parameter
     *
     * @param angle         : Final heading in degrees
     */
    protected void gotoHeading(double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(TURNING_SPEED, angle, P_TURN_COEFF)) {

            idle();
        }

        justWait(0.5);

        stopMoving();
        return;
    }


    /**
     * This function moves the robot at the given angle until it reaches the final Destination.
     * angleInRadians is ALWAYS the angle between the desired direction and the direction the robot is facing.
     *
     * Thie method MUST be called within a control loop.
     *
     * @param angleInRadians
     */
    private void moveAtAngle(double angleInRadians)
    {
        double temp = -angleInRadians + Math.PI/4.0;

        double front_left = DRIVE_SPEED * Math.cos(temp);
        double front_right = DRIVE_SPEED * Math.sin(temp);
        double rear_left = DRIVE_SPEED * Math.sin(temp);
        double rear_right = DRIVE_SPEED * Math.cos(temp);

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
     * moveRight()
     *
     * @param ms: Number of milliseconds to keep the motos moving
     */
    protected void moveRightByTime(int ms)
    {
        move(TravelDirection.RIGHT, ms,false);
    }

    /**
     * moveLeft()
     *
     * @param ms: Number of milliseconds to keep the motos moving
     */
    protected void moveLeftByTime(int ms)
    {
        move(TravelDirection.LEFT, ms,false);
    }

    /**
     * moveBackward()
     *
     * @param ms: Number of milliseconds to keep the motos moving
     */
    protected void moveBackwardByTime(int ms)
    {
        move(TravelDirection.BACKWARD, ms,false);
    }

    /**
     * moveForward()
     *
     * @param ms: Number of milliseconds to keep the motos moving
     */
    protected void moveForwardByTime(int ms)
    {
        move(TravelDirection.FORWARD, ms, false);
    }

    /**
     *  move()
     *
     *  The robot will move in either 4 direction for ms amount of time or until oriented whichever comes first
     *  if the untilOriented flag is set to true.  If untilOriented is set to false, motor will move according
     *  to time limit only.
     *
     * @param direction             : FORWARD,BACKWARD,LEFT,RIGHT
     * @param ms                    : Limit of time the motos will be in motion
     * @param untilRealigned         : The robot will move until oriented
     */
    private void move(TravelDirection direction, double ms, boolean untilRealigned) {

        double multiplierFL = 0;
        double multiplierFR = 0;
        double multiplierRL = 0;
        double multiplierRR = 0;

        switch (direction) {
            case FORWARD:
                multiplierFL = 1;
                multiplierFR = 1;
                multiplierRL = 1;
                multiplierRR = 1;
                break;
            case BACKWARD:
                multiplierFL = -1;
                multiplierFR = -1;
                multiplierRL = -1;
                multiplierRR = -1;
                break;
            case LEFT:
                multiplierFL = -1;
                multiplierFR = 1;
                multiplierRL = 1;
                multiplierRR = -1;
                break;
            case RIGHT:
                multiplierFL = 1;
                multiplierFR = -1;
                multiplierRL = -1;
                multiplierRR = 1;
                break;
            default:
                return;

        }

        botBase.getFrontRightDrive().setPower(DRIVE_SPEED * multiplierFR);
        botBase.getRearRightDrive().setPower(DRIVE_SPEED * multiplierRR);
        botBase.getFrontLeftDrive().setPower(DRIVE_SPEED * multiplierFL);
        botBase.getRearLeftDrive().setPower(DRIVE_SPEED * multiplierRL);

        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(direction) &&
//            !isStalled() &&
            runtime.milliseconds() < limit &&
            (!untilRealigned || untilRealigned && botCurrentPlacement != null)
        ) {
            idle();
        }

        dbugThis(String.format("isHittingSomething : %b", isHittingSomething(direction)));
        dbugThis(String.format("isStalled : %b", isStalled()));
        dbugThis(String.format("untilRealigned : %b", untilRealigned));

        stopMoving();
        botCurrentPlacement = navigation.getPlacement();
        return;
    }


    /**
     *
     * @return
     */
     public void moveCamera() {
         if ( camera_pan == null ) {
             return;
         }

         double position = camera_pan.getPosition();

         // pan right
         if (position == 0.0 || position > 0.5) {
             position = Math.min(position + 0.2, 1);
         }

         // pan left
         else if (position == 1.0 || position < 0.5 ) {
             position = Math.max(position - 0.2, 0);
         }

         camera_pan.setPosition(position);
     }


    /**
     * isStalled()
     *
     * This method determine if the robot is stalled by checking every 2 seconds if is has moved significantly.  It uses
     * class properties to keep track of current and previous positions, and it accrues the stallProbability each time it failed
     * to detect movement.  stallProbability is reset whenever movement is detected.
     *
     * NOTE:  This method is used as a condition to break a movement loop.
     *
     * @return true|false
     */
    protected boolean isStalled()
    {

        // check every 2s if position has changed significantly (current-previous > 1in.).  If it did, raise a flag
        if ((int) runtime.seconds() % 2 == 0) {
            if (!hasMovedSignificantly(botPreviousPlacement, botCurrentPlacement)) {
                stallProbability++;
            } else {
                stallProbability = 0;
            }

            botPreviousPlacement = botCurrentPlacement;
            botCurrentPlacement = navigation.getPlacement();
        }

        if ( stallProbability > ATTEMPTS_BEFORE_STALL_DETECTED ) {


            return true;
        }

        return false;
    }


    /**
     * hasMovedSignificantly()
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
     * isPositionWithinAcceptableTargetRange()
     *
     * @param currentPlacement
     * @param finalDestination
     * @return              : true if position is within target range according to the rule ERROR_POSITION_COUNT
     */
    public boolean isPositionWithinAcceptableTargetRange(FieldPlacement currentPlacement, FieldPlacement finalDestination)
    {
        return ( (Math.abs(currentPlacement.x - finalDestination.x) <= CLOSE_ENOUGH_X) && (Math.abs(currentPlacement.y - finalDestination.y) <= CLOSE_ENOUGH_Y));
    }







    /**
     * onHeading()
     *
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

        // Display it for the driver.
//        dbugThis(String.format("Target: %5.2f", angle));
//        dbugThis(String.format("Err/St: %5.2f/%5.2f", error, steer));
//        dbugThis(String.format("Speed : %5.2f:%5.2f", leftSpeed, rightSpeed));

        return onTarget;
    }


    /**
     * getHeadingError()
     *
     * Determines the error between the target angle and the robot's current heading
     *
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getHeadingError(double targetAngle) {

        double robotError;
        double actualAngle;

        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

//        dbugThis("Turning: Stuck at heading: " + actualAngle);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - actualAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;

//        dbugThis("error is : " + robotError);
        return robotError;
    }


    /**
     * getSteer()
     *
     * Returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {

        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     *  This method picks a random value inside the given enum
     *
     * @param clazz :
     * @param <T>
     * @return
     */

    public static <T extends Enum<?>> T randomEnum(Class<T> clazz){

        Random random = new Random();
        int x = random.nextInt(clazz.getEnumConstants().length);
        return clazz.getEnumConstants()[x];
    }


    /**
     * nudgeRobot()
     *
     * Move the robot or the camera, tying to find a beacon
     */
    public void nudgeRobot() {

        if (navigation.getType() == NavigationTypesEnum.VUFORIA) {
            moveCamera();
        }

        else if (navigation.getType() == NavigationTypesEnum.SENSORS) {
            move(randomEnum(TravelDirection.class), 800, true);
        }
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

    void dbugThis(String s) {

        if ( DEBUG == true ) {
            Log.d("OpModesBaseClass: ", s);
        }
    }

    /**
     * isHittingSomething()
     *
     * Given a direction, it will return true if the robot is too close
     *
     * @param direction
     * @return
     */
    public boolean isHittingSomething(TravelDirection direction) {

        double distance = 0.0;
        switch (direction) {
            case FORWARD:
                if ( distanceFront != null && Math.max(distance, distanceFront.getDistance(DistanceUnit.INCH)) <= CLOSE_ENOUGH_Y) {
                    return true;
                }
                break;
            case BACKWARD:
                if ( distanceBack != null && Math.max(distance, distanceBack.getDistance(DistanceUnit.INCH)) <= CLOSE_ENOUGH_Y) {
                    return true;
                }
                break;
            case LEFT:
                if ( distanceLeft != null && Math.max(distance, distanceLeft.getDistance(DistanceUnit.INCH)) <= CLOSE_ENOUGH_X) {
                    return true;
                }
                break;
            case RIGHT:
                if ( distanceRight != null && Math.max(distance, distanceRight.getDistance(DistanceUnit.INCH)) <= CLOSE_ENOUGH_X) {
                    return true;
                }
                break;
        }
        return false;
    }
}