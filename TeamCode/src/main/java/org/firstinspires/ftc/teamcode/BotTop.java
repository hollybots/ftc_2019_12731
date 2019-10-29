package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class BotTop {

    static final boolean DEBUG                                  = true;


    static final double TORQUENADO_COUNTS_PER_MOTOR_REV        = 1440;                 // eg: REV Motor Encoder
    static final double NEVEREST40_COUNTS_PER_MOTOR_REV        = 1120;                 // eg: REV Motor Encoder

    static final double PROPULSION_DRIVE_GEAR_REDUCTION     = 1.3;                  // This is < 1.0 if geared UP > 1 we are gering down (the small drives the big)
    static final double WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;        // For figuring circumference
    static final double PROPULSION_ENCODER_COUNTS_PER_INCH  = (TORQUENADO_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;


    static final double SPEED_COIL                          = 0.5;
    static final int    COIL_UP_DIRECTION                   = 1;
    static final int    COIL_DOWN_DIRECTION                 = -1;
    static final double COIL_UP_COMMAND                     = -0.99;
    static final double COIL_DOWN_COMMAND                   = 0.99;
    static final int    COIL_IDLE_DIRECTION                  = 0;

    static final double SPEED_SWING                         = 0.2;
    static final int    SWING_UP_DIRECTION                  = -1;
    static final int    SWING_DOWN_DIRECTION                = 1;
    static final double SWING_UP_COMMAND                    = -0.99;
    static final double SWING_DOWN_COMMAND                  = 0.99;
    static final int    SWING_IDLE_DIRECTION                = 0;

    static final double POS_OPEN_CLAW_RIGHT                     = 0.0;
    static final double POS_CLOSE_CLAW_RIGHT                    = 0.5;
    static final double POS_OPEN_CLAW_LEFT                      = 0.99;
    static final double POS_CLOSE_CLAW_LEFT                     = 0.4;

    static final double POS_PIN_UP_RIGHT                    = 0.0;
    static final double POS_PIN_DOWN_RIGHT                  = 0.7;
    static final double POS_PIN_UP_LEFT                     = 0.0;
    static final double POS_PIN_DOWN_LEFT                   = 0.65;



    // Timekeeper OpMode members.
    ElapsedTime runtime = new ElapsedTime();

    /* ************************************
        CD Motors
    */
    private DcMotor     armLiftSwing                    = null;
    private DcMotor     linearMotionCoil                = null;


    /* ************************************
        CR SErvos
    */
    private CRServo     rackAndPinionSlide              = null;


    /* ************************************
        SERVOS
    */
    Servo rightClaw     = null;
    Servo leftClaw      = null;
    Servo rightPin      = null;
    Servo leftPin       = null;


    /* ************************************
        LIMIT SWITCHES
    */
    DigitalChannel swingLimitUp = null;
    DigitalChannel swingLimitDown = null;

    DigitalChannel coilLimitUp = null;
    DigitalChannel coilLimitDown = null;


    /**
     * State variables
     */
    private int  coilMovementDirection      = COIL_IDLE_DIRECTION;
    private int  armMovementDirection       = SWING_IDLE_DIRECTION;


    /**
     * initRobot()
     *
     *    Configure the Hardware according to the Team Hardware Spreadsheet.
     */
    BotTop(HardwareMap hardwareMap) {


        /* ************************************
            DC MOTORS
        */
        try {
            armLiftSwing  = hardwareMap.get(DcMotor.class, "arm_lift_swing");
            armLiftSwing.setDirection(DcMotor.Direction.FORWARD);
            armLiftSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize armLiftSwing");
            armLiftSwing = null;
        }


        /**
         * RACK & PINION
         */
        try {
            rackAndPinionSlide = hardwareMap.get(CRServo.class, "rack_and_pinion_slide");
        }
        catch (Exception e) {
            dbugThis("Cannot initialize Rack and Pinion");
            rackAndPinionSlide = null;
        }

        /**
         * LINEAR MOTION COIL
         */
        try {
            linearMotionCoil = hardwareMap.get(DcMotor.class, "linear_motion_coil");
            linearMotionCoil.setDirection(DcMotor.Direction.FORWARD);
            linearMotionCoil.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize linearMotionCoil");
            linearMotionCoil = null;
        }

        /**
         * CLAW Mechanism
         */
        try {
            rightClaw                       = hardwareMap.get(Servo.class, "right_claw");
        }
        catch (Exception e) {
            dbugThis("Cannot initialize rightClaw");
            rightClaw = null;
        }
        try {
            leftClaw                        = hardwareMap.get(Servo.class, "left_claw");
        }
        catch (Exception e) {
            dbugThis("Cannot initialize leftClaw");
            leftClaw = null;
        }

        /**
         * LIMIT Switches for safety
         */
        try {
            swingLimitUp                         = hardwareMap.get(DigitalChannel.class, "swing_limit_up");
            swingLimitUp.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize swingLimitUp");
            swingLimitUp = null;
        }
        try {
            swingLimitDown                       = hardwareMap.get(DigitalChannel.class, "swing_limit_down");
            swingLimitDown.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize swingLimitDown");
            swingLimitDown = null;
        }

        try {
            coilLimitUp                          = hardwareMap.get(DigitalChannel.class, "coil_limit_up");
            coilLimitUp.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize coilLimitUp");
            coilLimitUp = null;
        }
        try {
            coilLimitDown                        = hardwareMap.get(DigitalChannel.class, "coil_limit_down");
            coilLimitDown.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize coilLimitDown");
            coilLimitDown = null;
        }

        /**
         * TRAY HOOK Mechanism
         */
        try {
            rightPin = hardwareMap.get(Servo.class, "right_pin");
        } catch (Exception e) {
            dbugThis("Cannot initialize rightPin");
            rightPin = null;
        }
        try {
            leftPin = hardwareMap.get(Servo.class, "left_pin");
        } catch (Exception e) {
            dbugThis("Cannot initialize leftPin");
            leftPin = null;
        }
    }

    public void stopAll() {
        if (linearMotionCoil != null) {
            linearMotionCoil.setPower(0.0);
        }
        if (armLiftSwing != null) {
            armLiftSwing.setPower(0.0);
        }
        if (rackAndPinionSlide != null) {
            rackAndPinionSlide.setPower(0.0);
        }
    }

    public DcMotor getCoil() {
        return linearMotionCoil;
    }

    public CRServo getSlide() {
        return rackAndPinionSlide;
    }


    /**
     * BLOCK CLAW
     */
    public void openClaw() {
        if (rightClaw != null) {
            rightClaw.setPosition(POS_OPEN_CLAW_RIGHT);
        }
        if (leftClaw != null) {
            leftClaw.setPosition(POS_OPEN_CLAW_LEFT);
        }
    }

    public void closeClaw() {
        if (rightClaw != null) {
            rightClaw.setPosition(POS_CLOSE_CLAW_RIGHT);
        }
        if (leftClaw != null) {
            leftClaw.setPosition(POS_CLOSE_CLAW_LEFT);
        }
    }

    /**
     * RACK AND PINION SLIDE
     */
    public void slideUp() {
        if (rackAndPinionSlide == null ) {
            return;
        }
        rackAndPinionSlide.setPower(-0.5);
    }

    public void slideDown() {
        if (rackAndPinionSlide == null ) {
            return;
        }
        rackAndPinionSlide.setPower(0.5);
    }

    public void stopSlide() {
        if (rackAndPinionSlide == null ) {
            return;
        }
        rackAndPinionSlide.setPower(0.0);
    }




    /**
     * TRAY CLAMPS METHODS
     */

    public void clampOn() {
        if (rightPin == null || leftPin == null) {
            return;
        }
        rightPin.setPosition(POS_PIN_DOWN_RIGHT);
        leftPin.setPosition(POS_PIN_DOWN_LEFT);
    }

    public void clampRelease() {
        if (rightPin == null || leftPin == null) {
            return;
        }
        rightPin.setPosition(POS_PIN_UP_RIGHT);
        leftPin.setPosition(POS_PIN_UP_LEFT);
    }


    /**
     * ARM SWING METHODS
     */


    /**
     * Checks the state of the limit switch for the arm limit up.
     * If there is no limit switch, it returns false.
     *
     * @return
     */
    private boolean isSwingLimitUp() {
        if (swingLimitUp == null) {
            return false;
        }
        return !(swingLimitUp.getState() == true );
    }

    /**
     * Checks the state of the limit switch for the arm limit down.
     * If there is no limit switch, it returns false.
     *
     * @return
     */
    private boolean isSwingLimitDown() {
        if (swingLimitDown == null) {
            return false;
        }
        return !(swingLimitDown.getState() == true );
    }


    /**
     * Check the state of the limit switch and the direction of the movement
     * as kept in the state variable armMovementDirection.  It returns true if
     * the limit condition is met.
     *
     * If the limit condition is met, the motor is stopped
     * @return
     */
    private boolean isArmAtLimit() {
        if (armMovementDirection == SWING_UP_DIRECTION  && isSwingLimitUp()) {
            armMovementDirection = SWING_IDLE_DIRECTION;
            armLiftSwing.setPower(0.0);
            return true;
        }

        if (armMovementDirection == SWING_DOWN_DIRECTION  && isSwingLimitDown()) {
            armMovementDirection = SWING_IDLE_DIRECTION;
            armLiftSwing.setPower(0.0);
            return true;
        }

        return false;
    }


    /**
     * This method is used to move the swinging arm up or down,
     * it takes a command directly from the Gamepad joystick (up -> lift arm, down -> lower arm)
     * or you can pass in a predefined command SWING_UP_COMMAND or SWING_DOWN_COMMAND.
     *
     * In either case the arm is move at a predefined speed = SPEED_SWING
     *
     * @param command
     */
    public void swing(double command) {

        if (armLiftSwing == null) {
            return;
        }

        if ( ( command > 0 ) || (command == SWING_UP_COMMAND) ) {
            armMovementDirection = SWING_UP_DIRECTION;
        }
        else if ( (command < 0) || (command == SWING_DOWN_COMMAND) ){
            armMovementDirection = SWING_DOWN_DIRECTION;
        }
        else {
            armMovementDirection = SWING_IDLE_DIRECTION;
            armLiftSwing.setPower(0.0);
            return;
        }

        if (!isArmAtLimit()) {
            linearMotionCoil.setPower(armMovementDirection * SPEED_SWING);
        }

    }



    /**
     * LINEAR MOTION CONTROL
     */

    /**
     * Checks the state of the limit switch for the coil limit up.
     * If there is no limit switch, it returns false.
     *
     * @return
     */
    private boolean isCoilLimitUp() {

        if (coilLimitUp == null) {
            return false;
        }

        return !(coilLimitUp.getState() == true );
    }


    /**
     * Checks the state of the limit switch for the coil limit down.
     * If there is no limit switch, it returns false.
     *
     * @return
     */
    private boolean isCoilLimitDown() {

        if (coilLimitDown == null) {
            return false;
        }

        return !(coilLimitDown.getState() == true );
    }



    /**
     * Check the state of the limit switch and the direction of the movement
     * as kept in the state variable coilMovementDirection.  It returns true if
     * the limit condition is met.
     *
     * If the limit condition is met, the motor is stopped
     * @return
     */
    private boolean isCoilAtLimit() {

        if (coilMovementDirection == COIL_UP_DIRECTION  && isCoilLimitUp()) {
            coilMovementDirection = COIL_IDLE_DIRECTION;
            linearMotionCoil.setPower(0.0);
            return true;
        }

        if (coilMovementDirection == COIL_DOWN_DIRECTION  && isCoilLimitDown()) {
            coilMovementDirection = COIL_IDLE_DIRECTION;
            linearMotionCoil.setPower(0.0);
            return true;
        }

        return false;
    }



    /**
     * This method is used to coil/uncoil the linear motion thread.
     * it takes a command directly from the Gamepad joystick (up -> coil, down -> uncoil)
     * or you can pass in a predefined command COIL_UP_COMMAND or COIL_DOWN_COMMAND.
     *
     * In either case coiling/uncoiling is done at a predefined speed = SPEED_COIL
     *
     * @param command
     */
    public void coil(double command) {


        if (linearMotionCoil == null) {
            return;
        }

        if ( (command < 0) || (command == COIL_UP_COMMAND) ) {
            coilMovementDirection = COIL_UP_DIRECTION;
        }
        else if ((command > 0) || (command == COIL_DOWN_COMMAND)) {
            coilMovementDirection = COIL_DOWN_DIRECTION;
        }
        else {
            coilMovementDirection = COIL_IDLE_DIRECTION;
            linearMotionCoil.setPower(0.0);
            return;
        }

        if (!isCoilAtLimit()) {
            linearMotionCoil.setPower(coilMovementDirection * SPEED_COIL);
        }
    }



    /**
     * Checks the current direction the motors and the states of the limit switches for
     * all movement limited by lijmit switches.
     *
     * This should be used inside ALL while loops in autonomous mode
     *
     * @return
     */
    public boolean checkAllLimitSwitches() {
        return isCoilAtLimit() || isArmAtLimit();
    }


    void dbugThis(String s) {

        if ( DEBUG == true ) {
            Log.d("BOTTOP: ", s);
        }
    }
}
