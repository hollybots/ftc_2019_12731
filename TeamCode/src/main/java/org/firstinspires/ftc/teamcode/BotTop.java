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
    static final int    COIL_UP                             = 1;
    static final int    COIL_DOWN                           = -1;
    static final int    COIL_IDLE                           = 0;

    static final double SPEED_SWING                         = 0.2;
    static final int    SWING_UP                            = -1;
    static final int    SWING_DOWN                          = 1;
    static final int    SWING_IDLE                          = 0;

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
     * TRAY CLAMPS
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
     * ARM SWING
     */

    public boolean isSwingLimitUp() {
        if (swingLimitUp == null) {
            return false;
        }
        return !(swingLimitUp.getState() == true );
    }

    public boolean isSwingLimitDown() {
        if (swingLimitDown == null) {
            return false;
        }
        return !(swingLimitDown.getState() == true );
    }


    public void swing(double command) {

        if (armLiftSwing == null) {
            return;
        }

        double movementDirection;

        if ( command > 0 ) {
            movementDirection = SWING_UP;
        }
        else if (command < 0) {
            movementDirection = SWING_DOWN;
        }
        else {
            armLiftSwing.setPower(0.0);
            return;
        }

        if (movementDirection == SWING_UP  && !isSwingLimitUp()) {
            armLiftSwing.setPower(movementDirection * SPEED_SWING);
        }

        else if (movementDirection == SWING_DOWN && !isSwingLimitDown()) {
            armLiftSwing.setPower(movementDirection * SPEED_SWING);
        }

        else {
            armLiftSwing.setPower(0.0);
        }
    }


    /**
     * LINEAR MOTION CONTROL
     */

    public boolean isCoilLimitUp() {

        if (coilLimitUp == null) {
            return false;
        }

        return !(coilLimitUp.getState() == true );
    }

    public boolean isCoilLimitDown() {

        if (coilLimitDown == null) {
            return false;
        }

        return !(coilLimitDown.getState() == true );
    }


    public void coil(double command) {


        if (linearMotionCoil == null) {
            return;
        }


        double movementDirection;

        if ( command < 0 ) {
            movementDirection = COIL_UP;
        }
        else if (command > 0) {
            movementDirection = COIL_DOWN;
        }
        else {
            linearMotionCoil.setPower(0.0);
            return;
        }

        if (movementDirection > 0  && !isCoilLimitUp()) {
            linearMotionCoil.setPower(movementDirection * SPEED_COIL);
        }

        else if (movementDirection < 0 && !isCoilLimitDown()) {
            linearMotionCoil.setPower(movementDirection * SPEED_COIL);
        }

        else {
            linearMotionCoil.setPower(0.0);
        }
    }

    void dbugThis(String s) {

        if ( DEBUG == true ) {
            Log.d("BOTTOP: ", s);
        }
    }


}
