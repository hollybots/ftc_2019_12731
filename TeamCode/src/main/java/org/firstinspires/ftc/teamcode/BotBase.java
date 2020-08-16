package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utils.BlinkinBling;


/**
 * This class abstracts the Robot propulsion sytem.
 * It can be used with any types of wheel or motors.
 */

public class BotBase {

    static final boolean DEBUG = true;


    /**
     * DC MOTORS RELATED CONSTANTS
     */
    static final double TORQUENADO_COUNTS_PER_MOTOR_REV = 1440;                 // eg: REV Motor Encoder
    static final double NEVEREST40_COUNTS_PER_MOTOR_REV = 1120;                 // eg: REV Motor Encoder

    static final double PROPULSION_DRIVE_GEAR_REDUCTION = 1.3;                  // This is < 1.0 if geared UP > 1 we are gering down (the small drives the big)
    static final double WHEEL_CIRCUMFERENCE = 4.0 * 3.14159;        // For figuring circumference
    static final double PROPULSION_ENCODER_COUNTS_PER_INCH = (TORQUENADO_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;


    // Timekeeper OpMode members.
    ElapsedTime runtime = new ElapsedTime();

    // Robot Hardware
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;

    // Decorative LEDs
    private BlinkinBling bling = null;


    /**
     * initRobot()
     * <p>
     * Configure the Hardware according to the Team Hardware Spreadsheet.
     */
    public BotBase(HardwareMap hardwareMap) {


        /* ************************************
            DC MOTORS
        */

        /**
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         *
         * NOTE
         * ====
         * Most robots need the motor on one side to be reversed to drive forward
         * Reverse the motor that runs backwards when connected directly to the battery
         */
        try {
            frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            frontLeftDrive = null;
            Log.d("BOTBASE: ", "Cannot intialize frontLeftDrive");
        }
        try {
            frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            frontRightDrive = null;
            Log.d("BOTBASE: ", "Cannot intialize frontRightDrive");
        }
        try {
            rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
            rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            rearLeftDrive = null;
            Log.d("BOTBASE: ", "Cannot intialize rearLeftDrive");
        }
        try {
            rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
            rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
            rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            rearRightDrive = null;
            Log.d("BOTBASE: ", "Cannot intialize rearRightDrive");
        }


        /* ***********************************
            LED LIGHTS
         */
        try {
            bling = new BlinkinBling(hardwareMap);
        } catch (Exception e) {
            bling = null;
            Log.d("BOTBASE: ", "Cannot intialize Bling");
        }

    }

    public void stop() {
        if (frontLeftDrive != null) {
            frontLeftDrive.setPower(0.0);
        }
        if (frontRightDrive != null) {
            frontRightDrive.setPower(0.0);
        }
        if (rearLeftDrive != null) {
            rearLeftDrive.setPower(0.0);
        }
        if (rearRightDrive != null) {
            rearRightDrive.setPower(0.0);
        }
    }

    public DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public DcMotor getRearRightDrive() {
        return rearRightDrive;
    }

    public DcMotor getRearLeftDrive() {
        return rearLeftDrive;
    }

    protected BlinkinBling getBling() {
        return bling;
    }


    public void setBling(double mode) {
        if (bling != null) {
            bling.setBlinkinPattern(mode);
        }
    }


    void dbugThis(String s) {

        if (DEBUG == true) {
            Log.d("BOTTOP: ", s);
        }
    }
}
