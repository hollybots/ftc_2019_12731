package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BotBase {

    static final double TORQUENADO_COUNTS_PER_MOTOR_REV        = 1440;                 // eg: REV Motor Encoder
    static final double NEVEREST40_COUNTS_PER_MOTOR_REV        = 1120;                 // eg: REV Motor Encoder

    static final double PROPULSION_DRIVE_GEAR_REDUCTION     = 1.3;                  // This is < 1.0 if geared UP > 1 we are gering down (the small drives the big)
    static final double WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;        // For figuring circumference
    static final double PROPULSION_ENCODER_COUNTS_PER_INCH  = (TORQUENADO_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;


    private BlinkinBling bling                              = null;


    // Timekeeper OpMode members.
    ElapsedTime runtime = new ElapsedTime();

    // Robot Hardware
    private DcMotor     frontLeftDrive     = null;
    private DcMotor     frontRightDrive    = null;
    private DcMotor     rearLeftDrive     = null;
    private DcMotor     rearRightDrive    = null;


    /**
     * initRobot()
     *
     *    Configure the Hardware according to the Team Hardware Spreadsheet.
     */
     BotBase(HardwareMap hardwareMap) {


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
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* ***********************************
            LIGHTS
         */
        bling = new BlinkinBling(hardwareMap);

    }

    protected void stop() {
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        rearLeftDrive.setPower(0.0);
        rearRightDrive.setPower(0.0);
    }

    protected DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    protected DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    protected DcMotor getRearRightDrive() {
        return rearRightDrive;
    }

    protected DcMotor getRearLeftDrive() {
        return rearLeftDrive;
    }

    protected BlinkinBling getBling() { return bling;}


    public void setBling(double mode) {
        if ( bling != null ) {
            bling.setBlinkinPattern(mode);
        }
    }
}
