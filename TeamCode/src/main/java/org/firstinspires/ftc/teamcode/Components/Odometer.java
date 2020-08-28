package org.firstinspires.ftc.teamcode.Components;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;



public class Odometer {

    static final boolean DEBUG = true;

    // defined by manufacturer
    final double COUNTS_PER_INCH                = 307.699557;

    /**
     * HARDWARE
     */
    private DcMotor verticalEncoderLeft         = null;
    private String verticalEncoderLeftName      = "encoder_1";
    private DcMotor verticalEncoderRight        = null;
    private String verticalEncoderRightName     = "encoder_2";
    private DcMotor horizontalEncoder           = null;
    private String horizontalEncoderName        = "encoder_3";

    /**
     * STATE VARIABLES
     */

    // Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition        = 0;
    double verticalLeftEncoderWheelPosition         = 0;
    double horizontalEncoderWheelPosition           = 0;
    double changeInRobotOrientation                 = 0;

    private double robotGlobalXCoordinatePosition   = 0;
    private double robotGlobalYCoordinatePosition   = 0;
    private double robotOrientationRadians          = 0;

    private double previousVerticalRightEncoderWheelPosition    = 0;
    private double previousVerticalLeftEncoderWheelPosition     = 0;
    private double prevHorizontalEncoderWheelPosition               = 0;


    /**
     * CONSTANTS FROM CONFIGURATION FILE
     */
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    private int verticalLeftEncoderPositionMultiplier       = 1;
    private int verticalRightEncoderPositionMultiplier      = 1;
    private int normalEncoderPositionMultiplier             = 1;


    /****
     *
     * @param hardwareMap
     */
    public Odometer(HardwareMap hardwareMap)
    {
        initDriveHardwareMap(hardwareMap);

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    /**
     * Init the hardware
     * @param hardwareMap
     */
    private void initDriveHardwareMap(HardwareMap hardwareMap)
    {
        try {
            verticalEncoderLeft = hardwareMap.dcMotor.get(verticalEncoderLeftName);
            verticalEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalEncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            verticalEncoderLeft = null;
            dbugThis("Can't initialize " + verticalEncoderLeftName);
        }
        try {
            verticalEncoderRight = hardwareMap.dcMotor.get(verticalEncoderRightName);
            verticalEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalEncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            verticalEncoderRight = null;
            dbugThis("Can't initialize " + verticalEncoderRightName);
        }
        try {
            horizontalEncoder = hardwareMap.dcMotor.get(horizontalEncoderName);
            horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            horizontalEncoder = null;
            dbugThis("Can't initialize " + horizontalEncoderName);
        }
    }


    /**
     * This is the method should run in the main event loop.  It will update the global (x, y, theta) coordinate position of the robot using the encoders
     */
    public void globalCoordinatePositionUpdate()
    {
        if ((verticalEncoderLeft == null) && (verticalEncoderLeft == null)) {
            return;
        }

        if (horizontalEncoder == null) {
            return;
        }

        double leftChange = 0;
        double rightChange = 0;

        // Get Current Positions
        if (verticalEncoderLeft != null) {
            verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
            leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        }

        if (verticalEncoderRight != null) {
            verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);
            rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;
        }

        // Calculate Angle if we have 2 encoders, if not don't
        if ((verticalEncoderLeft != null) && (verticalEncoderRight != null)) {
            changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
            robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));
        }

        // Get the components of the motion
        horizontalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = horizontalEncoderWheelPosition - prevHorizontalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);


        double horizontalDisplacement = horizontalChange;
        double verticalDisplacement = 0;

        int verticalComponents = 0;
        if (verticalEncoderLeft != null) {
            verticalDisplacement +=  rightChange;
            verticalComponents++;
        }
        if (verticalEncoderRight != null) {
            verticalDisplacement += rightChange;
            verticalComponents++;
        }
        if (verticalComponents > 0) {
            verticalDisplacement /= verticalComponents;
        }

        // Calculate and update the actual position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (verticalDisplacement*Math.sin(robotOrientationRadians) + horizontalDisplacement*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (verticalDisplacement*Math.cos(robotOrientationRadians) - horizontalDisplacement*Math.sin(robotOrientationRadians));

        // Update state variables
        previousVerticalLeftEncoderWheelPosition        = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition       = verticalRightEncoderWheelPosition;
        prevHorizontalEncoderWheelPosition              = horizontalEncoderWheelPosition;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double getCurrentXPos(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double getCurrentYPos(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }


    public void reverseLeftEncoder()
    {
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder()
    {
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseHorizontalEncoder()
    {
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }

    void dbugThis(String s) {
        if (DEBUG == true) {
            Log.d("ODOMETER: ", s);
        }
    }
}
