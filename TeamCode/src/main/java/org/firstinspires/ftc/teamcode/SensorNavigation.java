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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class SensorNavigation {


    // Since sensor gives distance in cm, we must use cm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float inchFTCFieldWidth      = (12 * 6);       // the width of the FTC field (from the center point to the outer panels)

    private OpenGLMatrix lastLocation = null;

    private Telemetry telemetry;
    private boolean DEBUG               = false;

    private HardwareMap hardwareMap;

    double distanceFront;
    double distanceRear;
    double distanceLeft;
    double distanceRight;

    List<ModernRoboticsI2cRangeSensor> frontDistanceSensors;
    List<ModernRoboticsI2cRangeSensor> rearDistanceSensors;
    List<ModernRoboticsI2cRangeSensor> leftDistanceSensors;
    List<ModernRoboticsI2cRangeSensor> rightDistanceSensors;


    /**
     * This method initializes this class with the parameters needed to orient our robot using the Distance sensors on the robot
     *
     * @param hardwareMap
     * @param telemetry
     * @param debug: in debug mode : true
     */
    public SensorNavigation(HardwareMap hardwareMap,
                            int nbFrontSensors,
                            double distanceFront,
                            int nbRearSensors,
                            double distanceRear,
                            int nbLeftSensors,
                            double distanceLeft,
                            int nbRightSensors,
                            double distanceRight,
                            Telemetry telemetry,
                            boolean debug)
    {
        this.DEBUG              = debug;
        this.telemetry          = telemetry;
        this.hardwareMap        = hardwareMap;
        this.distanceFront      = distanceFront;
        this.distanceRear       = distanceRear;
        this.distanceLeft       = distanceLeft;
        this.distanceRight      = distanceRight;


        frontDistanceSensors       = new ArrayList();
        rearDistanceSensors        = new ArrayList();
        leftDistanceSensors        = new ArrayList();
        rightDistanceSensors       = new ArrayList();

        /*********************************************
         * Configure Sensor Navigation
         */
        for (int i=0; i<nbFrontSensors; i++) {
            frontDistanceSensors.add(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_distance" + i));
        }
        for (int i=0; i<nbRearSensors; i++) {
            rearDistanceSensors.add(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rear_distance" + i));
        }
        for (int i=0; i<nbLeftSensors; i++) {
            leftDistanceSensors.add(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_distance" + i));
        }
        for (int i=0; i<nbRightSensors; i++) {
            rightDistanceSensors.add(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_distance" + i));
        }
    }


    public void activate() {

    }

    public void stop() {

    }


    public double getDistanceFromSensors(SensorLocations location) {

        List<ModernRoboticsI2cRangeSensor> sensors;

        switch (location) {
            case FRONT:
                sensors = frontDistanceSensors;
                break;
            case REAR:
                sensors = rearDistanceSensors;
                break;
            case LEFT:
                sensors = leftDistanceSensors;
                break;
            case RIGHT:
            default:
                sensors = rightDistanceSensors;
                break;
        }
        double distance = 0.0;
        for (int i=0; i < sensors.size(); i++) {
            distance = Math.max(distance, sensors.get(i).getDistance(DistanceUnit.INCH));
        }

        return distance;
    }



    public FieldPlacement getPlacement() {

        double y1,y2, x1, x2;

        FieldPlacement placement = null;

        /* We have a row of sensors in the front, back, and sides.  Each row of sensor contribute to the same
           distance reading.  By having many sensors, we prevent using the value of a obstructed sensor.
         */
        double front    = getDistanceFromSensors(SensorLocations.FRONT);
        double rear     = getDistanceFromSensors(SensorLocations.REAR);
        double left     = getDistanceFromSensors(SensorLocations.LEFT);
        double right    = getDistanceFromSensors(SensorLocations.RIGHT);

        /* With this type of navigation, the robot is always facing North (0 degree) which is facing the Blue Alliance box,
            while standing in the Red Alliance box
         */

        /* Find the location of the center of the robot */
        y1 = inchFTCFieldWidth - (front + distanceFront);
        y2 = -inchFTCFieldWidth + (rear + distanceRear);
        x1 = inchFTCFieldWidth - (right + distanceRight);
        x2 = -inchFTCFieldWidth + (left + distanceLeft);

        dbugThis("y1:" + y1);
        dbugThis("y2:" + y2);
        dbugThis("x1:" + x1);
        dbugThis("x2:" + x2);

        return  new FieldPlacement((x1 + x2) / 2.0, (y1 + y2) / 2.0);
    }


    private void dbugThis(String s) {

        if ( this.DEBUG == true ) {
            Log.d("SENSOR NAVIGATION:", s);
        }
    }

}