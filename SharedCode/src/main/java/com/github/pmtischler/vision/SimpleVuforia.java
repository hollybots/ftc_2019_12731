package com.github.pmtischler.vision;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Reads Vuforia markers off the camera.
 * Requires the Vuforia key is set in SharedCode/src/main/res/values/vuforia.xml.
 * Example:
 *   private SimpleVuforia vuforia;
 *
 *   public void init() {
 *       String vuforiaKey = "...";
 *       vuforia = new SimpleVuforia(vuforiaKey);
 *   }
 *
 *   public void loop() {
 *       RelicRecoveryVuMark vuMark = vuforia.detectMark();
 *       if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
 *           // ... use detected mark.
 *       }
 *   }
 */
public class SimpleVuforia {
    /**
     * Creates a Vuforia localizer and starts localization.
     * @param vuforiaLicenseKey The license key to access Vuforia code.
     */
    public SimpleVuforia(String vuforiaLicenseKey) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = vuforiaLicenseKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    /**
     * Returns the last detected mark type.
     */
    public RelicRecoveryVuMark detectMark() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }


    // The external Vuforia ID localizer.
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
}
