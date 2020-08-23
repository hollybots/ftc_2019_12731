package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class TensorFlowObjectIdentification implements ObjectIdentificationInterface
{

    private HardwareMap hardwareMap                     = null;
    private Telemetry telemetry                         = null;

    // VuForia Key, register online
    protected static final String VUFORIA_KEY = "AQSpIWb/////AAABmarkN2D6nEtEtiFZY75jnL8H12+mIcvqBOz+WbP/qxrgyY4JZJzBTN7NjGGX8q2gu0M06DZpIqUq8R0c06ZoXTOT/77I4+Hp8s4hwMr5jcZvqzq9TGgg/83hGs48KIRNW0kRiWWJulpUG8v+c+jNBW3csk/Un2yofaPK61SkPAQkLaddWk7j4zMZfk1lOaRv4H11MTX3g12DB2eVjRcv8jGC3Wt0T5q+zll0iLjqpegF2FrL2dxDyEb7dyfQJ+5OlnqotKESWhCMDEBTJZKgopJTA7Rnf5uUdLVzRGV2S0VoJFwG/eYc+SaP6jx1dTt3SqTEzs3GbY9u4HQms03n7WavQtc20U4SKU9eyXvhS6NX";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private String targetName = null;


    protected  List<Recognition> lastUpdatedRecognitions = null;

    public TensorFlowObjectIdentification(
        HardwareMap hardwareMap,
        Telemetry telemetry,
        String modelAssetName,      // identifiable models file ressource
        String [] assetNames,       // Name of the identifiable asset within the file, important
        String targetName,
        WebcamName webcam,
        boolean debug)
    {

        this.hardwareMap    = hardwareMap;
        this.telemetry      = telemetry;
        this.targetName     = targetName;

        /**
         * Initialize the Vuforia localization engine.
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(modelAssetName, assetNames);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }

    public FieldPlacement find()
    {
        FieldPlacement placement = null;

        if (tfod == null) {
            return null;
        }

        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions == null) {
            return null;
        }

        telemetry.addData("# Object Detected", recognitions.size());
        // step through the list of recognitions and display boundary info.
        int i = 0;

        // ther emigh be multpi objects to recognize here.  We are only interestd in one
        for (Recognition recognition : recognitions) {

            if (recognition.getLabel() != targetName) {
                continue;
            }

            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());

            placement = new FieldPlacement(recognition.getRight(), recognition.getBottom());
        }
        telemetry.update();
        return placement;
    }

    public void stop()
    {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private TFObjectDetector initTfod(String modelAssetName, String [] assetsLabel) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId",
            "id",
            hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(modelAssetName, assetsLabel);

        return tfod;
    }
}
