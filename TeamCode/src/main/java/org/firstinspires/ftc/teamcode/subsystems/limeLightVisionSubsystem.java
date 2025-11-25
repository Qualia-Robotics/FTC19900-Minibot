package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class limeLightVisionSubsystem {
    // Vision Variables
    private PredominantColorProcessor colorSensor;
    private VisionPortal visionPortal;

    // Color Result
    private PredominantColorProcessor.Result result;

    // Lime Light Instance Builder
    public limeLightVisionSubsystem(HardwareMap hardwareMap) {
        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        // Initialize the VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(620, 340))
                .addProcessor(colorSensor)
                .build();
        /**
         * Gets the detected color from the vision processor.
         * @return The Swatch that is the closest match (e.g., RED, BLUE), or null.
         */

        public PredominantColorProcessor.Swatch getDetectedColor() {
            // The getAnalysis() method returns the latest result from the background processing thread
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            if (result != null) {
                return result.closestSwatch;
            }
            return null;
        }
    }
}
