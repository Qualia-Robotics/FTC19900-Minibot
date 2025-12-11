package org.firstinspires.ftc.team28420;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "camera", group = "test")
public class CameraTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize AprilTag detection
        initAprilTag();

        // Display initialization message
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Touch START to begin AprilTag detection");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Display AprilTag detection data
                telemetryAprilTag();

                // Push telemetry to Driver Station
                telemetry.update();

                // Share the CPU
                sleep(20);
            }
        }

        // Close camera when done
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor and vision portal.
     */
    private void initAprilTag() {
        // Create the AprilTag processor with default settings
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal for webcam
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    /**
     * Display telemetry data about detected AprilTags.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Display info for each detected AprilTag
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Known AprilTag - display full pose information
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                // Unknown AprilTag - display basic info
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y));
            }
        }

        // Add key for understanding the telemetry data
        telemetry.addLine("\nKey:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) distance");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
