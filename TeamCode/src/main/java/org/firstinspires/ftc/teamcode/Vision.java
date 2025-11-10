package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Vision {
    private Limelight3A limelight;
    private LLStatus status;
    private LLResult result;
    private boolean detecting;
    private double targetOffsetAngle_Verticle;
    private final double limelightMountAngleDegrees = 11;
    private final double limelightLensHeightInches = 10.0;
    private final double goalHeightInches = 29.5;
    private double angleToGoalDegrees;
    private double angleToGoalRadians;
    private double distanceFromLimelightToGoalInches;
    public Vision(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        status = limelight.getStatus();
        result = limelight.getLatestResult();
        detecting = result.isValid();;
    }

    public Limelight3A getLimelight()
    {
        return this.limelight;
    }
    public LLStatus getStatus()
    {
        return this.status;
    }
    public LLResult getResult()
    {
        return this.result;
    }
    public boolean detecting()
    {
        return this.detecting;
    }
    public double findDistance()
    {
        if(this.detecting)
        {
            targetOffsetAngle_Verticle = (result.getTy());
            angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Verticle;
            angleToGoalRadians = angleToGoalDegrees * (Math.PI/180.0);
            distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            return this.distanceFromLimelightToGoalInches;
        }
        else
        {
            return 0.0;
        }
    }
    public double getTx()
    {
        return this.result.getTx();
    }
    public double getTargetVelocity()
    {
        return .2214*Math.pow(this.distanceFromLimelightToGoalInches,2) - 12.7553*this.distanceFromLimelightToGoalInches + 1216.8651;
    }
    public void recordVisionTelemetry(Telemetry telemetry)
    {
        if (this.result.isValid())
        {
            // Access general information
            Pose3D botpose = this.result.getBotpose();
            double captureLatency = this.result.getCaptureLatency();
            double targetingLatency = this.result.getTargetingLatency();
            double parseLatency = this.result.getParseLatency();
            targetOffsetAngle_Verticle = (this.result.getTy());
            angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Verticle;
            angleToGoalRadians = angleToGoalDegrees * (Math.PI/180.0);
            distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            //attempt one
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", this.result.getTx());
            telemetry.addData("txnc", this.result.getTxNC());
            telemetry.addData("ty", this.result.getTy());
            telemetry.addData("tync", this.result.getTyNC());
            telemetry.addData("tarea", this.result.getTa());
            telemetry.addData("angleToGoalDegrees", angleToGoalDegrees);
            telemetry.addData("angleToGoalRadians", angleToGoalRadians);
            telemetry.addData("distanceFromLimelightToGoalInches", distanceFromLimelightToGoalInches);
            telemetry.addData("Target Velocity", this.getTargetVelocity());
            telemetry.addData("Botpose", botpose.toString());

            // Access barcode results
            List<LLResultTypes.BarcodeResult> barcodeResults = this.result.getBarcodeResults();
            for (LLResultTypes.BarcodeResult br : barcodeResults)
            {
                telemetry.addData("Barcode", "Data: %s", br.getData());
            }

            // Access classifier results
            List<LLResultTypes.ClassifierResult> classifierResults = this.result.getClassifierResults();
            for (LLResultTypes.ClassifierResult cr : classifierResults)
            {
                telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
            }

            // Access detector results
            List<LLResultTypes.DetectorResult> detectorResults = this.result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults)
            {
                telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
            }

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = this.result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults)
            {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            // Access color results
            List<LLResultTypes.ColorResult> colorResults = this.result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults)
            {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
        }
        else
        {
            telemetry.addData("Limelight", "No data available");
        }
    }



}
