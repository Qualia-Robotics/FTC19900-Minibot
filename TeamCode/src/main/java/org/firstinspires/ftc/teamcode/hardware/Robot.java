package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;

public class Robot {

    public DriveSubsystem drive;
    //public IntakeSubsystem intake;
    public ShooterController shooter;

    public LimeLightSubsystem limelight;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize all subsystems in a controlled, predictable order
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterController(hardwareMap);
        limelight = new LimeLightSubsystem(hardwareMap);
        //intake = new IntakeSubsystem(hardwareMap);
    }

    public void addTelemetry(Telemetry telemetry) {
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        limelight.addTelemetry(telemetry);
    }

    /** Called every control loop — optional, but great practice */
    public void periodic() {
        drive.periodic();
        shooter.update();
        limelight.periodic();
        // Intake may not need periodic update
    }
}
