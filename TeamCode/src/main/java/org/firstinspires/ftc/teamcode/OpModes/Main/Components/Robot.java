package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    // All components
    private Turret turret;
    private Launcher launcher;
    private Spindexer spindexer;
    private DriveTrain driveTrain;
    private Lift lift;
    private Intake intake;

    /**
     * Initialize all robot components
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry from OpMode
     * @param opMode LinearOpMode instance (needed for Spindexer)
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        // Initialize all components
        turret = new Turret();
        turret.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, opMode);

        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        lift = new Lift();
        lift.initialize(hardwareMap, telemetry);

        intake = new Intake();
        intake.initialize(hardwareMap, telemetry);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    // Component update methods
    public void updateLauncher() {
        launcher.update();
    }

    /**
     * Update turret alignment
     * @return false if limelight not connected (should exit OpMode)
     */
    public boolean updateTurret() {
        return turret.update();
    }

    public void updateDriveTrain(double forward, double right, double rotate, boolean crossPressed) {
        driveTrain.update(forward, right, rotate, crossPressed);
    }

    public void updateSpindexer(boolean gamepadA, boolean gamepadB, boolean gamepadX, 
                                boolean gamepadY, boolean gamepadLeftBumper) {
        spindexer.update(gamepadA, gamepadB, gamepadX, gamepadY, gamepadLeftBumper);
    }

    public void updateIntake() {
        intake.update();
    }

    // ==================== LAUNCHER DIRECT ACCESS ====================
    
    /**
     * Start the flywheel spinning
     */
    public void startFlywheel() {
        launcher.setSpinning(true);
        launcher.update();
    }

    /**
     * Start the flywheel with specified power
     * @param power Power level (0.0 to 1.0)
     */
    public void startFlywheel(double power) {
        launcher.setPower(power);
        launcher.setSpinning(true);
        launcher.update();
    }

    /**
     * Stop the flywheel
     */
    public void stopFlywheel() {
        launcher.setSpinning(false);
        launcher.update();
    }

    /**
     * Check if flywheel is currently spinning
     * @return true if flywheel is spinning
     */
    public boolean isFlywheelSpinning() {
        return launcher.isSpinning();
    }

    /**
     * Set flywheel power (does not start/stop, just sets power level)
     * @param power Power level (0.0 to 1.0)
     */
    public void setFlywheelPower(double power) {
        launcher.setPower(power);
    }

    /**
     * Get current flywheel power
     * @return Current flywheel power (0.0 to 1.0)
     */
    public double getFlywheelPower() {
        return launcher.getPower();
    }

    /**
     * Set hood position
     * @param position Hood position (0.0 to 1.0)
     */
    public void setHoodPosition(double position) {
        launcher.setHoodPosition(position);
    }

    /**
     * Get current hood position
     * @return Current hood position (0.0 to 1.0)
     */
    public double getHoodPosition() {
        return launcher.getHoodPosition();
    }

    /**
     * Adjust hood position by increment
     * @param increment Amount to adjust (positive = up, negative = down)
     */
    public void adjustHood(double increment) {
        launcher.adjustHoodPosition(increment);
    }

    // ==================== INTAKE DIRECT ACCESS ====================

    /**
     * Start intake at default power
     */
    public void startIntake() {
        intake.start();
        intake.update();
    }

    /**
     * Start intake at specified power
     * @param power Intake power (-1.0 to 1.0, positive = forward)
     */
    public void startIntake(double power) {
        intake.start(power);
        intake.update();
    }

    /**
     * Stop intake
     */
    public void stopIntake() {
        intake.stop();
        intake.update();
    }

    /**
     * Reverse intake (eject)
     */
    public void reverseIntake() {
        intake.reverse();
        intake.update();
    }

    /**
     * Check if intake is running
     * @return true if intake is running
     */
    public boolean isIntakeRunning() {
        return intake.isRunning();
    }

    /**
     * Get current intake power
     * @return Current intake power (-1.0 to 1.0)
     */
    public double getIntakePower() {
        return intake.getPower();
    }

    // ==================== SPINDEXER DIRECT ACCESS ====================

    /**
     * Check if spindexer is ready to shoot (all balls intaked)
     * @return true if ready to shoot
     */
    public boolean isReadyToShoot() {
        return spindexer.shouldStopFlywheel() == false; // Simplified check
    }

    /**
     * Check if previous X button state (for edge detection)
     * @return true if X was pressed in previous update
     */
    public boolean wasShootButtonPressed() {
        return spindexer.isPrevX();
    }

    /**
     * Manually trigger a shot
     */
    public void shoot() {
        spindexer.shootBall();
    }

    /**
     * Rotate spindexer one division
     */
    public void rotateSpindexer() {
        spindexer.rotateOneDivision();
    }

    // ==================== TURRET DIRECT ACCESS ====================

    /**
     * Check if turret is aligned with target
     * @return true if aligned (limelight connected and on target)
     */
    public boolean isTurretAligned() {
        // Turret update returns true if limelight connected
        // We'd need to add a method to Turret to check alignment state
        return turret != null; // Placeholder - would need Turret to expose alignment state
    }

    // ==================== LIFT/PARKING DIRECT ACCESS ====================

    /**
     * Move lift up (parking position)
     */
    public void parkUp() {
        lift.moveUp();
    }

    /**
     * Move lift down (unparked position)
     */
    public void parkDown() {
        lift.moveDown();
    }

    /**
     * Update parking with gamepad inputs
     * @param dpadUp D-Pad up button state
     * @param dpadDown D-Pad down button state
     */
    public void updateParking(boolean dpadUp, boolean dpadDown) {
        lift.update(dpadUp, dpadDown);
    }

    /**
     * Get current lift state
     * @return Lift state (IDLE, MOVING_UP, MOVING_DOWN, HOLDING)
     */
    public Lift.State getLiftState() {
        return lift.getState();
    }

    /**
     * Check if lift is at top position
     * @return true if lift is at or near top position
     */
    public boolean isLiftAtTop() {
        Lift.State state = lift.getState();
        return state == Lift.State.HOLDING && lift.getLeftPosition() > 10000; // Approximate top check
    }

    /**
     * Check if lift is at bottom position
     * @return true if lift is at or near bottom position
     */
    public boolean isLiftAtBottom() {
        Lift.State state = lift.getState();
        return state == Lift.State.HOLDING && lift.getLeftPosition() < 100; // Approximate bottom check
    }

    // ==================== DRIVETRAIN DIRECT ACCESS ====================

    /**
     * Stop all drive motors
     */
    public void stopDriving() {
        driveTrain.stopMotors();
    }

    // ==================== HIGH-LEVEL WORKFLOW METHODS ====================

    /**
     * Prepare robot for shooting sequence
     * Starts flywheel and ensures turret is aligned
     * @return true if ready to shoot
     */
    public boolean prepareToShoot() {
        startFlywheel();
        return updateTurret(); // Returns false if limelight not connected
    }

    /**
     * Complete shooting sequence: shoot and check if should stop flywheel
     * @return true if should stop flywheel (3 shots completed)
     */
    public boolean shootSequence() {
        spindexer.shootBall();
        return spindexer.shouldStopFlywheel();
    }

    /**
     * Stop all robot movement and subsystems
     */
    public void stopAll() {
        stopFlywheel();
        stopIntake();
        stopDriving();
    }

    // Component getters (for advanced access if needed)
    public Turret getTurret() {
        return turret;
    }

    public Launcher getLauncher() {
        return launcher;
    }

    public Spindexer getSpindexer() {
        return spindexer;
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public Lift getLift() {
        return lift;
    }

    public Intake getIntake() {
        return intake;
    }
}
