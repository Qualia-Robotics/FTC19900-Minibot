package org.firstinspires.ftc.teamcode.OpModes.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Spindexer;

@Autonomous(name = "Autonomous - Ball Collection", group = "Autonomous")
public class SamarBlueRight extends OpMode {
    private Follower follower;
    private Robot robot;
    private Timer pathTimer, opmodeTimer, waitTimer, shootingTimer, colorDetectionTimer;

    private int pathState;
    private boolean isWaiting = false;
    private static final double WAIT_TIME_SECONDS = 1.5; // Wait 1.5 seconds after each path
    private static final double BALL_WAIT_TIME_SECONDS = 2.0; // Longer wait for ball collection paths
    
    // Shooting state management
    private boolean isShooting = false;
    private int shootingState = 0; // 0 = not shooting, 1-10 = rapidfire states, 11-30 = spindexing states
    private int rapidFireShotCount = 0;
    private int spindexingShotCount = 0;
    private boolean colorDetectedThisSegment = false;
    private static final double COLOR_DETECTION_DELAY = 1.0; // Wait 1 second after color detection before rotating
    private static final double KICKER_FLICK_DELAY = 0.4; // Delay for kicker flick
    private static final double KICKER_RESET_DELAY = 0.2; // Delay for kicker reset
    private static final double SERVO_MOVE_DELAY = 0.2; // Delay for servo movement
    private static final double DIVISION_ROTATE_DELAY = 0.15; // Delay for division rotation
    private static final double SHOOTING_DELAY = 0.8; // Delay between shooting actions
    
    // Spindexer hardware access
    private com.qualcomm.robotcore.hardware.Servo indexServo;
    private com.qualcomm.robotcore.hardware.Servo kickerServo;
    private com.qualcomm.robotcore.hardware.NormalizedColorSensor intakeColorSensor;
    private double targetDegrees = 0.0;
    private int currentDivision = 0;
    private java.util.Map<Integer, String> indexColors = new java.util.HashMap<>();
    private String[] need_colors = {"purple", "purple", "green"};
    private int flag = 0;

    // Starting position of the robot from trajectory (8).pp
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    // Path declarations - trajectory path segments (removing duplicates)
    private PathChain Path1; // align: (56,8) to (56,12)
    private PathChain Path2; // align with set 1: (56,12) to (40,39)
    private PathChain Path3; // get ball 1: (40,39) to (35,39)
    private PathChain Path4; // get ball 2: (35,39) to (29,39)
    private PathChain Path5; // get ball 3: (29,39) to (22,39)
    private PathChain Path6; // reset to shoot: (22,39) to (56,12)
    private PathChain Path7; // align with set 2: (56,12) to (40,65)
    private PathChain Path8; // get ball 1: (40,65) to (35,65)
    private PathChain Path9; // get ball 2: (35,65) to (29,65)
    private PathChain Path10; // get ball 3: (29,65) to (22,65)
    private PathChain Path11; // reset to shoot: (22,65) to (56,110)
    private PathChain Path12; // final path: (56,110) to (56,50)

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        waitTimer = new Timer();
        shootingTimer = new Timer();
        colorDetectionTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        robot = new Robot();
        robot.initialize(hardwareMap, telemetry, null); // OpMode is null since we're using OpMode, not LinearOpMode
        
        // Lock turret at 0.6
        robot.getTurret().setPositionDirect(0.6);
        
        // Initialize spindexer hardware directly for autonomous use
        initializeSpindexerHardware();
        
        buildPaths();
        follower.setStartingPose(startPose);
    }
    
    private void initializeSpindexerHardware() {
        try {
            Spindexer spindexer = robot.getSpindexer();
            if (spindexer != null) {
                java.lang.reflect.Field indexField = Spindexer.class.getDeclaredField("indexServo");
                indexField.setAccessible(true);
                indexServo = (com.qualcomm.robotcore.hardware.Servo) indexField.get(spindexer);
                
                java.lang.reflect.Field kickerField = Spindexer.class.getDeclaredField("kickerServo");
                kickerField.setAccessible(true);
                kickerServo = (com.qualcomm.robotcore.hardware.Servo) kickerField.get(spindexer);
                
                java.lang.reflect.Field colorField = Spindexer.class.getDeclaredField("intakeColorSensor");
                colorField.setAccessible(true);
                intakeColorSensor = (com.qualcomm.robotcore.hardware.NormalizedColorSensor) colorField.get(spindexer);
                
                java.lang.reflect.Field targetField = Spindexer.class.getDeclaredField("targetDegrees");
                targetField.setAccessible(true);
                targetDegrees = targetField.getDouble(spindexer);
                
                java.lang.reflect.Field divField = Spindexer.class.getDeclaredField("currentDivision");
                divField.setAccessible(true);
                currentDivision = divField.getInt(spindexer);
                
                java.lang.reflect.Field colorsField = Spindexer.class.getDeclaredField("indexColors");
                colorsField.setAccessible(true);
                indexColors = (java.util.Map<Integer, String>) colorsField.get(spindexer);
                
                // Initialize index colors if needed
                if (indexColors.isEmpty()) {
                    indexColors.put(0, "none");
                    indexColors.put(1, "none");
                    indexColors.put(2, "none");
                }
            }
        } catch (Exception e) {
            telemetry.addLine("Error initializing spindexer hardware: " + e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        // You can add initialization feedback here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        robot.updateIntake(); // Update intake to keep it running
        
        // Keep turret locked at 0.6
        robot.getTurret().setPositionDirect(0.6);
        
        // Handle color detection during intake segments
        handleColorDetection();
        
        // Handle shooting sequences
        handleShooting();
        
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("is waiting", isWaiting);
        telemetry.addData("is shooting", isShooting);
        telemetry.addData("shooting state", shootingState);
        telemetry.addData("is busy", follower.isBusy());
        telemetry.addData("intake running", robot.isIntakeRunning());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        if (pathState >= 2 && pathState <= 4) {
            telemetry.addLine("Getting balls - Path " + (pathState - 1));
        }
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        // Handle waiting state
        if (isWaiting) {
            // Use longer wait time for ball collection paths (states 2-4 and 7-9)
            double currentWaitTime = (pathState >= 2 && pathState <= 4) || (pathState >= 7 && pathState <= 9) 
                    ? BALL_WAIT_TIME_SECONDS : WAIT_TIME_SECONDS;
            
            if (waitTimer.getElapsedTimeSeconds() >= currentWaitTime) {
                isWaiting = false;
                waitTimer.resetTimer();
            } else {
                return; // Still waiting, don't process path states
            }
        }
        
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && !isShooting) {
                    // After path 1, shoot in rapidfire mode before moving to path 2
                    startRapidFireShooting();
                }
                break;
            case 2:
                if(!follower.isBusy() && !isShooting) {
                    robot.startIntake(); // Start intake before aligning with set 1
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path2);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && !isShooting) {
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path3); // Get ball 1
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() && !isShooting) {
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path4); // Get ball 2
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && !isShooting) {
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path5); // Get ball 3
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() && !isShooting) {
                    robot.stopIntake(); // Stop intake after getting all balls from set 1
                    startWait();
                    follower.followPath(Path6);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() && !isShooting) {
                    // After path 6, shoot in spindexing mode before moving to path 7
                    startSpindexingShooting();
                }
                break;
            case 8:
                if(!follower.isBusy() && !isShooting) {
                    robot.startIntake(); // Start intake before aligning with set 2
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path7);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy() && !isShooting) {
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path8);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy() && !isShooting) {
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path9);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() && !isShooting) {
                    colorDetectedThisSegment = false;
                    startWait();
                    follower.followPath(Path10);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy() && !isShooting) {
                    robot.stopIntake(); // Stop intake after getting all balls from set 2
                    startWait();
                    follower.followPath(Path11);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy() && !isShooting) {
                    // After path 11, shoot in spindexing mode before moving to path 12
                    startSpindexingShooting();
                }
                break;
            case 14:
                if(!follower.isBusy() && !isShooting) {
                    startWait();
                    follower.followPath(Path12);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    // Set the state to a Case we won't use or define, so it just stops running any new paths
                    setPathState(-1);
                }
                break;
        }
    }
    
    private void startWait() {
        isWaiting = true;
        waitTimer.resetTimer();
    }
    
    private void startWait(double waitTime) {
        isWaiting = true;
        waitTimer.resetTimer();
        // Store custom wait time - we'll check against WAIT_TIME_SECONDS for now
        // For ball collection, we'll use longer waits
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        // Path 1: align - (56,8) to (56,12), linear heading 90→115
        Path1 = follower.pathBuilder()
                .setGlobalDeceleration(2.0) // Higher deceleration slows paths down significantly
                .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        // Path 2: align with set 1 - (56,12) to (40,39), linear heading 115→180
        Path2 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 39, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        // Path 3: get ball 1 - (40,39) to (35,39), tangential heading - SLOWER for intake
        Path3 = follower.pathBuilder()
                .setGlobalDeceleration(3.0) // Slower deceleration
                .setMaxVelocity(0.6) // Reduced max velocity for intake segments
                .addPath(new BezierLine(new Pose(40, 39, Math.toRadians(180)), new Pose(35, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 4: get ball 2 - (35,39) to (29,39), tangential heading - SLOWER for intake
        Path4 = follower.pathBuilder()
                .setGlobalDeceleration(3.0) // Slower deceleration
                .setMaxVelocity(0.6) // Reduced max velocity for intake segments
                .addPath(new BezierLine(new Pose(35, 39, Math.toRadians(180)), new Pose(29, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5: get ball 3 - (29,39) to (22,39), tangential heading - SLOWER for intake
        Path5 = follower.pathBuilder()
                .setGlobalDeceleration(3.0) // Slower deceleration
                .setMaxVelocity(0.6) // Reduced max velocity for intake segments
                .addPath(new BezierLine(new Pose(29, 39, Math.toRadians(180)), new Pose(22, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 6: reset to shoot - (22,39) to (56,12), linear heading 180→115
        Path6 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 39, Math.toRadians(180)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        // Path 7: align with set 2 - (56,12) to (40,65), linear heading 115→180
        Path7 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 65, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        // Path 8: get ball 1 - (40,65) to (35,65), tangential heading - SLOWER for intake
        Path8 = follower.pathBuilder()
                .setGlobalDeceleration(3.0) // Slower deceleration
                .setMaxVelocity(0.6) // Reduced max velocity for intake segments
                .addPath(new BezierLine(new Pose(40, 65, Math.toRadians(180)), new Pose(35, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 9: get ball 2 - (35,65) to (29,65), tangential heading - SLOWER for intake
        Path9 = follower.pathBuilder()
                .setGlobalDeceleration(3.0) // Slower deceleration
                .setMaxVelocity(0.6) // Reduced max velocity for intake segments
                .addPath(new BezierLine(new Pose(35, 65, Math.toRadians(180)), new Pose(29, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 10: get ball 3 - (29,65) to (22,65), tangential heading - SLOWER for intake
        Path10 = follower.pathBuilder()
                .setGlobalDeceleration(3.0) // Slower deceleration
                .setMaxVelocity(0.6) // Reduced max velocity for intake segments
                .addPath(new BezierLine(new Pose(29, 65, Math.toRadians(180)), new Pose(22, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 11: reset to shoot - (22,65) to (56,110), linear heading 180→150
        Path11 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 65, Math.toRadians(180)), new Pose(56, 110, Math.toRadians(150))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();

        // Path 12: final path - (56,110) to (56,50), linear heading 150→150
        Path12 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 110, Math.toRadians(150)), new Pose(56, 50, Math.toRadians(150))))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                .build();
    }

    // ==================== COLOR DETECTION ====================
    
    private void handleColorDetection() {
        // Only detect color during intake segments (paths 3-6 and 9-12)
        boolean isIntakeSegment = (pathState >= 3 && pathState <= 6) || (pathState >= 9 && pathState <= 12);
        
        if (isIntakeSegment && robot.isIntakeRunning() && !colorDetectedThisSegment && intakeColorSensor != null) {
            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            float hue = JavaUtil.colorToHue(colors.toColor());
            
            boolean found = false;
            String detectedColor = "unknown";
            if (hue > 160 && hue < 350) { // purple
                detectedColor = "purple";
                found = true;
            } else if (hue >= 100 && hue <= 160) { // green
                detectedColor = "green";
                found = true;
            }
            
            if (found && !colorDetectedThisSegment) {
                indexColors.put(currentDivision, detectedColor);
                colorDetectedThisSegment = true;
                colorDetectionTimer.resetTimer();
                telemetry.addLine("Ball detected: " + detectedColor + " at division " + currentDivision);
            }
        }
        
        // Rotate division after delay when ball detected
        if (colorDetectedThisSegment && colorDetectionTimer.getElapsedTimeSeconds() >= COLOR_DETECTION_DELAY) {
            rotateOneDivision();
            colorDetectedThisSegment = false; // Reset for next segment
            telemetry.addLine("Rotated division after ball detection");
        }
    }
    
    private void rotateOneDivision() {
        currentDivision = (currentDivision + 1) % 3;
        targetDegrees = clipDeg(targetDegrees - Spindexer.DIVISION_DEGREES);
        smoothMoveTo(targetDegrees);
    }
    
    private double clipDeg(double d) {
        return com.qualcomm.robotcore.util.Range.clip(d, 0.0, Spindexer.MAX_DEGREES);
    }
    
    private void smoothMoveTo(double newTargetDeg) {
        if (indexServo == null) return;
        double startPos = indexServo.getPosition();
        double startDeg = startPos * Spindexer.MAX_DEGREES;
        double distance = newTargetDeg - startDeg;
        int steps = (int) Math.ceil(Math.abs(distance) / Spindexer.SPEED_DEG_PER_STEP);
        
        // Set position directly (non-blocking)
        indexServo.setPosition(Spindexer.posFromDeg(newTargetDeg));
    }
    
    // ==================== SHOOTING SEQUENCES ====================
    
    private void startRapidFireShooting() {
        isShooting = true;
        shootingState = 1; // State 1: First shot - prepare
        rapidFireShotCount = 0;
        shootingTimer.resetTimer();
        robot.startFlywheel(); // Start flywheel
        telemetry.addLine("Starting rapidfire shooting sequence");
    }
    
    private void startSpindexingShooting() {
        isShooting = true;
        shootingState = 11; // State 11: First spindexing shot - prepare
        spindexingShotCount = 0;
        shootingTimer.resetTimer();
        robot.startFlywheel(); // Start flywheel
        telemetry.addLine("Starting spindexing shooting sequence");
    }
    
    private void handleShooting() {
        if (!isShooting || kickerServo == null) return;
        
        // Rapidfire mode (after path 1): shoot, rotate, shoot, rotate, shoot
        if (shootingState >= 1 && shootingState <= 10) {
            handleRapidFireShooting();
        }
        // Spindexing mode (after paths 6 and 11): shoot all 3 balls in order
        else if (shootingState >= 11 && shootingState <= 30) {
            handleSpindexingShooting();
        }
    }
    
    private void handleRapidFireShooting() {
        double elapsed = shootingTimer.getElapsedTimeSeconds();
        
        switch (shootingState) {
            case 1: // First shot - perform kicker sequence
                kickerServo.setPosition(Spindexer.KICKER_FLICK_POSITION);
                shootingTimer.resetTimer();
                shootingState = 2;
                break;
            case 2: // Wait for kicker flick
                if (elapsed >= KICKER_FLICK_DELAY) {
                    kickerServo.setPosition(Spindexer.KICKER_RESET_POSITION);
                    shootingTimer.resetTimer();
                    shootingState = 3;
                }
                break;
            case 3: // Wait for kicker reset, then move division
                if (elapsed >= KICKER_RESET_DELAY) {
                    rotateOneDivision();
                    shootingTimer.resetTimer();
                    shootingState = 4;
                }
                break;
            case 4: // Second shot - wait for division rotation, then shoot
                if (elapsed >= DIVISION_ROTATE_DELAY) {
                    kickerServo.setPosition(Spindexer.KICKER_FLICK_POSITION);
                    shootingTimer.resetTimer();
                    shootingState = 5;
                }
                break;
            case 5: // Wait for kicker flick
                if (elapsed >= KICKER_FLICK_DELAY) {
                    kickerServo.setPosition(Spindexer.KICKER_RESET_POSITION);
                    shootingTimer.resetTimer();
                    shootingState = 6;
                }
                break;
            case 6: // Wait for kicker reset, then move division
                if (elapsed >= KICKER_RESET_DELAY) {
                    rotateOneDivision();
                    shootingTimer.resetTimer();
                    shootingState = 7;
                }
                break;
            case 7: // Third shot - wait for division rotation, then shoot
                if (elapsed >= DIVISION_ROTATE_DELAY) {
                    kickerServo.setPosition(Spindexer.KICKER_FLICK_POSITION);
                    shootingTimer.resetTimer();
                    shootingState = 8;
                }
                break;
            case 8: // Wait for kicker flick
                if (elapsed >= KICKER_FLICK_DELAY) {
                    kickerServo.setPosition(Spindexer.KICKER_RESET_POSITION);
                    shootingTimer.resetTimer();
                    shootingState = 9;
                }
                break;
            case 9: // Wait for kicker reset, then finish
                if (elapsed >= KICKER_RESET_DELAY) {
                    finishShooting();
                    setPathState(2); // Move to path 2
                }
                break;
        }
    }
    
    private void handleSpindexingShooting() {
        double elapsed = shootingTimer.getElapsedTimeSeconds();
        
        // States 11-20: First ball
        // States 21-30: Second ball  
        // States 31-40: Third ball
        // State 41: Finish
        
        if (shootingState >= 11 && shootingState <= 20) {
            // First ball shooting sequence
            handleSpindexingBallShot(0, 11, 21);
        } else if (shootingState >= 21 && shootingState <= 30) {
            // Second ball shooting sequence
            handleSpindexingBallShot(1, 21, 31);
        } else if (shootingState >= 31 && shootingState <= 40) {
            // Third ball shooting sequence
            handleSpindexingBallShot(2, 31, 41);
        } else if (shootingState == 41) {
            // Finished all 3 balls
            finishShooting();
            if (pathState == 7) {
                setPathState(8); // Move to path 7 after shooting at path 6
            } else if (pathState == 13) {
                setPathState(14); // Move to path 12 after shooting at path 11
            }
        }
    }
    
    private void handleSpindexingBallShot(int ballIndex, int startState, int nextState) {
        double elapsed = shootingTimer.getElapsedTimeSeconds();
        int localState = shootingState - startState;
        
        switch (localState) {
            case 0: // Find division with needed color and move to it
                String neededColor = need_colors[flag % need_colors.length];
                int goalDiv = findDivisionWithColor(neededColor);
                if (goalDiv > 2) goalDiv -= 3;
                moveToDivision(goalDiv);
                shootingTimer.resetTimer();
                shootingState = startState + 1;
                break;
            case 1: // Wait for division movement, then rotate one division
                if (elapsed >= DIVISION_ROTATE_DELAY * 3) { // Allow time for multiple divisions
                    rotateOneDivision();
                    shootingTimer.resetTimer();
                    shootingState = startState + 2;
                }
                break;
            case 2: // Adjust for shooting
                targetDegrees = clipDeg(targetDegrees - Spindexer.SHOOTING_DEGREE_ADJUSTMENT);
                smoothMoveTo(targetDegrees);
                shootingTimer.resetTimer();
                shootingState = startState + 3;
                break;
            case 3: // Wait for servo movement, then flick kicker
                if (elapsed >= SERVO_MOVE_DELAY) {
                    kickerServo.setPosition(Spindexer.KICKER_FLICK_POSITION);
                    shootingTimer.resetTimer();
                    shootingState = startState + 4;
                }
                break;
            case 4: // Wait for kicker flick
                if (elapsed >= KICKER_FLICK_DELAY) {
                    kickerServo.setPosition(Spindexer.KICKER_RESET_POSITION);
                    shootingTimer.resetTimer();
                    shootingState = startState + 5;
                }
                break;
            case 5: // Wait for kicker reset, then adjust back
                if (elapsed >= KICKER_RESET_DELAY) {
                    targetDegrees = clipDeg(targetDegrees + Spindexer.SHOOTING_DEGREE_ADJUSTMENT);
                    smoothMoveTo(targetDegrees);
                    indexColors.put(flag, "none");
                    flag++;
                    shootingTimer.resetTimer();
                    shootingState = nextState; // Move to next ball
                }
                break;
        }
    }
    
    private int findDivisionWithColor(String c) {
        for (int i = 0; i < 3; i++) {
            if (indexColors.get(i) != null && indexColors.get(i).equals(c)) {
                return i;
            }
        }
        return currentDivision;
    }
    
    private void moveToDivision(int targetDiv) {
        int steps = (targetDiv - currentDivision + 3) % 3;
        for (int i = 0; i < steps; i++) {
            rotateOneDivision();
        }
    }
    
    private void finishShooting() {
        int previousShootingState = shootingState;
        isShooting = false;
        shootingState = 0;
        rapidFireShotCount = 0;
        spindexingShotCount = 0;
        robot.stopFlywheel();
        
        // Reset spindexer state if needed (was spindexing mode)
        if (previousShootingState >= 11) {
            indexColors.put(0, "none");
            indexColors.put(1, "none");
            indexColors.put(2, "none");
            targetDegrees = Spindexer.MAX_DEGREES - Spindexer.INITIAL_POSITION_OFFSET;
            smoothMoveTo(targetDegrees);
            currentDivision = 0;
            flag = 0;
        }
        
        telemetry.addLine("Shooting sequence complete");
    }

    @Override
    public void stop() {}
}