package org.firstinspires.ftc.teamcode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Poses;

import java.util.function.Supplier;

public abstract class LessSimpleAutonomous extends OpMode {
    private Follower follower;
    protected Alliance alliance;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private Poses.AlliancePoses poses;

    boolean running = false;

    protected LessSimpleAutonomous() {

    }

    public LessSimpleAutonomous(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        poses = Poses.forAlliance(alliance);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poses.get(Poses.NamedPose.STARTING_TOP_2));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, poses.get(Poses.NamedPose.LEAVE_TOP))))
                .setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(38)))
                .build();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!running) {
            follower.followPath(pathChain.get(), 0.5, true);
            running = true;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
    }
}