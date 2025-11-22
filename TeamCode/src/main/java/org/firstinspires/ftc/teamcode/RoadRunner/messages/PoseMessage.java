package org.firstinspires.ftc.teamcode.RoadRunner.messages;

import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Pose2d;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.getX();
        this.y = pose.getY();
        this.heading = pose.getRotation().getDegrees();
    }
}

