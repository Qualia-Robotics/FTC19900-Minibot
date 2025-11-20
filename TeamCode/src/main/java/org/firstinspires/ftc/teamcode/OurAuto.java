package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class OurAuto extends LinearOpMode {
    private DcMotor flywheel;
    private DcMotor feedRoller;
    private CRServo agitator;

    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        agitator = hardwareMap.get(CRServo.class, "servo");

        waitForStart();
        while(opModeIsActive()) {
            flywheel.setPower(1);
            sleep(1000);
            agitator.setPower(1);
            sleep(500);
            feedRoller.setPower(1);
            sleep(6000);
            flywheel.setPower(0);
            agitator.setPower(0);
            feedRoller.setPower(0);
        }
    }
}
