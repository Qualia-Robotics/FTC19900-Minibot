package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwarePushbot {
    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */
    /**
     * This is NOT an opmode.
     *
     * This class can be used to define all the specific hardware for a single robot.
     * In this case that robot is a Pushbot.
     * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
     *
     * This hardware class assumes the following device names have been configured on the robot:
     * Note:  All names are lower case and some have single spaces between words.
     */
// hello this is testing :)


        /* Public OpMode members. */
        public DcMotor  leftFrontDriveWheel   = null; //motor for left front wheel
        public DcMotor  leftBackDriveWheel   = null; //motor for left back wheel
        public DcMotor  rightFrontDriveWheel  = null; //motor for right front wheel
        public DcMotor  rightBackDriveWheel  = null; //motor for right back wheel

        public Servo launcherServo = null; 

        
        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
        public HardwarePushbot()
        {
        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) 
        {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            leftFrontDriveWheel  = hwMap.get(DcMotor.class, "leftFrontDriveWheel");
            leftBackDriveWheel  = hwMap.get(DcMotor.class, "leftBackDriveWheel");
            rightFrontDriveWheel = hwMap.get(DcMotor.class, "rightFrontDriveWheel");
            rightBackDriveWheel = hwMap.get(DcMotor.class, "rightBackDriveWheel");

            //left wheels are counterclockwise and because of 90 degree gearboxes, they all are forward
            leftFrontDriveWheel.setDirection(DcMotor.Direction.FORWARD);// Set to REVERSE if using AndyMark motors
            leftBackDriveWheel.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDriveWheel.setDirection(DcMotor.Direction.FORWARD);
            rightBackDriveWheel.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

            // Set all motors to zero power
            leftFrontDriveWheel.setPower(0);
            leftBackDriveWheel.setPower(0);
            rightFrontDriveWheel.setPower(0);
            rightBackDriveWheel.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftFrontDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

