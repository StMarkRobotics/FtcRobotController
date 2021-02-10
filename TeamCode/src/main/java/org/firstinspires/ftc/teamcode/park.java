package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.sun.xml.internal.messaging.saaj.packaging.mime.util.BEncoderStream;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="park", group="Pushbot")
public class park extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareQuadbot robot   = new HardwareQuadbot();   //Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.4;
    static final double     FORWARD_SPEED_ENCODER = 0.7;
    static double     rearRight_pos = 1;
    int whiteLine = 4000;

    @Override
    public void runOpMode() {


        //Initialize the drive system variables. The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
        robot.RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1: Drive forward.
        robot.FrontLeft.setPower(FORWARD_SPEED);
        robot.FrontRight.setPower(FORWARD_SPEED);
        robot.RearLeft.setPower(FORWARD_SPEED);
        robot.RearRight.setPower(FORWARD_SPEED_ENCODER);
        runtime.reset();
        rearRight_pos =robot.RearRight.getCurrentPosition();
        while (opModeIsActive() && rearRight_pos < whiteLine) {
            rearRight_pos = robot.RearRight.getCurrentPosition();
            telemetry.addData("Path", "Part One: Moving forwards to get off wall.");
            //I don't know if I did this line right, I copied something from the sensor reading code.
            telemetry.addData("Encoder", String.format("%f", rearRight_pos));
            telemetry.update();
        }

        // Step 2:  Stop.
        robot.FrontLeft.setPower(0);
        robot.FrontRight.setPower(0);
        robot.RearLeft.setPower(0);
        robot.RearRight.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}

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
