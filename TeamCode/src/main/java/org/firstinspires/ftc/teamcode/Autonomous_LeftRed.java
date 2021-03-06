package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")

/**
 * This file uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="don't_use", group="Pushbot")

public class Autonomous_LeftRed extends LinearOpMode {
    private DistanceSensor sensorRange;
    // Declare OpMode members.
    HardwareQuadbot robot   = new HardwareQuadbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.45;
    static final double     TURN_SPEED    = 0.45;
    static double     rearRight_pos = 1;
    static double arm_pos= 1.5;


    @Override
    public void runOpMode() {

        //Initialize the drive system variables. The init() method of the hardware class does all the work here.
        robot.init(hardwareMap);
        robot.RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sensorRange = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_range");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        rearRight_pos = robot.RearRight.getCurrentPosition();

        // Step 1: Drive forward to get off the line and move wobble thing.
        robot.FrontLeft.setPower(FORWARD_SPEED);
        robot.FrontRight.setPower(FORWARD_SPEED);
        robot.RearLeft.setPower(FORWARD_SPEED);
        robot.RearRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (rearRight_pos < 3300)) {
            rearRight_pos = robot.RearRight.getCurrentPosition();
            telemetry.addData("Encoder", String.format("%f", rearRight_pos));
            telemetry.addData("Path", "Part One: Moving forwards to get off wall.", runtime.seconds());
            telemetry.update();
        }

        //Stop encoder
        robot.RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Step 2: Strafe right to check starter stack, temporarily leaving wobble thing.
        robot.FrontLeft.setPower(FORWARD_SPEED*1.5);
        robot.FrontRight.setPower(-FORWARD_SPEED);
        robot.RearLeft.setPower(-FORWARD_SPEED);
        robot.RearRight.setPower(FORWARD_SPEED*1.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Part Two: Strafing to starter stack.", runtime.seconds());
            telemetry.update();
        }

        // Step 3: Check starter stack for how many rings there are, then store the distance in a variable
        robot.FrontLeft.setPower(0);
        robot.FrontRight.setPower(0);
        robot.RearLeft.setPower(0);
        robot.RearRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("deviceName", sensorRange.getDeviceName());
            telemetry.addData("Path", "Part Three: Checking starter stack height.", runtime.seconds());
            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }

        //Creates variable to store distance
        float stack = (float) sensorRange.getDistance(DistanceUnit.INCH);

        // Step 4: Strafe back to the wobble thing.
        robot.FrontLeft.setPower(-FORWARD_SPEED*1.5);
        robot.FrontRight.setPower(FORWARD_SPEED);
        robot.RearLeft.setPower(FORWARD_SPEED);
        robot.RearRight.setPower(-FORWARD_SPEED*1.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Part Four: Strafing back to wobble thing. ", runtime.seconds());
            telemetry.update();
        }

        //Depending on what the height of the starter stack was, this will do different things
        if (stack >=5.8) {
            // Step 5: Turn towards target zone A.
            robot.FrontLeft.setPower(TURN_SPEED);
            robot.FrontRight.setPower(-TURN_SPEED);
            robot.RearLeft.setPower(TURN_SPEED);
            robot.RearRight.setPower(-TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .9)) {
                telemetry.addData("Path", "Part Five: Starter Stack 0- Turning towards target zone A. ", runtime.seconds());
                telemetry.update();
            }
            // Step 6: Push wobble thing to target zone A.
            robot.FrontLeft.setPower(FORWARD_SPEED);
            robot.FrontRight.setPower(FORWARD_SPEED);
            robot.RearLeft.setPower(FORWARD_SPEED);
            robot.RearRight.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.85)) {
                telemetry.addData("Path", "Part Six: Starter Stack 0- Pushing wobble thing to target zone A.", runtime.seconds());
                telemetry.update();
            }

            //Step 6b: Drop of the wobble thing.
            robot.FrontLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.RearLeft.setPower(0);
            robot.RearRight.setPower(0);
            robot.arm_motor.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (arm_pos < 5500)) {
                arm_pos = robot.arm_motor.getCurrentPosition();
                telemetry.addData("Path", "dropping off wobble thing ", runtime.seconds());
                telemetry.update();
            }
            robot.arm_motor.setPower(0);
            robot.FrontLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.RearLeft.setPower(0);
            robot.RearRight.setPower(0);
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("waiting","waiting");
                telemetry.update();
            }
            robot.claw_servo.setPosition(0.5);
            robot.arm_motor.setPower(-FORWARD_SPEED);
            runtime.reset();
            arm_pos = robot.arm_motor.getCurrentPosition();
            while (opModeIsActive() && (arm_pos > 100)) {
                arm_pos = robot.arm_motor.getCurrentPosition();
                telemetry.addData("Path", "dropping off wobble thing", runtime.seconds());
                telemetry.update();
            }
            robot.arm_motor.setPower(0);
        }
         else if (stack >= 4.4 ) {
            // Step 5: Turn towards target zone B.
            robot.FrontLeft.setPower(TURN_SPEED);
            robot.FrontRight.setPower(-TURN_SPEED);
            robot.RearLeft.setPower(TURN_SPEED);
            robot.RearRight.setPower(-TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .35)) {
                telemetry.addData("Path", "Part Six: Starter Stack 1- Turning towards target zone B. ", runtime.seconds());
                telemetry.update();
            }
            // Step 6: Push wobble thing to target zone B.
            robot.FrontLeft.setPower(FORWARD_SPEED);
            robot.FrontRight.setPower(FORWARD_SPEED);
            robot.RearLeft.setPower(FORWARD_SPEED);
            robot.RearRight.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.35)) {
                telemetry.addData("Path", "Part Six: Starter Stack 1- Pushing wobble thing to target zone B.", runtime.seconds());
                telemetry.update();
            }
            //Step 6b: Drop of the wobble thing.
            robot.FrontLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.RearLeft.setPower(0);
            robot.RearRight.setPower(0);
            robot.arm_motor.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (arm_pos < 5500)) {
                arm_pos = robot.arm_motor.getCurrentPosition();
                telemetry.addData("Path", "dropping off wobble thing ", runtime.seconds());
                telemetry.update();
            }
            robot.arm_motor.setPower(0);
            robot.FrontLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.RearLeft.setPower(0);
            robot.RearRight.setPower(0);
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("waiting","waiting");
                telemetry.update();
            }
            robot.claw_servo.setPosition(0.5);
            robot.arm_motor.setPower(-FORWARD_SPEED);
            runtime.reset();
            arm_pos = robot.arm_motor.getCurrentPosition();
            while (opModeIsActive() && (arm_pos > 100)) {
                arm_pos = robot.arm_motor.getCurrentPosition();
                telemetry.addData("Path", "dropping off wobble thing", runtime.seconds());
                telemetry.update();
            }
            robot.arm_motor.setPower(0);

            //Step Seven: Park on line.
            robot.FrontLeft.setPower(-FORWARD_SPEED);
            robot.FrontRight.setPower(-FORWARD_SPEED);
            robot.RearLeft.setPower(-FORWARD_SPEED);
            robot.RearRight.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .6)) {
                telemetry.addData("Path", "Part Seven: Parking on launch line. ", runtime.seconds());
                telemetry.update();
            }

        }
         else {
            // Step 5: Turn towards target zone C.
            robot.FrontLeft.setPower(TURN_SPEED);
            robot.FrontRight.setPower(-TURN_SPEED);
            robot.RearLeft.setPower(TURN_SPEED);
            robot.RearRight.setPower(-TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .4)) {
                telemetry.addData("Path", "Part Five: Starter Stack 4- Turning towards target zone C. ", runtime.seconds());
                telemetry.update();
            }
            // Step 6: Push wobble thing to target zone C.
            robot.FrontLeft.setPower(FORWARD_SPEED);
            robot.FrontRight.setPower(FORWARD_SPEED);
            robot.RearLeft.setPower(FORWARD_SPEED);
            robot.RearRight.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4)) {
                telemetry.addData("Path", "Part Six: Starter Stack 4- Pushing wobble thing to target zone C.", runtime.seconds());
                telemetry.update();
            }
            //Step 6b: Drop of the wobble thing.
            robot.FrontLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.RearLeft.setPower(0);
            robot.RearRight.setPower(0);
            robot.arm_motor.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (arm_pos < 5500)) {
                arm_pos = robot.arm_motor.getCurrentPosition();
                telemetry.addData("Path", "dropping off wobble thing ", runtime.seconds());
                telemetry.update();
            }
            robot.arm_motor.setPower(0);
            robot.FrontLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.RearLeft.setPower(0);
            robot.RearRight.setPower(0);
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("waiting","waiting");
                telemetry.update();
            }
            robot.claw_servo.setPosition(0.5);
            robot.arm_motor.setPower(-FORWARD_SPEED);
            runtime.reset();
            arm_pos = robot.arm_motor.getCurrentPosition();
            while (opModeIsActive() && (arm_pos > 100)) {
                arm_pos = robot.arm_motor.getCurrentPosition();
                telemetry.addData("Path", "dropping off wobble thing", runtime.seconds());
                telemetry.update();
            }
            robot.arm_motor.setPower(0);

            //Park on launch line.
            robot.FrontLeft.setPower(-FORWARD_SPEED);
            robot.FrontRight.setPower(-FORWARD_SPEED);
            robot.RearLeft.setPower(-FORWARD_SPEED);
            robot.RearRight.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.5)) {
                telemetry.addData("Path", "Part Seven: Parking on launch line.", runtime.seconds());
                telemetry.update();
            }
        }

        // ...Last Step:  Stop.
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
