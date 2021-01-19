package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")

/**
 * This file uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous_LeftBlue", group="Pushbot")
//@Disabled
public class Autonomous_LeftBlue extends LinearOpMode {
    private DistanceSensor sensorRange;
    // Declare OpMode members.
    HardwareQuadbot robot   = new HardwareQuadbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.45;
    static final double     TURN_SPEED    = 0.45;

    @Override
    public void runOpMode() {
        sensorRange = hardwareMap.get(Rev2mDistanceSensor .class, "sensor_range");

        //Initialize the drive system variables. The init() method of the hardware class does all the work here.
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Step 1: Drive forward to get off the line and move wobble thing.
        robot.FrontLeft.setPower(FORWARD_SPEED);
        robot.FrontRight.setPower(FORWARD_SPEED);
        robot.RearLeft.setPower(FORWARD_SPEED);
        robot.RearRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
            telemetry.addData("Path", "Part One: Moving forwards to get off wall.", runtime.seconds());
            telemetry.update();
        }

        // Step 2: Strafe right to check starter stack, temporarily leaving wobble thing.
        robot.FrontLeft.setPower(FORWARD_SPEED);
        robot.FrontRight.setPower(-FORWARD_SPEED);
        robot.RearLeft.setPower(-FORWARD_SPEED);
        robot.RearRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
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
        robot.FrontLeft.setPower(-FORWARD_SPEED);
        robot.FrontRight.setPower(FORWARD_SPEED);
        robot.RearLeft.setPower(FORWARD_SPEED);
        robot.RearRight.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
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
            while (opModeIsActive() && (runtime.seconds() < .4)) {
                telemetry.addData("Path", "Part Five: Starter Stack 0- Turning towards target zone A. ", runtime.seconds());
                telemetry.update();
            }
            // Step 6: Push wobble thing to target zone A.
            robot.FrontLeft.setPower(FORWARD_SPEED);
            robot.FrontRight.setPower(FORWARD_SPEED);
            robot.RearLeft.setPower(FORWARD_SPEED);
            robot.RearRight.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2)) {
                telemetry.addData("Path", "Part Six: Starter Stack 0- Pushing wobble thing to target zone A.", runtime.seconds());
                telemetry.update();
            }
        }
         else if (stack >= 4.4 ) {
            // Step 5: Turn towards target zone B.
            robot.FrontLeft.setPower(-TURN_SPEED);
            robot.FrontRight.setPower(TURN_SPEED);
            robot.RearLeft.setPower(-TURN_SPEED);
            robot.RearRight.setPower(TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .4)) {
                telemetry.addData("Path", "Part Six: Starter Stack 2- Turning towards target zone B. ", runtime.seconds());
                telemetry.update();
            }
            // Step 6: Push wobble thing to target zone B.
            robot.FrontLeft.setPower(FORWARD_SPEED);
            robot.FrontRight.setPower(FORWARD_SPEED);
            robot.RearLeft.setPower(FORWARD_SPEED);
            robot.RearRight.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) {
                telemetry.addData("Path", "Part Six: Starter Stack 2- Pushing wobble thing to target zone B.", runtime.seconds());
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
