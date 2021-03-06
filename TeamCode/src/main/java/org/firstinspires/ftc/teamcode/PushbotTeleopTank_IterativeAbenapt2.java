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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Code", group="Pushbot")
//@Disabled
public class PushbotTeleopTank_IterativeAbenapt2 extends OpMode{

    /* Declare OpMode members. */
    HardwareQuadbot robot       = new HardwareQuadbot(); // use the class created to define a Pushbot's hardware
    double          ClawOffset  = 0.0 ;                  // Servo mid position
    final double    Claw_Speed  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double strafe;
        double arm_pos;
        boolean arm_up;
        boolean arm_down;
        boolean treadmill;
        boolean treadmill_50;
        boolean disc_on;
        boolean disc_off;
        boolean disc_90;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        treadmill = gamepad1.dpad_down;
        treadmill_50 = gamepad1.dpad_right;
        disc_on = gamepad1.a;
        disc_off= gamepad1.b;
        disc_90 = gamepad1.x;
        arm_up = gamepad1.left_bumper;
        arm_down = gamepad1.right_bumper;


        robot.FrontRight.setPower(right);
        robot.FrontLeft.setPower(left);
        robot.RearRight.setPower(right);
        robot.RearLeft.setPower(left);

        // Strafe
        if (strafe !=0) {
            robot.FrontRight.setPower(-strafe);
            robot.FrontLeft.setPower(strafe);
            robot.RearRight.setPower(strafe);
            robot.RearLeft.setPower(-strafe);}

        // Disc (on)
        if (disc_on) {
            robot.DiscShooter.setPower(1);
        }

        // Disc (off)
        if (disc_off) {
            robot.DiscShooter.setPower(0);
        }

        // Disc (90%)
        if (disc_90) {
            robot.DiscShooter.setPower(0.9);
        }

        // Treadmill
        if (treadmill) {
            robot.Tread.setPower(1);
        }
        else if (treadmill_50){
            robot.Tread.setPower(0.5);
        }
        else {
            robot.Tread.setPower(0); }

        // Arm Position
        arm_pos = robot.arm_motor.getCurrentPosition();
        if (arm_up & arm_pos< 6000) {
            robot.arm_motor.setPower(0.5);
        }
        else if (arm_down & arm_pos > 0) {
            robot.arm_motor.setPower(-0.5);
        }
        else {
            robot.arm_motor.setPower(0);
        }

        // CLAW
        if (gamepad1.right_trigger != 0)
            ClawOffset += Claw_Speed;
        else if (gamepad1.left_trigger != 0)
            ClawOffset -= Claw_Speed;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        ClawOffset = Range.clip(ClawOffset, -0.5, 0.5);
        robot.claw_servo.setPosition(robot.MID_SERVO + ClawOffset);


        /* Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            robot.leftArm.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
        else
            robot.leftArm.setPower(0.0);
        */

        // Send telemetry message to signify robot running;
        telemetry.addData("strafe",  "Offset = %.2f", strafe);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("claw", "%.2f", ClawOffset);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
