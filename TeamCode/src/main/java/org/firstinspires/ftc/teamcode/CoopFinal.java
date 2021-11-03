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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="CoopFinal", group="Linear Opmode")
@Disabled
public class CoopFinal extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor intakeMotor = null;
    private DcMotor caroMotor = null;
    private DcMotor liftMotor = null;
    private Servo pushServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //control hub
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront"); // 0
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");// 1
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); // 2
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); // 3
        pushServo = hardwareMap.get(Servo.class, "pushServo"); // 0 (servo)

        //expansion hub
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor"); // 0
        caroMotor = hardwareMap.get(DcMotor.class, "caroMotor"); // 1
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor"); // 2
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //declaring extra variables
        int liftLevel = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            //intake
            if (gamepad2.right_trigger > 0) {
                intakeMotor.setPower(-0.6);
            } else if (gamepad2.left_trigger > 0) {
                intakeMotor.setPower(0.6);
            } else {
                intakeMotor.setPower(0.0);
            }

            //carousel
            if (gamepad2.right_bumper) {
                caroMotor.setPower(-1.0);
            } else if (gamepad2.left_bumper) {
                caroMotor.setPower(1.0);
            } else {
                caroMotor.setPower(0.0);
            }

            //lift
            if (liftMotor.isBusy() == false){
                if (gamepad1.dpad_up && liftLevel == 0) {
                    liftMotor.setPower(1.0);
                    liftMotor.setTargetPosition(3500);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLevel++;
                } else if (gamepad1.dpad_up && liftLevel == 1) {
                    liftMotor.setPower(1.0);
                    liftMotor.setTargetPosition(5250);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLevel++;
                } else if (gamepad1.dpad_up && liftLevel == 2) {
                    liftMotor.setPower(1.0);
                    liftMotor.setTargetPosition(6750);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLevel++;
                } else if (gamepad1.dpad_down && liftLevel == 3) {
                    liftMotor.setPower(-1.0);
                    liftMotor.setTargetPosition(5250);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLevel--;
                } else if (gamepad1.dpad_down && liftLevel == 2) {
                    liftMotor.setPower(-1.0);
                    liftMotor.setTargetPosition(3500);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLevel--;
                } else if (gamepad1.dpad_down && liftLevel == 1) {
                    liftMotor.setPower(-1.0);
                    liftMotor.setTargetPosition(0);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLevel--;
                }
            } //resets lift just incase
            if (gamepad1.ps) {
                liftMotor.setPower(-1.0);
                liftMotor.setTargetPosition(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftLevel = 0;
            }


            //push
            if (gamepad2.a) {
                pushServo.setPosition(0.5);
            } else {
                pushServo.setPosition(1.0);
            }

            // Send calculated power to wheels
            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0){
                leftFront.setPower(leftPower);
                leftBack.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightBack.setPower(rightPower);
            } else {
                leftFront.setPower(0.0);
                leftBack.setPower(0.0);
                rightFront.setPower(0.0);
                rightBack.setPower(0.0);
            }

            // Show the elapsed game time + extras INCORRECTLY :D
            telemetry.addData("Status", "\nRun Time: " + runtime.toString() + "\nLift Encoder : " + liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}