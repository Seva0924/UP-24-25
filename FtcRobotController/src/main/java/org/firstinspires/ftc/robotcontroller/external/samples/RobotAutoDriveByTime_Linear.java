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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
//@Disabled
public class RobotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private DcMotor vertSlide = null;
//    private Servo funnel = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double SLIDE_SPEED = 0.95;
    //    double     funnelPos = 0.5;
    double leftArmServoPos = 0.5;
    double rightArmServoPos = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        vertSlide = hardwareMap.get(DcMotor.class, "vertSlide");
//        funnel = hardwareMap.get(Servo.class, "funnel");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        vertSlide.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        vertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

////         Step 1:  Drive forward for 3 seconds
//        leftFrontDrive.setPower(FORWARD_SPEED);
//        leftBackDrive.setPower(FORWARD_SPEED);
//        rightBackDrive.setPower(FORWARD_SPEED);
//        rightFrontDrive.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.8)) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 2:  Spin right for 1.3 seconds
//        leftFrontDrive.setPower(TURN_SPEED);
//        leftBackDrive.setPower(TURN_SPEED);
//        rightFrontDrive.setPower(-TURN_SPEED);
//        rightBackDrive.setPower(-TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
////        // Step 3:  Drive Backward for 2 Seconds
//        leftFrontDrive.setPower(-TURN_SPEED);
//        leftBackDrive.setPower(-TURN_SPEED);
//        rightFrontDrive.setPower(-TURN_SPEED);
//        rightBackDrive.setPower(-TURN_SPEED);
//        telemetry.addData("Backwards Successful", getRuntime());
//        telemetry.update();
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            leftFrontDrive.setPower(-TURN_SPEED);
            leftBackDrive.setPower(-TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);
            telemetry.addData("Backwards Successful", getRuntime());
            telemetry.update();
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

            leftArmServoPos = leftArmServoPos - 0.13;//position= 0.37
            leftArmServo.setPosition(leftArmServoPos);
            rightArmServoPos = rightArmServoPos + 0.13;//position= 0.63
            rightArmServo.setPosition(rightArmServoPos);

            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }




//// vert slides up

//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            vertSlide.setPower(SLIDE_SPEED);
//            telemetry.addData("Vertical Slides Successful", getRuntime());
//            telemetry.update();
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        vertSlide.setPower(SLIDE_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        funnelPos = funnelPos + 0.3;
//        funnel.setPosition(funnelPos);

        // Step 4:  Stop
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        vertSlide.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}

