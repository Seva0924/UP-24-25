package org.firstinspires.ftc.teamcode;/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")

public class BasicOmniOpMode_Linear_Testing extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo claw = null;
    private Servo wrist = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private DcMotor horizontalSlide = null;
    private DcMotor vertSlide = null;
    private Servo funnel = null;
    private DcMotor pullUp = null;
    private DcMotor extension = null;
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
        vertSlide = hardwareMap.get(DcMotor.class, "vertSlide");
        funnel = hardwareMap.get(Servo.class, "funnel");
        pullUp = hardwareMap.get(DcMotor.class, "pullUp");
        extension = hardwareMap.get(DcMotor.class, "extension");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        vertSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
//
//            // Combine the joystick requests for each axis-motion to determine each wheel's power.
//            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
//            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels\
            double clawPos = 0.5;
            double wristPos = 0.5;
            double leftArmServoPos = 0.5;
            double rightArmServoPos = 0.5;
            double funnelPos = 0.5;
//CLAW CODE
            if (gamepad2.y) { //Open Claw
                clawPos = clawPos - 0.15;
                claw.setPosition(clawPos);
            }
            if (gamepad2.x) { //Close Claw
                clawPos = clawPos + 0.2;
                claw.setPosition(clawPos);
            }
            if (gamepad2.right_stick_x < 0) { // when joystick pushed left Wrist Moves to Out position
                wristPos = wristPos - 0.225;
                wrist.setPosition(wristPos);
            }
            if (gamepad2.right_stick_x > 0) { //when joystick pushed right Wrist Moves flips into the robot
                wristPos = wristPos + 0.4;
                wrist.setPosition(wristPos);
            }

//PULL UP CODE
            if (gamepad2.left_stick_x > 0) { // when joystick pushed right pull up
                pullUp.setPower(.95);
            }
            if (gamepad2.right_stick_x < 0) { //when joystick pushed up Wrist Moves ___?
                pullUp.setPower(-.95);
            } else {
                pullUp.setPower(0);

                if (gamepad2.left_stick_y > 0) { // when joystick pushed right pull up
                    extension.setPower(-.95);
                }
                if (gamepad2.right_stick_y < 0) { //when joystick pushed up Wrist Moves ___?
                    extension.setPower(.95);
                } else {
                    extension.setPower(0);

//FUNNEL CODE
                    if (gamepad2.dpad_right) {
                        funnelPos = funnelPos + .12;
                        funnel.setPosition(funnelPos);
                    }
                    if (gamepad2.dpad_left) {// Close Funnel
                        funnelPos = funnelPos - 0.5;
                        funnel.setPosition(funnelPos);
                    }

//ARMS CODE
                    if (gamepad2.right_bumper) { //Arms lower
                        leftArmServoPos = leftArmServoPos + 0.34;// position= 0.86; original (add .34 -- pos .84)
                        leftArmServo.setPosition(leftArmServoPos);
                        rightArmServoPos = rightArmServoPos - 0.28;// position= 0.19; original (sub .34 -- pos .16)
                        rightArmServo.setPosition(rightArmServoPos);
                    }
                    if (gamepad2.left_bumper) { //Arms raise
                        leftArmServoPos = leftArmServoPos - 0.07;//position= 0.35; original (sub .13 -- pos .37)
                        leftArmServo.setPosition(leftArmServoPos);
                        rightArmServoPos = rightArmServoPos + 0.10;//position= 0.70; original (add .13 -- pos .63)
                        rightArmServo.setPosition(rightArmServoPos);
                    }


//MULTI-HYPO CODE
                    // horizontal slides to the bumpers

//            if (gamepad2.a) { //slides go in arm goes up, claw opens
//                horizontalSlide.setPower(-.95);//slides in
//                leftArmServoPos = leftArmServoPos + 0.485;
//                leftArmServo.setPosition(leftArmServoPos);
//                rightArmServoPos = rightArmServoPos + 0.485;
//                rightArmServo.setPosition(rightArmServoPos);//arms raise
//                clawPos = clawPos + 0.05;
//                claw.setPosition(clawPos);//claw opens
//
//            }
//
                    //SLIDES CODE
                    if (gamepad2.left_trigger >= .95) { // Slides Move Out
                        horizontalSlide.setPower(-.95);
                    } else if (gamepad2.right_trigger > .95) { //Slides Retract
                        horizontalSlide.setPower(.95);
                    } else {
                        horizontalSlide.setPower(0);
                    }


                    if (gamepad2.b) {
                        vertSlide.setPower(-.95);// slides down
//                    } else {
//                        vertSlide.setPower(0);
                    }

                    if (gamepad2.a) {
                        vertSlide.setPower(.95);//slides up
//                    } else {
//                        vertSlide.setPower(0);
                    }

                    if (gamepad2.dpad_up) {
                        vertSlide.setPower(0.1);
                    }
//                    } else {
//                        vertSlide.setPower(0.0);
//                    }


//drive by encoder
//                    if (gamepad2.b && vertSlide.getCurrentPosition()>0) {
//                        vertSlide.setPower(-.95);// slides down
//                    }// else {
//                    //   vertSlide.setPower(0);
//
//                    else if (gamepad2.a && vertSlide.getCurrentPosition()<5000) {
//                        vertSlide.setPower(.95);//slides up
//                    }
//                    else if (gamepad2.dpad_up){
//                        vertSlide.setPower(0.1);
//                    }
//                    else{
//                        vertSlide.setPower(0.0);
//                    }
//SLOW MO
                    if (gamepad1.dpad_left) {
                        leftFrontDrive.setPower(0.65);
                        rightFrontDrive.setPower(-0.65);
                        leftBackDrive.setPower(0.65);
                        rightBackDrive.setPower(-0.65);
                    }
                    if (gamepad1.dpad_right) {
                        leftFrontDrive.setPower(-0.65);
                        rightFrontDrive.setPower(0.65);
                        leftBackDrive.setPower(-0.65);
                        rightBackDrive.setPower(0.65);
                    }

                    if (gamepad1.dpad_down) {
                        leftFrontDrive.setPower(-0.65);
                        rightFrontDrive.setPower(-0.65);
                        leftBackDrive.setPower(-0.65);
                        rightBackDrive.setPower(-0.65);
                    }

                    if (gamepad1.dpad_up) {
                        leftFrontDrive.setPower(0.65);
                        rightFrontDrive.setPower(0.65);
                        leftBackDrive.setPower(0.65);
                        rightBackDrive.setPower(0.65);
                    }
//WHEELS


                    // Show the elapsed game time and wheel power.
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                    telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                    telemetry.addData("Claw", clawPos);
                    telemetry.addData("Wrist", wristPos);
                    telemetry.addData("LeftArmServo", leftArmServoPos);
                    telemetry.addData("RightArmServo", rightArmServoPos);
                    telemetry.addData("funnel", funnelPos);
                    telemetry.addData("HorizontalSlide", horizontalSlide.getPower());
                    telemetry.addData("vertSlide", vertSlide.getPower());
                    telemetry.update();

                }
            }
        }

    }
}