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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Up Encoder Auto")

public class UPencoderautotesting extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private Servo claw = null;
    private Servo wrist = null;
    private DcMotor vertSlide = null;
    private DcMotor horizontalSlide = null;
    private Servo funnel = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    double clawPos = 0.5;
    double wristPos = 0.7;
    double leftArmServoPos = 0.5;
    double rightArmServoPos = 0.5;
    double funnelPos = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        vertSlide = hardwareMap.get(DcMotor.class, "vertSlide");
        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        funnel = hardwareMap.get(Servo.class, "funnel");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        horizontalSlide.setDirection(DcMotor.Direction.REVERSE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vertSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vertSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                vertSlide.getCurrentPosition(),
                horizontalSlide.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.


        /*
         *  Method to perform a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the OpMode running.
         */
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
        while (opModeIsActive() && (runtime.seconds() < 30)) {
            //arms down set pos
            leftArmServo.setPosition(.39);
            rightArmServo.setPosition(.64);
            sleep(1000);
            // Ensure that the OpMode is still active
            //horizontal slides go out
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() > 2.0 && runtime.seconds() > 0)) {
                vertSlide.setPower(.95);//slides in
                telemetry.addData("Runtime", getRuntime());
                telemetry.update();
            }
//        if (horizontalSlide.getCurrentPosition() >= 0 ||  horizontalSlide.getCurrentPosition() < 200) {
//            telemetry.addData("Horz Slide Postion (inside IF)", horizontalSlide.getCurrentPosition());
//            telemetry.addData("Slides Extend Power", horizontalSlide.getPower());
//
//            horizontalSlide.setPower(.3);
//            telemetry.addData("Horz Slide Power (inside IF)", horizontalSlide.getPower());
//            telemetry.addData("Horz Slide Postion (inside IF)", horizontalSlide.getCurrentPosition());
//            telemetry.addData("Slides Extend Power", horizontalSlide.getPower());
//            telemetry.update();
//            if (horizontalSlide.getCurrentPosition()>=200){
//                telemetry.addData("Horz Slide Postion (inside ELSE IF)", horizontalSlide.getCurrentPosition());
//                telemetry.update();
//                horizontalSlide.setPower(0);
//                telemetry.addData("Horz Slide Power (inside ELSE IF)", horizontalSlide.getPower());
//                telemetry.update();
//            }
//        }


                sleep(1000);
//claw opens and closes
                claw.setPosition(.15);
                sleep(1000);
                claw.setPosition(.55);
                sleep(1000);
//slides in
//        if (opModeIsActive() && horizontalSlide.getCurrentPosition()>0) {
//                horizontalSlide.setPower(-.95);
//        } else if (opModeIsActive() && horizontalSlide.getCurrentPosition()<=0){
//            horizontalSlide.setPower(0);
//            }
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() > 2.0 && runtime.seconds() > 0)) {
                    vertSlide.setPower(-.95);//slides in
                    telemetry.addData("Runtime", getRuntime());
                    telemetry.update();
                }
                    sleep(1000);
//arms raise

                    leftArmServo.setPosition(.9);
                    rightArmServo.setPosition(.26);
                    sleep(1000);
//wrist turns
                    wrist.setPosition(.1);
                    sleep(1000);

                    //claw opens
                    claw.setPosition(.15);
                    sleep(1000);
                    //arms down
                    leftArmServo.setPosition(.39);
                    rightArmServo.setPosition(.64);
                    sleep(1000);
                    //vert slides up
//            if (opModeIsActive() && vertSlide.getCurrentPosition()<4500) {
//                vertSlide.setPower(.95);
//            } else if (opModeIsActive() && vertSlide.getCurrentPosition()>=4500 && vertSlide.getCurrentPosition()<5000) {
//                vertSlide.setPower(0.1);
//            } else{
//                vertSlide.setPower(0.1);
//            }
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() > 2.8 && runtime.seconds() > 0)) {
                        vertSlide.setPower(-.95);//slides up
                        telemetry.addData("Runtime", getRuntime());
                        telemetry.update();
                    }

                        sleep(1000);
                        //funnel servo open and close
                        funnelPos = funnelPos + 0.15;
                        funnel.setPosition(funnelPos);
                        sleep(1000);
                        funnelPos = funnelPos - 0.5;
                        funnel.setPosition(funnelPos);
                        sleep(1000);
// slides down
//            if (opModeIsActive() && vertSlide.getCurrentPosition()>0) {
//                vertSlide.setPower(-.95);
//            } else{
//                vertSlide.setPower(0);
//            }
                        resetRuntime();
                        while (opModeIsActive() && (runtime.seconds() > 2.8 && runtime.seconds() > 0)) {
                            vertSlide.setPower(.95);//slides down
                            telemetry.addData("Runtime", getRuntime());
                            telemetry.update();

                        }

                        // Determine new target position, and pass to motor controller
//            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            leftDrive.setTargetPosition(newLeftTarget);
//            rightDrive.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            leftDrive.setPower(Math.abs(speed));
//            rightDrive.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (leftDrive.isBusy() && rightDrive.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Currently at",  " at %7d :%7d",
//                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            leftDrive.setPower(0);
//            rightDrive.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move.
                        telemetry.addData("Auto", "Complete");
                        telemetry.update();
                        telemetry.addData("Status", "Run Time: " + runtime.toString());
                        telemetry.addData("Claw", clawPos);
                        telemetry.addData("Wrist", wristPos);
                        telemetry.addData("LeftArmServo", leftArmServoPos);
                        telemetry.addData("RightArmServo", rightArmServoPos);
                        telemetry.addData("funnel", funnelPos);
//        telemetry.addData("HorizontalSlide", horizontalSlide.getPower());
//        telemetry.addData("vertSlide", vertSlide.getPower());
                        telemetry.update();
                    }
                }
            }
