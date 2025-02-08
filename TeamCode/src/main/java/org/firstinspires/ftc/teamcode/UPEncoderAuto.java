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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name="UP Encoder Auto Drive Test", group="Robot")
public class UPEncoderAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         rightBackDrive  = null;
    private DcMotor         leftBackDrive  = null;
    private DcMotor vertSlide = null;
    private Servo funnel = null;
    private Servo claw = null;
    private Servo wrist = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    double funnelPos = 0.5;
    double clawPos = 0.5;
    double wristPos = 0.5;
    double leftArmServoPos = 0.5;
    double rightArmServoPos = 0.5;
    static final double     COUNTS_PER_MOTOR_REV    = 384.5  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.95;
    static final double     TURN_SPEED              = 0.95;
    static final double     SLOW_SPEED             = 0.6;
    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        vertSlide = hardwareMap.get(DcMotor.class, "vertSlide");
        funnel = hardwareMap.get(Servo.class, "funnel");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        funnel = hardwareMap.get(Servo.class, "funnel");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        vertSlide.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        funnel.setPosition(0);//funnel closed set pos
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);//forward
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);//back
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);//forward
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);//back

        encoderDrive(DRIVE_SPEED,  -7,  -7, 5.0);  //straif right

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        vertSlide.setDirection(DcMotor.Direction.FORWARD);
        encoderDrive(DRIVE_SPEED,  -30,  -30, 5.0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        encoderDrive(DRIVE_SPEED,  5,  5, 5.0);
 //vertSlide.setPower(0);

//        while(vertSlide.getCurrentPosition() <= -490 && vertSlide.getCurrentPosition() >= -530) {
//            vertSlide.setPower(.01);
//            sleep(5000);
//            telemetry.addData("Currently at",  " at %7d", vertSlide.getCurrentPosition());
//            telemetry.update();
//        }
        leftArmServo.setPosition(0.84);//arms down
        rightArmServo.setPosition(0.22);//arms down
        sleep(2000);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5 && runtime.seconds() > 0)) {
            vertSlide.setPower(.95);//slides up
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
//        encoderDrive(DRIVE_SPEED,  -7,  -7, 5.0);  // S1:
        encoderDrive(DRIVE_SPEED,  -9,  9, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
//        encoderDrive(DRIVE_SPEED,  -7,  -7, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(DRIVE_SPEED,  -4,  -4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            vertSlide.setPower(0);
            encoderDrive(DRIVE_SPEED,  -2,  -2, 5.0);
            funnel.setPosition(.62);//open
            sleep(700);
            funnel.setPosition(0);//close
        }
        encoderDrive(DRIVE_SPEED,  13,  13, 5.0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1 && runtime.seconds() > 0)) {
            vertSlide.setPower(-.95);//slides down
            wrist.setPosition(0.26);//wrist turns for pick up
            claw.setPosition(.15);//open
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
        vertSlide.setPower(0);
//        encoderDrive(DRIVE_SPEED,  -9,  9, 5.0);  // S1: For
        ;
//        encoderDrive(TURN_SPEED,  2,  2, 5.0);
        encoderDrive(TURN_SPEED,  -15,  15, 7.0);//
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        encoderDrive(SLOW_SPEED,  6.5,  6.5, 10.0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);//forward
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);//back
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);//forward
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);//back

        encoderDrive(SLOW_SPEED,  -4,  -4, 5.0);  //straif right

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        encoderDrive(TURN_SPEED,  -2,  2, 10.0);
        sleep(300);
        claw.setPosition(.7);//close
//        encoderDrive(TURN_SPEED,  2,  -2, 10.0);
        sleep(500);
        leftArmServo.setPosition(.43);//arms up
        rightArmServo.setPosition(.6);//arms uo
        sleep(500);
        wrist.setPosition(.9);//wrist turns for drop
        sleep(1000);
        claw.setPosition(.15);//calw opens
        sleep(1000);

        leftArmServo.setPosition(0.84);//arms down
        rightArmServo.setPosition(0.22);//arms down
        sleep(100);
        wrist.setPosition(0.275);//wrist turns for pick up
        encoderDrive(TURN_SPEED,  -9,  -9, 10.0);
        encoderDrive(TURN_SPEED,  15,  -15, 7.0);
//        encoderDrive(TURN_SPEED,  -15,  -15, 10.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5 && runtime.seconds() > 0)) {
            vertSlide.setPower(.95);//slides up
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
//        encoderDrive(TURN_SPEED,  -12,  -12, 7.0);
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            encoderDrive(TURN_SPEED,  -12,  -12, 7.0);
            vertSlide.setPower(0);
            funnel.setPosition(.62);//open
            sleep(2000);
            funnel.setPosition(0);//close
        }
        encoderDrive(TURN_SPEED,  10,  10, 7.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5 && runtime.seconds() > 0)) {
            vertSlide.setPower(-.95);//slides down
            encoderDrive(TURN_SPEED,  -15,   15, 7.0);
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
//        encoderDrive(TURN_SPEED,  -15,   15, 7.0);

//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

//        encoderDrive(DRIVE_SPEED,  -4,  -4, 5.0);  //straif left
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        vertSlide.setDirection(DcMotor.Direction.FORWARD);

        encoderDrive(TURN_SPEED,   6.7,   6.7, 7.0);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED,  -9,  -9, 5.0);  //straif left
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        vertSlide.setDirection(DcMotor.Direction.FORWARD);

        claw.setPosition(.7);//close
        sleep(650);
        leftArmServo.setPosition(.43);//arms up
        rightArmServo.setPosition(.6);//arms uo
        sleep(650);
        wrist.setPosition(.9);//wrist turns for drop
        sleep(900);
        claw.setPosition(.15);//calw opens
        sleep(900);
//        encoderDrive(TURN_SPEED,  5,   -5, 7.0);
        encoderDrive(TURN_SPEED,  -8,   -8, 7.0);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED,  11,  11, 5.0);  //straif right
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        vertSlide.setDirection(DcMotor.Direction.FORWARD);

        encoderDrive(TURN_SPEED,  14,   -14, 7.0);
        runtime.reset();
        leftArmServo.setPosition(0.84);//arms down
        rightArmServo.setPosition(0.22);//arms down
        sleep(100);
        while (opModeIsActive() && (runtime.seconds() < 2.4 && runtime.seconds() > 0)) {
            vertSlide.setPower(.95);//slides up
            encoderDrive(TURN_SPEED,  -14,   -14, 7.0);
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
//        encoderDrive(TURN_SPEED,  -14,   -14, 7.0);
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            vertSlide.setPower(0);
            funnel.setPosition(.62);//open
            sleep(2000);
            funnel.setPosition(0);//close
        }
        encoderDrive(TURN_SPEED,  3,   3, 7.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.4 && runtime.seconds() > 0)) {
            vertSlide.setPower(-.95);//slides down
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
//        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
//            leftFrontDrive.setPower(0);
//            leftBackDrive.setPower(0);
//            rightFrontDrive.setPower(0);
//            rightBackDrive.setPower(0);
//            vertSlide.setPower(.10);//slides up
//        }
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.8 && runtime.seconds() > 0)) {
//            vertSlide.setPower(.10);
//            sleep(1000);
//            funnel.setPosition(.5);
//            sleep(1000);
//            funnel.setPosition(.62);
////            funnelPos = funnelPos + 0.5;
////            funnel.setPosition(funnelPos);
//        }yyyy

//        sleep(500);
//        if( opModeIsActive() && vertSlide.getCurrentPosition()<=0){
//            vertSlide.setPower(0);
//        }
//        vertSlide.setPower(0);
//        runtime.reset();
//        while (runtime.seconds() < 1 && runtime.seconds() > 0) {
//            vertSlide.setPower(.95);//slides up
//            telemetry.addData("Runtime", getRuntime());
//            telemetry.update();
//        }
//        runtime.reset();
//        if(runtime.seconds() < 2.8 && runtime.seconds() > 0) {
//            vertSlide.setPower((-.10));
//            funnelPos = funnelPos + 0.5;
//            funnel.setPosition(funnelPos);
//        }
//        runtime.reset();
//       if(runtime.seconds() < 2.8 && runtime.seconds() > 0) {
//            funnel.setPosition(.62);
//            vertSlide.setPower(.95);//slides down
//            telemetry.addData("Runtime", getRuntime());
//            telemetry.update();
//        }
//        resetRuntime();
//        if (runtime.seconds() < 1 && runtime.seconds() > 0) {
//            vertSlide.setPower(.95);//slides up
//        }
//            sleep(2000);
//        resetRuntime();
//        telemetry.addData("slides", "up");
//        if (runtime.seconds()< 5  && runtime.seconds()>0){
//            sleep(1000);
//            vertSlide.setPower(.01);
//            funnel.setPosition(0);
//            sleep(1000);
//            funnel.setPosition(.62);
//        }
        vertSlide.setPower(0);
        telemetry.addData("left back power: " , leftBackDrive.getPower());
        telemetry.addData("left front power: ", leftFrontDrive.getPower());
        telemetry.addData("right back power: ",rightBackDrive.getPower());
        telemetry.addData("right front power: ",rightFrontDrive.getPower() );
        telemetry.update();
        telemetry.addData("left back power: " , leftBackDrive.getCurrentPosition());
        telemetry.addData("left front power: ", leftFrontDrive.getCurrentPosition());
        telemetry.addData("right back power: ",rightBackDrive.getCurrentPosition());
        telemetry.addData("right front power: ",rightFrontDrive.getCurrentPosition() );
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());
                telemetry.update();


            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.

        }

    }

}