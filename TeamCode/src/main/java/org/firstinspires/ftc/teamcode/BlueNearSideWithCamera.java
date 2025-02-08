//
///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Disabled
//@Autonomous(name="Blue Near Side With Camera", group="Robot")
//
//
//public class BlueNearSideWithCamera extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    private DcMotor frontLeft = null;
//    private DcMotor backLeft = null;
//    private DcMotor frontRight = null;
//    private DcMotor backRight = null;
//
//    private DcMotor rightSlide = null;
//    private DcMotor leftSlide = null;
//
//    private Servo outtakeLeft = null;
//    private Servo outtakeRight = null;
//
//    private CRServo intakeRight = null;
//    private CRServo intakeLeft = null;
//
//    static final double SPEED = 0.75;
//    static final double GUMMY_POWER = 0.75;
//    static final double SLIDES_POWER = 0.75;
//    static final double IN_PER_TICK = 0.02288067;
//    static final double LATERAL_IN_PER_TICK = 0.02481154;
//    static final double DEGREES_PER_TICK = 0;
//    static final int RIGHT_SLIDE_MAX = 3260;
//    static final int LEFT_SLIDE_MAX = 3140;
//
//    @Override
//    public void runOpMode() {
//        // Initialize the drive system variables.
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
//        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
//        outtakeLeft = hardwareMap.get(Servo.class, "outtakeLeft");
//        outtakeRight = hardwareMap.get(Servo.class, "outtakeRight");
//        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
//        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
//
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlide.setDirection(DcMotor.Direction.REVERSE);
//        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftSlide.setDirection(DcMotor.Direction.FORWARD);
//        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
////        outtakeLeft.setPosition(.75);
//////        outtakeRight.setPosition(-.75);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Ready to run");
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        while (opModeIsActive()) {
//                driveByEncoders(SPEED, 12, true);
//                strafeByEncoders(SPEED, 36, true);
//                turnByEncoders(SPEED, 90, true);
//                driveByEncoders(SPEED, 6, true);
//                strafeByEncoders(SPEED, 6, true);
////                raiseSlides(SLIDES_POWER);
////                rotateBox();
////                dropPixels();
//                strafeByEncoders(SPEED, 30, false);
////                stopWheels();
//            sleep(2000);
//
//            }
////            else if (c_position == 2) {
////                driveByEncoders(SPEED, 12, true);
////                strafeByEncoders(SPEED, 36, true);
////                turnByEncoders(SPEED, 90, true);
////                driveByEncoders(SPEED, 6, true);
////                raiseSlides(SLIDES_POWER);
////                rotateBox();
////                dropPixels();
////                strafeByEncoders(SPEED, 24, false);
////                stopWheels();
////            }
////            else {
////                driveByEncoders(SPEED, 12, true);
////                strafeByEncoders(SPEED, 36, true);
////                turnByEncoders(SPEED, 90, true);
////                driveByEncoders(SPEED, 6, true);
////                strafeByEncoders(SPEED, 6, false);
////                raiseSlides(SLIDES_POWER);
////                rotateBox();
////                dropPixels();
////                strafeByEncoders(SPEED, 18, false);
////                stopWheels();
////            }
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//            sleep(1000);
//        }
//
//
////    private void driveByEncoders(double power, double distance, boolean direction) {
////        int encoderPosition = (int) (distance / IN_PER_TICK);
////
////        if (direction) { // forward
////            frontLeft.setTargetPosition(encoderPosition);
////            backLeft.setTargetPosition(encoderPosition);
////            frontRight.setTargetPosition(encoderPosition);
////            backRight.setTargetPosition(encoderPosition);
////
////            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            frontLeft.setPower(SPEED);
////            backLeft.setPower(SPEED);
////            frontRight.setPower(SPEED);
////            backRight.setPower(SPEED);
////
////        }
////        else { // backward
////            frontLeft.setTargetPosition(-encoderPosition);
////            backLeft.setTargetPosition(-encoderPosition);
////            frontRight.setTargetPosition(-encoderPosition);
////            backRight.setTargetPosition(-encoderPosition);
////
////            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            frontLeft.setPower(-SPEED);
////            backLeft.setPower(-SPEED);
////            frontRight.setPower(-SPEED);
////            backRight.setPower(-SPEED);
////        }
////        while (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy()){}
////
////        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
////    }
//
//    private void strafeByEncoders(double power, double distance, boolean direction) {
//        int encoderPosition = (int) (distance / LATERAL_IN_PER_TICK);
//
//        if (direction) { // strafe left
//            frontLeft.setTargetPosition(-encoderPosition);
//            backLeft.setTargetPosition(encoderPosition);
//            frontRight.setTargetPosition(encoderPosition);
//            backRight.setTargetPosition(-encoderPosition);
//
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            frontLeft.setPower(-SPEED);
//            backLeft.setPower(SPEED);
//            frontRight.setPower(SPEED);
//            backRight.setPower(-SPEED);
//        }
//        else { // strafe right
//            frontLeft.setTargetPosition(encoderPosition);
//            backLeft.setTargetPosition(-encoderPosition);
//            frontRight.setTargetPosition(-encoderPosition);
//            backRight.setTargetPosition(encoderPosition);
//
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            frontLeft.setPower(SPEED);
//            backLeft.setPower(-SPEED);
//            frontRight.setPower(-SPEED);
//            backRight.setPower(SPEED);
//        }
//        while (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy()){}
//
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
////    private void turnByEncoders(double power, double degrees, boolean direction) {
////        int encoderPosition = (int) (degrees / DEGREES_PER_TICK);
////
////        if (direction) { // turn left
////            frontLeft.setTargetPosition(-encoderPosition);
////            backLeft.setTargetPosition(-encoderPosition);
////            frontRight.setTargetPosition(encoderPosition);
////            backRight.setTargetPosition(encoderPosition);
////
////            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            frontLeft.setPower(-SPEED);
////            backLeft.setPower(-SPEED);
////            frontRight.setPower(SPEED);
////            backRight.setPower(SPEED);
////        }
////        else { // turn right
////
////            frontLeft.setTargetPosition(encoderPosition);
////            backLeft.setTargetPosition(encoderPosition);
////            frontRight.setTargetPosition(-encoderPosition);
////            backRight.setTargetPosition(-encoderPosition);
////
////            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            frontLeft.setPower(SPEED);
////            backLeft.setPower(SPEED);
////            frontRight.setPower(-SPEED);
////            backRight.setPower(-SPEED);
////
////        }
////        while (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy()){}
////
////        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////    }
//
////    void raiseSlides (double power) {
////        rightSlide.setTargetPosition(RIGHT_SLIDE_MAX);
////        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightSlide.setPower(SLIDES_POWER);
////        leftSlide.setTargetPosition(LEFT_SLIDE_MAX);
////        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        leftSlide.setPower(SLIDES_POWER);
////        while (leftSlide.isBusy() || rightSlide.isBusy()){}
////    }
////
////    void rotateBox () {
////        outtakeLeft.setPosition(1);
//////        outtakeRight.setPosition(-1);
////    }
////
////    void dropPixels() {
////        ElapsedTime runtime = new ElapsedTime();
////        while (runtime.seconds() < 1) {
////            intakeLeft.setPower(GUMMY_POWER);
//////            intakeRight.setPower(GUMMY_POWER);
////        }
////        intakeLeft.setPower(0);
//////        intakeRight.setPower(0);
////        runtime.reset();
////    }
////
////    void stopWheels () {
////        frontLeft.setPower(0);
////        backLeft.setPower(0);
////        frontRight.setPower(0);
////        backRight.setPower(0);
////    }
//}
