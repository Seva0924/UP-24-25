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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
//@Disabled
public class UPAutoTestingDrivebyTime extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private DcMotor vertSlide = null;
    private Servo funnel = null;
    private Servo claw = null;
    private Servo wrist = null;
//    private DcMotor horizontalSlide = null;

//    private Servo funnel = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double SLIDE_SPEED = 0.95;
    static final double SLOW_SPEED = 0.25;
    static final double SLOW_SPEEDY = 0.3;
    double clawPos = 0.5;
    double wristPos = 0.5;
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
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        funnel = hardwareMap.get(Servo.class, "funnel");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        vertSlide.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
//backwards
        runtime.reset();
        funnel.setPosition(.62);
        wrist.setPosition(.35);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            leftArmServo.setPosition(.36);
            rightArmServo.setPosition(.67);
        }
        while (opModeIsActive() && (runtime.seconds() < 3.5 && runtime.seconds() > 0.5)) {
            leftFrontDrive.setPower(-SLOW_SPEED);
            leftBackDrive.setPower(-SLOW_SPEED);
            rightFrontDrive.setPower(-SLOW_SPEED);
            rightBackDrive.setPower(-SLOW_SPEED);
            vertSlide.setPower(-.95);//slides up
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
            telemetry.addData("Backwards Successful", getRuntime());
            telemetry.update();
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

//            leftArmServo.setPosition(leftArmServoPos);
//            rightArmServo.setPosition(rightArmServoPos);
            //arms down
            leftArmServo.setPosition(.36);
            rightArmServo.setPosition(.67);

            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            vertSlide.setPower(-.10);//slides up
        }


        while (opModeIsActive() && (runtime.seconds() < 1)) {
            vertSlide.setPower((-.10));
            funnelPos = funnelPos - 0.5;
            funnel.setPosition(funnelPos);
            sleep(2000);
            funnel.setPosition(.62);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(TURN_SPEED);
            rightBackDrive.setPower(TURN_SPEED);//backwards
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();

        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        while (opModeIsActive() && (runtime.seconds() < 3.2 && runtime.seconds() > 0.2)) {
            vertSlide.setPower(.95);//slides down
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(-TURN_SPEED);
            leftBackDrive.setPower(-TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);//back forwards
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() < 1 && runtime.seconds() > 0.2)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        //arms down
        while (opModeIsActive() && (runtime.seconds() < 3 && runtime.seconds() > 2.8)) {
            leftArmServo.setPosition(.39);
            rightArmServo.setPosition(.64);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.87 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(-TURN_SPEED);
            leftBackDrive.setPower(-TURN_SPEED);
            rightFrontDrive.setPower(TURN_SPEED);
            rightBackDrive.setPower(TURN_SPEED);

        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        leftArmServo.setPosition(.39);
        rightArmServo.setPosition(.64);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(TURN_SPEED);
            rightBackDrive.setPower(TURN_SPEED);
        }
        while (opModeIsActive() && (runtime.seconds() < 0.5 && runtime.seconds() > 0.4)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
//        while (opModeIsActive() && (runtime.seconds() > 2.0 && runtime.seconds() > 0)) {
//            horizontalSlide.setPower(-.95);//slides out
//            telemetry.addData("Runtime", getRuntime());
////            telemetry.update();
//        }


        claw.setPosition(.15);//open
        sleep(2000);
        claw.setPosition(.7);///close
        sleep(2000);
        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() > 2.0 && runtime.seconds() > 0)) {
//            horizontalSlide.setPower(.95);//slides in
//            telemetry.addData("Runtime", getRuntime());
//            telemetry.update();
//        }
        //arms raise
        leftArmServo.setPosition(.9);
        rightArmServo.setPosition(.26);
        //wrist turns for drop
        wrist.setPosition(1);
        sleep(1000);

        //claw opens
        claw.setPosition(.15);
        sleep(1000);
//        wrist.setPosition(.35);//wrist turns for pickup
        //arms down
//        leftArmServo.setPosition(.39);
//        rightArmServo.setPosition(.64);
        sleep(1000);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(-TURN_SPEED);
            leftBackDrive.setPower(-TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);
        }
        while (opModeIsActive() && (runtime.seconds() < 1.5 && runtime.seconds() > 0.5)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.45 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);

        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5 && runtime.seconds() > 0)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        //arms down
        leftArmServo.setPosition(.39);
        rightArmServo.setPosition(.64);
        sleep(1000);
        wrist.setPosition(.35);//wrist turns for pickup

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3 && runtime.seconds() > 0)) {
            vertSlide.setPower(-.95);//slides up
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.8 && runtime.seconds() > 0)) {
            vertSlide.setPower((-.10));
            funnelPos = funnelPos + 0.5;
            funnel.setPosition(funnelPos);
        }
        sleep(1000);
        claw.setPosition(.15);

        runtime.reset();
        sleep(1000);
        while (opModeIsActive() && (runtime.seconds() < 3 && runtime.seconds() > 0)) {
            vertSlide.setPower(.95);//slides down
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        rightBackDrive.setPower(0);
//        vertSlide.setPower(0);
//        horizontalSlide.setPower(0);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Claw", clawPos);
        telemetry.addData("Wrist", wristPos);
        telemetry.addData("LeftArmServo", leftArmServoPos);
        telemetry.addData("RightArmServo", rightArmServoPos);
        telemetry.addData("funnel", funnelPos);
//        telemetry.addData("HorizontalSlide", horizontalSlide.getPower());
        telemetry.addData("vertSlide", vertSlide.getPower());
        telemetry.addData("Turn Speed", TURN_SPEED);
        telemetry.update();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    }
}

