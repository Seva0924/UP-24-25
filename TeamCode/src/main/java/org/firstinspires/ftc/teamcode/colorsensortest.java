//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.DigitalIoDeviceConfigurationType;
//
//@TeleOp
//public class colorsensortest extends LinearOpMode {
//    // Define a variable for our color sensor
//    ColorSensor test_color;
//
//    @Override
//    public void runOpMode() {
//        // Get the color sensor from hardwareMap
//         test_color = hardwareMap.get(OpticalDistanceSensor.class, "test_color");
//
//        // Wait for the Play button to be pressed
//        waitForStart();
//
//        // While the OpMode is running, update the telemetry values.
//        while (opModeIsActive()) {
//            telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
//            telemetry.addData("Red", test_color.red());
//            telemetry.addData("Green", test_color.green());
//            telemetry.addData("Blue", test_color.blue());
//            telemetry.update();
//
////            while (opModeIsActive() && test_color.yellow() )
//        }
//    }
//}