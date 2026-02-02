package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class testByEncoder_test extends LinearOpMode {
    DcMotor frontViper;
    DcMotor backViper;
    DcMotor testEncoder;
    Servo bucket;
    Boolean activation_button_pressed = false;
    Boolean activation = false;


    @Override
    public void runOpMode() throws InterruptedException {

        frontViper = hardwareMap.dcMotor.get("frontViper");
        backViper = hardwareMap.dcMotor.get("backViper");
        testEncoder = hardwareMap.dcMotor.get("testEncoder");
        bucket = hardwareMap.servo.get("bucket");

        // Reset the encoder during initialization
        frontViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

            if (gamepad1.a) {
                if (!activation_button_pressed) {
                    activation = !activation;
                }
                activation_button_pressed = true;
            } else activation_button_pressed = false;

            updateBooleans();
        }


    }

    private void updateBooleans() {

    if (activation){

        // viper slide going up
        // Set the motor's target position to 300 ticks
        testEncoder.setTargetPosition(2850);

        // Switch to RUN_TO_POSITION mode
        testEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second

        testEncoder.setPower(0.2);

        // While the Op Mode is running, show the motor's status via telemetry
//        while (opModeIsActive()) {
//            telemetry.addData("power", testEncoder.getPower());
//            telemetry.addData("position", testEncoder.getCurrentPosition());
//            telemetry.addData("is at target", !testEncoder.isBusy());
//            telemetry.update();
//        }

        // Loop while the motor is moving to the target
        while ((testEncoder.isBusy()) && !isStopRequested()) {
            // Let the driveRobot team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting to reach top");
            telemetry.addData("power", testEncoder.getPower());
            telemetry.addData("position", testEncoder.getCurrentPosition());
            telemetry.addData("is at target", !testEncoder.isBusy());
            telemetry.update();
        }
// The motor has reached its target position, and the program will continue

        // Stop all motion;
        testEncoder.setPower(0);

        telemetry.addData("Status", "position achieved");
        telemetry.update();

        sleep(1000);   // optional pause after each move.

        //add bucket up here
        bucket.setPosition(0.55);

        sleep(1000);   // optional pause after each move.

        //add bucket down here
        bucket.setPosition(1);

        // viper slide going down
        testEncoder.setTargetPosition(0);

        // Switch to RUN_TO_POSITION mode
        testEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        //if (gamepad1.dpad_down) {
        testEncoder.setPower(0.5);
        //}else {
        //    testEncoder.setPower(0);
        //}

        // Loop while the motor is moving to the target
        while ((testEncoder.isBusy()) && !isStopRequested()) {
            // Let the driveRobot team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting to reach bottom");
            telemetry.addData("power", testEncoder.getPower());
            telemetry.addData("position", testEncoder.getCurrentPosition());
            telemetry.addData("is at target", !testEncoder.isBusy());
            telemetry.update();
        }
// The motor has reached its target position, and the program will continue

        // Stop all motion;
        testEncoder.setPower(0);

        telemetry.addData("Status", "position achieved");
        telemetry.update();

        sleep(1000);   // optional pause after each move.

        activation = !activation;



    }
    }



}
