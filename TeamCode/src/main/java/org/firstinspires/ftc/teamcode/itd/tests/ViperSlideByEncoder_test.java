package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class ViperSlideByEncoder_test extends LinearOpMode {
    DcMotor frontViper;
    DcMotor backViper;
    Servo bucket;
    Boolean activation_button_pressed = false;
    Boolean activation = false;

    @Override
    public void runOpMode() throws InterruptedException {

        frontViper = hardwareMap.dcMotor.get("frontViper");
        backViper = hardwareMap.dcMotor.get("backViper");
        bucket = hardwareMap.servo.get("bucket");

        // Reset the encoder during initialization
        frontViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontViper.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

            if (gamepad1.dpad_up) {
                if (!activation_button_pressed) {
                    activation = !activation;
                }
                activation_button_pressed = true;
            } else activation_button_pressed = false;

//    updateBooleans();

//    private void updateBooleans() {

            if (activation) {

                // viper slide going up
                // Set the motor's target position
                frontViper.setTargetPosition(2000);
                backViper.setTargetPosition(2000);

                // Switch to RUN_TO_POSITION mode
                frontViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Start the motor moving by setting power ratio

                frontViper.setPower(1);
                backViper.setPower(1);

                // Loop while the motor is moving to the target
                while ((frontViper.isBusy()) && (backViper.isBusy()) && !isStopRequested()) {
                    // Let the driveRobot team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach top");
                    telemetry.addData("power", frontViper.getPower());
                    telemetry.addData("position", frontViper.getCurrentPosition());
                    telemetry.addData("is at target", !frontViper.isBusy());
                    telemetry.update();
                }
// One of the motor has reached its target position, and the program will continue

                // Stop all motion;
                frontViper.setPower(0);
                backViper.setPower(0);

                telemetry.addData("Status", "position achieved");
                telemetry.update();

//        sleep(1000);   // optional pause after each move.

                //add bucket up here
                bucket.setPosition(0.55);

                sleep(500);   // optional pause after each move.

                //add bucket down here
                bucket.setPosition(1);

                // viper slide going down
                frontViper.setTargetPosition(0);
                backViper.setTargetPosition(0);

                // Switch to RUN_TO_POSITION mode
                frontViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontViper.setPower(0.8);
                backViper.setPower(0.8);

                // Loop while the motor is moving to the target
                while ((frontViper.isBusy()) && backViper.isBusy() && !isStopRequested()) {
                    // Let the driveRobot team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach bottom");
                    telemetry.addData("power", frontViper.getPower());
                    telemetry.addData("position", frontViper.getCurrentPosition());
                    telemetry.addData("is at target", !frontViper.isBusy());
                    telemetry.update();
                }
// One of the motor has reached its target position, and the program will continue

                // Stop all motion;
                frontViper.setPower(0);
                backViper.setPower(0);

                telemetry.addData("Status", "position achieved");
                telemetry.update();

//        sleep(1000);   // optional pause after each move.

                activation = !activation;

            }
        }
    }
}