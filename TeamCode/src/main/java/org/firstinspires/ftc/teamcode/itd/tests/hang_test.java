package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.itd.nationals.CycleGamepad;
import org.firstinspires.ftc.teamcode.itd.nationals.positions_and_variables;


@Disabled
@TeleOp
public class hang_test extends LinearOpMode {
    Boolean manual_running = false;
    Boolean auto_up_button_pressed = false;
    Boolean auto_up = false;
    Boolean auto_down_button_pressed = false;
    Boolean auto_down = false;
    Boolean specscore_button_pressed = false;
    Boolean specscore = false;

    DcMotor VSlideF;
    DcMotor VSlideB;

    Servo OArm;
    DigitalChannel limitSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
        positions_and_variables pos = new positions_and_variables();
        CycleGamepad cycle_gamepad2 = new CycleGamepad(gamepad2);
        VSlideF = hardwareMap.get(DcMotor.class, "VSlideF");
        VSlideB = hardwareMap.get(DcMotor.class, "VSlideB");
        OArm = hardwareMap.get(Servo.class, "OArm");
        VSlideF.setDirection(DcMotorSimple.Direction.REVERSE);
        VSlideB.setDirection(DcMotorSimple.Direction.REVERSE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        OArm.setPosition(pos.outtake_arm_transfer);
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            VSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cycle_gamepad2.updateLB(3);
            cycle_gamepad2.updateX(2);
            if (gamepad2.y) {
                if (!auto_up_button_pressed) {
                    auto_up = !auto_up;
                }
                auto_up_button_pressed = true;
            } else auto_up_button_pressed = false;
            if (gamepad2.b) {
                if (!auto_down_button_pressed) {
                    auto_down = !auto_down;
                }
                auto_down_button_pressed = true;
            } else auto_down_button_pressed = false;

            if (cycle_gamepad2.lbPressCount == 2){
                OArm.setPosition(pos.outtake_arm_specimenHold);
            }
            else if (cycle_gamepad2.lbPressCount == 1||cycle_gamepad2.xPressCount == 1){
                OArm.setPosition(pos.outtake_arm_sample);
            }
            else if (cycle_gamepad2.lbPressCount == 0||cycle_gamepad2.xPressCount == 0){
                OArm.setPosition(pos.outtake_arm_transfer);
            }

            if (gamepad2.right_bumper) {
                if (!specscore_button_pressed) {
                    specscore = !specscore;
                }
                specscore_button_pressed = true;
            } else specscore_button_pressed = false;

            if (!limitSwitch.getState() && !auto_up && !auto_down && !manual_running) {
                //if pressed
                VSlideF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            V_SLIDES:
            if (gamepad2.dpad_up) {
                manual_running = true;
                if (VSlideF.getCurrentPosition() > 2850 || VSlideB.getCurrentPosition() > 2850) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    telemetry.addData("viper slides", "over limit");
                    telemetry.update();
                    break V_SLIDES;
                }
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(1);
                VSlideB.setPower(1);

            } else if (gamepad2.dpad_down) {
                manual_running = true;
                if (!limitSwitch.getState()) {
                    //if limit switch is pressed and dpad down
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    telemetry.addData("viper slides", "stopped");
                    telemetry.update();
                    break V_SLIDES;
                }
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(-1);
                VSlideB.setPower(-1);


            } else if (auto_up) { //viper slide auto actions

                manual_running = false;
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(1);
                VSlideB.setPower(1);


                if ((VSlideF.isBusy()) || (VSlideB.isBusy()) || !isStopRequested()) {

                    // Check for an emergency stop condition
                    if (gamepad2.start) { // **ADDED: Use right bumper for emergency stop**
                        // **ADDED: Stop the motors immediately**
                        VSlideF.setPower(0);
                        VSlideB.setPower(0);
                        auto_up = !auto_up;
//                            break; // **ADDED: Exit the loop on emergency stop**

                    }

                    // Let the driveRobot team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach top");
                    telemetry.addData("VSlideF power", VSlideF.getPower());
                    telemetry.addData("VSlideB power", VSlideB.getPower());
                    telemetry.addData("VSlideF position", VSlideF.getCurrentPosition());
                    telemetry.addData("VSlideF position", VSlideB.getCurrentPosition());
                    telemetry.addData("is at target", !VSlideF.isBusy() && !VSlideB.isBusy());
                    telemetry.update();
                }

                if (VSlideF.getCurrentPosition() > 2750 || VSlideB.getCurrentPosition() > 2750) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    auto_up = !auto_up;
                    telemetry.addData("Status", "position reached");
                    telemetry.update();
                }

            } else if (auto_down) { //viper slide auto action down

                manual_running = false;
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(-1);
                VSlideB.setPower(-1);


                if ((VSlideF.isBusy()) || (VSlideB.isBusy()) || !isStopRequested()) {

                    // Check for an emergency stop condition
                    if (gamepad2.start) { // **ADDED: Use right bumper for emergency stop**
                        // **ADDED: Stop the motors immediately**
                        VSlideF.setPower(0);
                        VSlideB.setPower(0);
                        auto_down = !auto_down;
//                            break; // **ADDED: Exit the loop on emergency stop**

                    }

                    // Let the driveRobot team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach bottom");
                    telemetry.addData("VSlideF power", VSlideF.getPower());
                    telemetry.addData("VSlideB power", VSlideB.getPower());
                    telemetry.addData("VSlideF position", VSlideF.getCurrentPosition());
                    telemetry.addData("VSlideF position", VSlideB.getCurrentPosition());
                    telemetry.addData("is at target", !VSlideF.isBusy() && !VSlideB.isBusy());
                    telemetry.update();
                }


                if (VSlideF.getCurrentPosition() < 30 || VSlideB.getCurrentPosition() < 30) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    auto_down = !auto_down;
                    telemetry.addData("Status", "position reached");
                    telemetry.update();
                }


            } else if (specscore) { //viper slide auto action down score

                manual_running = false;
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(0.5);
                VSlideB.setPower(0.5);

                if ((VSlideF.isBusy()) || (VSlideB.isBusy()) || !isStopRequested()) {

                    // Check for an emergency stop condition
                    if (gamepad2.start) { // **ADDED: Use right bumper for emergency stop**
                        // **ADDED: Stop the motors immediately**
                        VSlideF.setPower(0);
                        VSlideB.setPower(0);
                        specscore = !specscore;
//                            break; // **ADDED: Exit the loop on emergency stop**

                    }

                    // Let the driveRobot team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach bottom");
                    telemetry.addData("VSlideF power", VSlideF.getPower());
                    telemetry.addData("VSlideB power", VSlideB.getPower());
                    telemetry.addData("VSlideF position", VSlideF.getCurrentPosition());
                    telemetry.addData("VSlideF position", VSlideB.getCurrentPosition());
                    telemetry.addData("is at target", !VSlideF.isBusy() && !VSlideB.isBusy());
                    telemetry.update();
                }


                if (VSlideF.getCurrentPosition() > 130 || VSlideB.getCurrentPosition() > 130) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    specscore = !specscore;
                    telemetry.addData("Status", "position reached");
                    telemetry.update();
                }

            }
            else {
                manual_running = false;
                VSlideF.setPower(0);
                VSlideB.setPower(0);
            }
        }
    }
}
