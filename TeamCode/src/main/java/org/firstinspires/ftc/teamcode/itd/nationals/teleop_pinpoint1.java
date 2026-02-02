package org.firstinspires.ftc.teamcode.itd.nationals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class teleop_pinpoint1 extends LinearOpMode {
    GoBildaPinpointDriverRR pinpoint;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime grabTimer = new ElapsedTime();
    boolean isTransferTimerRunning = false; // Track if timer is running
    boolean isGrabTimerRunning = false; // Track if timer is running
    Boolean extendoIn = true;
    Servo IArmL;
    Servo IArmR;
    Servo IWrist;
    Servo IClaw;
    Servo IArmC;
    Servo HSlideL;
    Servo HSlideR;

    Servo OClaw;
    Servo OArm;
    Boolean specArmOn = false;

    Boolean slowModeOn = false;
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    IMU imu;

    Boolean manual_running = false;
    Boolean auto_up_button_pressed = false;
    Boolean auto_up = false;
    Boolean auto_down_button_pressed = false;
    Boolean auto_down = false;
    Boolean specscore_button_pressed = false;
    Boolean specscore = false;

    DcMotor VSlideF;
    DcMotor VSlideB;

    DigitalChannel limitSwitch;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);

        positions_and_variables pos = new positions_and_variables();
        CycleGamepad cycle_gamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cycle_gamepad2 = new CycleGamepad(gamepad2);

        IArmL = hardwareMap.get(Servo.class, "IArmL");
        IArmR = hardwareMap.get(Servo.class, "IArmR");
        IArmC = hardwareMap.get(Servo.class, "IArmC");
        IWrist = hardwareMap.get(Servo.class, "IWrist");
        IWrist.scaleRange(0.0, 0.6);
        IClaw = hardwareMap.get(Servo.class, "IClaw");
        HSlideL = hardwareMap.get(Servo.class, "HSlideL");
        HSlideR = hardwareMap.get(Servo.class, "HSlideR");

        OClaw = hardwareMap.get(Servo.class, "OClaw");
        OArm = hardwareMap.get(Servo.class, "OArm");

        // drivetrain motors
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");

        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        pinpoint.recalibrateIMU();
//        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        VSlideF = hardwareMap.get(DcMotor.class, "VSlideF");
        VSlideB = hardwareMap.get(DcMotor.class, "VSlideB");
        OArm = hardwareMap.get(Servo.class, "OArm");
        VSlideF.setDirection(DcMotorSimple.Direction.REVERSE);
        VSlideB.setDirection(DcMotorSimple.Direction.REVERSE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

            cycle_gamepad1.updateX(5);
            cycle_gamepad1.updateRB(4);
            cycle_gamepad1.updateLB(2);

            cycle_gamepad2.updateA(2);
            cycle_gamepad2.updateX(2);
            cycle_gamepad2.updateLB(3);

            slowModeOn = cycle_gamepad1.lbPressCount != 0;

            pinpoint.update();
            //drivetrain
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.7;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                pinpoint.recalibrateIMU();
            }

            double botHeading = pinpoint.getHeading();
            // Field-oriented adjustments
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (slowModeOn){
                FL.setPower(frontLeftPower * 0.5);
                BL.setPower(backLeftPower * 0.5);
                FR.setPower(frontRightPower * 0.5);
                BR.setPower(backRightPower * 0.5);
            }
            else{
                FL.setPower(frontLeftPower * 1);
                BL.setPower(backLeftPower * 1);
                FR.setPower(frontRightPower * 1);
                BR.setPower(backRightPower * 1);
            }

            dashboard.sendTelemetryPacket(packet);

            //arm movements
            if (cycle_gamepad1.xPressCount == 0){
                HSlideL.setPosition(pos.hslide_trans);
                HSlideR.setPosition(1-pos.hslide_trans);
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_trans);
                IClaw.setPosition(pos.intake_claw_close);
                extendoIn = true;
            }
            else if (cycle_gamepad1.xPressCount == 1){
                HSlideL.setPosition(pos.hslide_after_trans);
                HSlideR.setPosition(1-pos.hslide_after_trans);
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
                cycle_gamepad1.rbPressCount = 0;
                extendoIn = false;
                isTransferTimerRunning = false;
            }
            else if (cycle_gamepad1.xPressCount == 2){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
                IClaw.setPosition(pos.intake_claw_open);
                isTransferTimerRunning = false;
                extendoIn = false;
            }
            else if (cycle_gamepad1.xPressCount == 3){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_grab);
                IArmR.setPosition(1-pos.intake_arm_grab);
                IArmC.setPosition(pos.intake_coax_grab);
                IClaw.setPosition(pos.intake_claw_close);
                isTransferTimerRunning = false;
                extendoIn = false;

                if (!isGrabTimerRunning) {
                    grabTimer.reset();
                    isGrabTimerRunning = true;  // Indicate timer has started
                    telemetry.addData("Grab Timer Started", grabTimer.milliseconds());
                }
            }
            else{
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
                IClaw.setPosition(pos.intake_claw_close);
//                cycle_gamepad1.rbPressCount = 0;
                isTransferTimerRunning = false;
                extendoIn = false;
            }

            //Delayed IClaw lifting
            if (isGrabTimerRunning) {
                telemetry.addData("Grab Timer Running", grabTimer.milliseconds()); // Track progress

                if (grabTimer.milliseconds() >= 300) {
                    telemetry.addData("Grab Timer Expired", grabTimer.milliseconds());

                    if (cycle_gamepad1.xPressCount == 3) {
                        cycle_gamepad1.xPressCount = 4;
                        isGrabTimerRunning = false; // Stop tracking timer once done
                    }

                }
            }

            if (gamepad1.y) {
                cycle_gamepad1.xPressCount = 2;
            }

            //wrist movements
            if (cycle_gamepad1.xPressCount == 1 || cycle_gamepad1.xPressCount == 2 || cycle_gamepad1.xPressCount == 3){
                if (cycle_gamepad1.rbPressCount == 0){
                    IWrist.setPosition(pos.intake_wrist0);
                }
                else if (cycle_gamepad1.rbPressCount == 1){
                    IWrist.setPosition(pos.intake_wrist45);
                }
                else if (cycle_gamepad1.rbPressCount == 2){
                    IWrist.setPosition(pos.intake_wrist90);
                }
                else{
                    IWrist.setPosition(pos.intake_wrist135);
                }
            }
            else if (cycle_gamepad1.xPressCount == 0 || cycle_gamepad1.xPressCount == 4){
                if (cycle_gamepad1.rbPressCount%2 == 1){
                    IWrist.setPosition(pos.intake_wrist180);
                }
                else{
                    IWrist.setPosition(pos.intake_wrist0);
                }
            }

            //intake claw movement
//            if (!extendoIn) {
//                if (cycle_gamepad1.aPressCount == 1) {
//                    IClaw.setPosition(pos.intake_claw_close);
//                } else {
//                    IClaw.setPosition(pos.intake_claw_open);
//                }
//            }

            //outtake claw movement

            if (cycle_gamepad2.aPressCount == 1) {
                OClaw.setPosition(pos.outtake_claw_close);
                if (!isTransferTimerRunning) {
                    isTransferTimerRunning = true;
                }

            }
            else{
                OClaw.setPosition(pos.outtake_claw_open);
            }

            //Delayed IClaw opening

            if (extendoIn && isTransferTimerRunning && transferTimer.milliseconds() >= 200) {
                IClaw.setPosition(pos.intake_claw_open);
                cycle_gamepad1.aPressCount = 0;
                isTransferTimerRunning = false; // Stop tracking timer once done
                if (cycle_gamepad1.xPressCount == 0) {
                    cycle_gamepad1.xPressCount = 1;
                }
            }


            if (!isTransferTimerRunning) {
                transferTimer.reset();
                // Indicate timer has started
            }

            VSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

            if (gamepad2.left_bumper){
                cycle_gamepad2.xPressCount = -2;
            }

            if (cycle_gamepad2.xPressCount == 1){
                OArm.setPosition(pos.outtake_arm_sample);
            }
            else if (cycle_gamepad2.xPressCount == 0){
                OArm.setPosition(pos.outtake_arm_transfer);
            }
            else if (cycle_gamepad2.xPressCount == -1){
                OArm.setPosition(pos.outtake_arm_specimenScore);
            }
            else if (cycle_gamepad2.xPressCount == -2){
                OArm.setPosition(pos.outtake_arm_specimenHold);
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
                    cycle_gamepad2.xPressCount = 1;
                    auto_up = !auto_up;
                    telemetry.addData("Status", "position reached");
                    telemetry.update();
                }

            } else if (auto_down) { //viper slide auto action down
                cycle_gamepad2.xPressCount = 0;
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
                VSlideF.setPower(0.3);
                VSlideB.setPower(0.3);

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
                    telemetry.addData("Status", "Waiting to score specimen");
                    telemetry.addData("VSlideF power", VSlideF.getPower());
                    telemetry.addData("VSlideB power", VSlideB.getPower());
                    telemetry.addData("VSlideF position", VSlideF.getCurrentPosition());
                    telemetry.addData("VSlideF position", VSlideB.getCurrentPosition());
                    telemetry.addData("is at target", !VSlideF.isBusy() && !VSlideB.isBusy());
                    telemetry.update();
                }


                if (VSlideF.getCurrentPosition() > 400 || VSlideB.getCurrentPosition() > 400) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    specscore = !specscore;
                    telemetry.addData("Status", "specimen scored");
                    telemetry.update();
                }

            }
            else {
                manual_running = false;
                VSlideF.setPower(0);
                VSlideB.setPower(0);
            }

            telemetry.addData("extendo", extendoIn);
            telemetry.addData("transfer Timer Active", isTransferTimerRunning);
            telemetry.addData("transfer Timer Time", transferTimer.milliseconds());
            telemetry.addData("Grab Timer Active", isGrabTimerRunning);
            telemetry.addData("Grab Timer Time", grabTimer.milliseconds());
            telemetry.addData("Gamepad1 xPressCount", cycle_gamepad1.xPressCount);
            telemetry.addData("Gamepad1 aPressCount", cycle_gamepad1.aPressCount);
            telemetry.addData("Gamepad2 aPressCount", cycle_gamepad2.aPressCount);
            telemetry.addData("Gamepad2 xPressCount", cycle_gamepad2.xPressCount);
            telemetry.addData("VSlideF Position", VSlideF.getCurrentPosition());
            telemetry.addData("VSlideB Position", VSlideB.getCurrentPosition());
            telemetry.update();
        }
    }
}
