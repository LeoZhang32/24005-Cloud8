package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;
import org.firstinspires.ftc.teamcode.decode.DecodeRobotHardware;
import org.firstinspires.ftc.teamcode.decode.national.hardware.color_sensor_hardware;

import java.util.ArrayList;

@Config
@TeleOp(name="teleoptest2")
public class teleop_test2 extends LinearOpMode {
    Boolean slowModeOn = false;
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    IMU imu;
    GoBildaPinpointDriverRR pinpoint;

    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;

    DcMotorEx intake;

    double integralSum = 0;
    public static double Kp = 0.0325;
    double Ki = 0;
    double Kd = 0;
    public static double Kf = 0.00325;
    double targetVelocity = 0;
    public static double targetVClose = 140;
    public static double targetVFar = 190;
    private double lastError = 0;
    ElapsedTime PIDtimer = new ElapsedTime();

    color_sensor_hardware cSensors = new color_sensor_hardware();
    ArrayList<Flicker> flickOrder = new ArrayList<>();
    Boolean detect1;
    Boolean detect2;
    Boolean detect3;
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickCounter = 1;
    public static double home1 = 0.96;
    public static double home2 = 0.03;
    public static double home3 = 0.175;
    public static double score1 = 0.66;
    public static double score2 = 0.33;
    public static double score3 = 0.475;
    public static double nextTime = 0.4;
    public static double homeTime = 0.25;
    Boolean shootingFinished = false;
    Boolean capacityChecked = false;
    DcMotorEx lift;
    CRServo turret;
    Servo hood;
    ElapsedTime hoodTimer = new ElapsedTime();
    public static double hoodPos = 0.5;
    public static double hoodRate = 0.1;
    public static double hoodInterval = 50;
    public static double hoodPClose = 0.9;
    public static double hoodPFar = 0.2;

    ElapsedTime loopTimer = new ElapsedTime();
    double loopTime = 0;
    Servo light1;
    Servo light2;
    double oldTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        pinpoint.setOffsets(136,36);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset());
        telemetry.addData("Y offset", pinpoint.getYOffset());
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
//        telemetry.update();

//        // Retrieve the IMU from the hardware map
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);


        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setDirection(DcMotorEx.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret = hardwareMap.get(CRServo.class, "turret");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0.5);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        CycleGamepad cyclegamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cyclegamepad2 = new CycleGamepad(gamepad2);
        nextTimer.reset();
        cSensors.init(hardwareMap);
        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        PIDtimer.reset();
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            pinpoint.update();

            slowModeOn = cyclegamepad1.lbPressCount != 0;

            //drivetrain
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.7;

            if (gamepad1.start) {
//                imu.resetYaw();
                pinpoint.resetPosAndIMU();
            }
            Pose2d pos = pinpoint.getPositionRR();
            double botHeading = pos.heading.toDouble();
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, pos.heading.toDouble());
//            telemetry.addData("Position", data);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
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

            loopTimer.reset();
            cyclegamepad2.updateRB(2);
            cyclegamepad2.updateLB(2);
            cyclegamepad1.updateLB(2);
            telemetry.addData("velocity top", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("velocity", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("reference", targetVelocity);
            dashboardTelemetry.update();

            //if haven't checked for artifact, check
            if (!capacityChecked){
                //check if each spot has artifact
                detect1 = cSensors.checkDetected1();
                detect2 = cSensors.checkDetected2();
                detect3 = cSensors.checkDetected3();
                //add detected spots to array to be shot
                if (detect1){
                    flickOrder.add(new Flicker(flicker1, home1, score1));
                }
                if (detect2){
                    flickOrder.add(new Flicker(flicker2, home2, score2));
                }
                if (detect3){
                    flickOrder.add(new Flicker(flicker3, home3, score3));
                }
                //we have detected for artifacts! for next loops in press, don't check again
                capacityChecked = true;
                shootingFinished = false;
            }
            if (gamepad2.a && shooterTop.getVelocity(AngleUnit.DEGREES) > 80){
                if (!flickOrder.isEmpty() && !shootingFinished){
                    //actually move the flickers.
                    if (flickerTimer.seconds() <= homeTime){
                        //if the timer is before time to move back, it's in score position.
                        flickOrder.get(flickCounter - 1).goScore();
                    }
                    //if timer is after time to move back, move back.
                    else flickOrder.get(flickCounter - 1).goHome();

                    //if we reach the time to cycle to the next artifact, plus 1 to the counter and reset timers.
                    if (nextTimer.seconds() >= nextTime){
                        flickerTimer.reset();
                        nextTimer.reset();
                        flickCounter += 1;
                    }
                    //once the counter reaches larger than the number of spots in the array.
                    //this means that all artifacts have been shot.
                    if (flickCounter > flickOrder.size()){
                        flickCounter = 1;
                        flickOrder.clear();
                        shootingFinished = true;
                    }
                }
            }

            //once input is let go, be ready to check again, and reset everything.
            else {
                telemetry.addData("1", detect1);
                telemetry.addData("2", detect2);
                telemetry.addData("3", detect3);
                capacityChecked = false;
                flickerTimer.reset();
                nextTimer.reset();
                flickOrder.clear();
                flickCounter = 1;
                flicker1.setPosition(home1);
                flicker2.setPosition(home2);
                flicker3.setPosition(home3);
            }

            telemetry.addData("nexttimer:", nextTimer.seconds());

//            if (gamepad2.dpad_up){
//                lift.setPower(1);
//            }
//            else if (gamepad2.dpad_down){
//                lift.setPower(-1);
//            }
//            else lift.setPower(0);

            if (gamepad1.a){
                if (detect1 && detect2 && detect3) intake.setPower(-1);
                else intake.setPower(1);
            }
            else if (gamepad1.b){
                intake.setPower(-1);
            }
            else intake.setPower(0);

            if (detect1 && detect2 && detect3){
                light1.setPosition(1);
                light2.setPosition(1);
            }
            else if ((detect1 && detect2 && !detect3) || (detect1 && !detect2 && detect3) || (!detect1 && detect2 && detect3)){
                light1.setPosition(0.66);
                light2.setPosition(0.66);
            }
            else if ((detect1 && !detect2 && !detect3) || (!detect1 && !detect2 && detect3) || (!detect1 && detect2 && !detect3)){
                light1.setPosition(0.33);
                light2.setPosition(0.33);
            }
            else {
                light1.setPosition(0);
                light2.setPosition(0);
            }

            Pose2d turretPos = new Pose2d(-pos.position.y, pos.position.x, pos.heading.toDouble());
            Vector2d goal = new Vector2d(-63,63);
            double dx = Math.abs(goal.x - turretPos.position.x);
            double dy = Math.abs(goal.y - turretPos.position.y);
            double robotGoalAngle = 0;
            if (Math.abs(dx) > 1e-6 || Math.abs(dy) > 1e-6) {
                robotGoalAngle = Math.toDegrees(Math.atan(dx/dy));
            }
            double desiredAngle = 165 - robotGoalAngle + Math.toDegrees(turretPos.heading.toDouble());
            if (desiredAngle < 0) {
                desiredAngle += 360;
            }

            if (Double.isNaN(desiredAngle)) {
                desiredAngle = 0;
            }
            double distance = Math.sqrt(Math.pow((turretPos.position.x-goal.x),2) + Math.pow((turretPos.position.y-goal.y),2));
            telemetry.addData("distance", distance);
            double hoodPos = Range.clip((-0.00716426*distance+1.18404),0,1);
            hood.setPosition(hoodPos);
            turret.setPower(PIDControlTurret(desiredAngle % 360,  -(double) BL.getCurrentPosition() /(8192*4) * 360, 0.06));
            double targetVel = 0.625689*distance + 107.15305;
            double shooterPower = Range.clip(PIDControlShooter(targetVel, shooterTop.getVelocity(AngleUnit.DEGREES), 0.0325,0.00325),0, 1);
            if (cyclegamepad2.rbPressCount == 1){
                shooterTop.setPower(shooterPower);
                shooterBottom.setPower(shooterPower);
            }
            else {
                shooterTop.setPower(0);
                shooterBottom.setPower(0);
            }

            loopTime = 1/loopTimer.seconds();
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            telemetry.addData("loop time (Hz)", frequency);
            telemetry.addData("hood pos", hoodPos);
            telemetry.update();
//            if (cSensors.checkFull()) {
//                light1.setPosition(1);
//                light2.setPosition(1);
//            }
//            else if (cSensors.checkTwoHeld()){
//                light1.setPosition(0.66);
//                light2.setPosition(0.66);
//            }
//            else if (cSensors.checkOneHeld()){
//                light1.setPosition(0.33);
//                light2.setPosition(0.33);
//            }
//            else {
//                light1.setPosition(0);
//                light2.setPosition(0);
//            }

        }
    }
    private double PIDControlTurret(double reference, double state, double Kp) {
        double error = reference - state;
        double minimum = 0.05;
        double maximum = 1;
        double output = (error * Kp);
        if (Math.abs(output) < minimum) {
            output = 0;
        }
        if (output > maximum) output = maximum;
        if (output < -maximum) output = -maximum;
        return output;
    }
    private double PIDControlShooter(double reference, double state, double Kp, double Kf){
        double error = reference - state;

        return (error * Kp) + (reference * Kf);
    }
    static class Flicker {
        Servo servo;
        double home, score;
        Flicker(Servo servo, double home, double score){
            this.servo = servo;
            this.home = home;
            this.score = score;
        }
        void goHome(){
            servo.setPosition(home);
        }
        void goScore(){
            servo.setPosition(score);
        }
    }

}
