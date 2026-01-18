package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;
import org.firstinspires.ftc.teamcode.decode.DecodeRobotHardware;

import java.util.ArrayList;

@Config
@TeleOp(name="teleoptest1")
public class teleop_test1 extends LinearOpMode {
    Boolean slowModeOn = false;
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    IMU imu;

    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;

    DcMotorEx intake;

    double integralSum = 0;
    public static double Kp = 0.0325;
    double Ki = 0;
    double Kd = 0;
    public static double Kf = 0.00325;
    public static double targetVelocity = 190;
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
    ElapsedTime loopTimer = new ElapsedTime();
    double loopTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        CycleGamepad cyclegamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cyclegamepad2 = new CycleGamepad(gamepad2);
        nextTimer.reset();
        cSensors.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

            slowModeOn = cyclegamepad1.lbPressCount != 0;

            //drivetrain
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.7;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
            double power = PIDControl(targetVelocity, shooterTop.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("velocity top", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("velocity", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("reference", targetVelocity);
            dashboardTelemetry.update();

            if (gamepad2.a){
                //if haven't checked for artifact this press, check
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
                else {
                    flicker1.setPosition(home1);
                    flicker2.setPosition(home2);
                    flicker3.setPosition(home3);
                }
            }
            //once input is let go, be ready to check again, and reset everything.
            else {
                detect1 = cSensors.checkDetected1();
                detect2 = cSensors.checkDetected2();
                detect3 = cSensors.checkDetected3();
                telemetry.addData("1", detect1);
                telemetry.addData("2", detect2);
                telemetry.addData("3", detect3);

                capacityChecked = false;
                flickerTimer.reset();
                nextTimer.reset();
                flickOrder.clear();
                flickCounter = 1;

            }

            telemetry.addData("nexttimer:", nextTimer.seconds());
            telemetry.update();

            if (cyclegamepad2.rbPressCount == 0) {
                shooterTop.setPower(0);
                shooterBottom.setPower(0);
            } else {
                shooterTop.setPower(power);
                shooterBottom.setPower(power);
            }

            telemetry.update();


//
//            if (gamepad2.dpad_up){
//                lift.setPower(1);
//            }
//            else if (gamepad2.dpad_down){
//                lift.setPower(-1);
//            }
//            else lift.setPower(0);

            if (gamepad1.a){
                intake.setPower(1);
            }
            else if (gamepad1.b){
                intake.setPower(-1);
            }
            else intake.setPower(0);

            turret.setPower(gamepad2.left_stick_x*0.6);
            loopTime = 1/loopTimer.seconds();
            telemetry.addData("loop time (Hz)", loopTime);
            telemetry.update();
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * PIDtimer.seconds();

        double derivative = (error - lastError) / PIDtimer.seconds();
        lastError = error;

        PIDtimer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
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
