package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;
import org.firstinspires.ftc.teamcode.decode.DecodeRobotHardware;

@Config
@TeleOp(name="intake + flicker + PID Test2")
public class intake_flicker_pid_test2 extends LinearOpMode {
    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;

    DcMotorEx intake;

    double integralSum = 0;
    double Kp = 0.0325;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0.0038;
    double targetVelocity = 190;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    Servo flicker1;
    boolean ball1_button_pressed;
    boolean ball1_released;
    Servo flicker2;
    boolean ball2_button_pressed;
    boolean ball2_released;
    Servo flicker3;
    boolean ball3_button_pressed;
    boolean ball3_released;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickerCount = 1;
    public static double home1 = 0.96;
    public static double home2 = 0.03;
    public static double home3 = 0.175;
    public static double score1 = 0.66;
    public static double score2 = 0.33;
    public static double score3 = 0.475;
    public static double nextTime = 0.4;
    public static double homeTime = 0.25;
    DcMotorEx lift;
    @Override
    public void runOpMode() throws InterruptedException {

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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        CycleGamepad cyclegamepad1 = new CycleGamepad(gamepad1);
        nextTimer.reset();

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            cyclegamepad1.updateRB(2);
            double power = PIDControl(targetVelocity, shooterTop.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("velocity top", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("velocity", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("reference", targetVelocity);
            dashboardTelemetry.update();

            if (gamepad1.y){
                if (nextTimer.seconds() >= nextTime){
                    flickerTimer.reset();
                    nextTimer.reset();
                    flickerCount += 1;
                }
                if (flickerCount == 4){
                    flickerCount = 1;
                }
                if (flickerCount == 1){
                    if (flickerTimer.seconds() <= homeTime){
                        flicker1.setPosition(score1);
                    }
                    else flicker1.setPosition(home1);
                }
                else if (flickerCount == 2){
                    if (flickerTimer.seconds() <= homeTime){
                        flicker2.setPosition(score2);
                    }
                    else flicker2.setPosition(home2);
                }
                else if (flickerCount == 3){
                    if (flickerTimer.seconds() <= homeTime){
                        flicker3.setPosition(score3);
                    }
                    else flicker3.setPosition(home3);
                }
            }
            else {
                if (gamepad1.x) {
                    if (!ball1_button_pressed) {
                        ball1_released = !ball1_released;
                    }
                    ball1_button_pressed = true;

                } else ball1_button_pressed = false;



                if (gamepad1.a) {
                    if (!ball2_button_pressed) {
                        ball2_released = !ball2_released;
                    }
                    ball2_button_pressed = true;

                } else ball2_button_pressed = false;



                if (gamepad1.b) {
                    if (!ball3_button_pressed) {
                        ball3_released = !ball3_released;
                    }
                    ball3_button_pressed = true;

                } else ball3_button_pressed = false;

                updateBooleans();
            }
            telemetry.addData("nexttimer:", nextTimer.seconds());
            telemetry.update();

            if (cyclegamepad1.rbPressCount == 0) {
                shooterTop.setPower(0);
                shooterBottom.setPower(0);
            } else {
                shooterTop.setPower(power);
                shooterBottom.setPower(power);
            }

            telemetry.update();



            if (gamepad2.dpad_up){
                lift.setPower(1);
            }
            else if (gamepad2.dpad_down){
                lift.setPower(-1);
            }
            else lift.setPower(0);

            if (gamepad1.dpad_up){
                intake.setPower(1);
            }
            else if (gamepad1.dpad_down){
                intake.setPower(-1);
            }
            else intake.setPower(0);

        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }

    public void updateBooleans() {
        if (ball1_released) {
            flicker1.setPosition(score1);
        } else {
            flicker1.setPosition(home1);// lower position
        }

        if (ball2_released) {
            flicker2.setPosition(score2);
        } else {
            flicker2.setPosition(home2);// lower position
        }

        if (ball3_released) {
            flicker3.setPosition(score3);
        } else {
            flicker3.setPosition(home3);// lower position
        }

    }
}
