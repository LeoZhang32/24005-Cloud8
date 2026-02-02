package org.firstinspires.ftc.teamcode.decode.national;

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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@TeleOp
@Config
public class turretpidtest_pinpoint extends LinearOpMode {
    GoBildaPinpointDriverRR pinpoint;
    Vector2d goal = new Vector2d(-63,63);
    double integralSum = 0;
    public static double Kp = 0.06;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static double minimum = 0.15; // tune this
    public static double maximum = 1; // tune this

    private double lastError = 0;
    ElapsedTime PIDtimer = new ElapsedTime();
    public static double targetPosition = 190;
    DcMotorEx turretEncoder;
    CRServo turret;
    IMU imu;

    public double toDegrees(double currentHeading) {
        return currentHeading*180/Math.PI;
    }
    public double mmToInch (double mm) {
        return mm / 25.4;
    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * PIDtimer.seconds();

        double derivative = 0;
        if (PIDtimer.seconds() > 1e-6) {
            derivative = (error - lastError) / PIDtimer.seconds();
        }

        lastError = error;

        PIDtimer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        if (Math.abs(output) < minimum) {
            output = 0;
        }
        if (output > maximum) output = 1;
        if (output < -maximum) output = -1;
        return output;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        pinpoint.setOffsets(136, 36); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        turretEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret = hardwareMap.get(CRServo.class, "turret");
        double currentPos = 0;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            pinpoint.update();
            if (gamepad1.start){
                pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
                imu.resetYaw();
            }
            Pose2d pinpointpos = pinpoint.getPositionRR();
            Pose2d pos = new Pose2d(-pinpointpos.position.y, pinpointpos.position.x, pinpointpos.heading.toDouble());
            double distance = Math.sqrt(Math.pow((pos.position.x-goal.x),2) + Math.pow((pos.position.y-goal.y),2));
            double dx = Math.abs(goal.x - pos.position.x);
            double dy = Math.abs(goal.y - pos.position.y);

            double robotGoalAngle = 0;
            if (Math.abs(dx) > 1e-6 || Math.abs(dy) > 1e-6) {
                robotGoalAngle = Math.toDegrees(Math.atan(dx/dy));
            }

            double desiredAngle = 165 - robotGoalAngle + toDegrees(pinpointpos.heading.toDouble());
            if (desiredAngle < 0) {
                desiredAngle += 360;
            }

            if (Double.isNaN(desiredAngle)) {
                desiredAngle = 0;
                telemetry.addLine("WARNING: desiredAngle was NaN!");
            }

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, pos.heading.toDouble());
            String pinpointdata = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pinpointpos.position.x, pinpointpos.position.y, pinpointpos.heading.toDouble());
            telemetry.addData("position", data);
            telemetry.addData("pinpoint position", pinpointdata);
            telemetry.addData("imu heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("target", desiredAngle);
            telemetry.addData("distance", distance);
            telemetry.addData("robot goal angle", robotGoalAngle);
            if (gamepad1.a)turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            else turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currentPos = -(double) turretEncoder.getCurrentPosition() /(8192*4) * 360;
            telemetry.addData("encoder pos (degrees)", currentPos);
            double power = PIDControl(desiredAngle % 360, currentPos);
            telemetry.addData("power", power);
            if (gamepad1.b)turret.setPower(0);
            else turret.setPower(power);
            telemetry.update();

        }
    }
}
