package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import java.util.Locale;

@TeleOp
@Config
public class turretpidtest extends LinearOpMode {
    GoBildaPinpointDriverRR pinpoint;
    Vector2d goal = new Vector2d(-63,63);
    double integralSum = 0;
    public static double Kp = 0.09;
    public static double Ki = 0.00008;
    public static double Kd = 0.0056;
    public static double Kf = 0;
    public static double minimum = 0.3; // tune this
    public static double maximum = 1; // tune this

    private double lastError = 0;
    ElapsedTime PIDtimer = new ElapsedTime();
    public static double targetPosition = 190;
    DcMotorEx turretEncoder;
    CRServo turret;
    double lastDerivative = 0;
    double lastDt = 0;

    public double toDegrees(double currentHeading) {
        return currentHeading*180/Math.PI;
    }
    public double mmToInch (double mm) {
        return mm / 25.4;
    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        double dt = PIDtimer.seconds();
        if (dt < 1e-4) dt = 1e-4;
        integralSum += error * dt;

        double derivative = (error - lastError) / dt;
        lastDerivative = derivative;
        lastDt = dt;
        lastError = error;

        PIDtimer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        if (Math.abs(output) < minimum || Double.isNaN(output) || Double.isInfinite(output)) {
            output = 0;
        }
        output = Range.clip(output, -maximum, maximum);
        return output;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        pinpoint.setOffsets(136, 36); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        turretEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret = hardwareMap.get(CRServo.class, "turret");
        double currentPos = 0;
        PIDtimer.reset();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            pinpoint.update();
            if (gamepad1.start){
                pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }
            Pose2d pinpointpos = pinpoint.getPositionRR();
            Pose2d pos = new Pose2d(-pinpointpos.position.y, pinpointpos.position.x, pinpointpos.heading.toDouble());
            double distance = Math.sqrt(Math.pow((pos.position.x-goal.x),2) + Math.pow((pos.position.y-goal.y),2));
            double dx = Math.abs(goal.x - pos.position.x);
            double dy = Math.abs(goal.y - pos.position.y);

            double robotGoalAngle = Math.toDegrees(Math.atan2(dx,dy));
//            double robotGoalAngle = toDegrees(Math.acos((63-pos.position.y)/distance));
            double desiredAngle = 165 - (robotGoalAngle+toDegrees(-pos.heading.toDouble()));
            if (desiredAngle < 0) {
                desiredAngle += 360;
            }
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, pos.heading.toDouble());
            telemetry.addData("Position", data);
            telemetry.addData("target", desiredAngle);
            telemetry.addData("distance", distance);
            telemetry.addData("robot goal angle", robotGoalAngle);
            currentPos = -(double) turretEncoder.getCurrentPosition() /(8192*4) * 360;
            telemetry.addData("encoder pos (degrees)", currentPos);
            double power = PIDControl(desiredAngle, currentPos);
            telemetry.addData("PID timer", lastDt);
            telemetry.addData("error", lastError);
            telemetry.addData("derivative", lastDerivative);
            telemetry.addData("power", power);
            if (gamepad1.b)turret.setPower(power);
            else turret.setPower(0);
            telemetry.update();

        }
    }
}
