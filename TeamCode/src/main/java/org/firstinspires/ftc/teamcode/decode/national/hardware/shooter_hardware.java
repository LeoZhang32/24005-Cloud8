package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@Config
public class shooter_hardware {
    private LinearOpMode myOpMode = null;
    Limelight3A limelight;
    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;
    CRServo turret;
    Servo hood;
    double lastError= 0;
    double desiredXOffset = 0;
    double hoodPos = 0;
    public static double minimum = 0.1;
    double manualOffset = 0;
//    public static double targetVel = 0;
    ElapsedTime timer = new ElapsedTime();
    encoders_hardware encoders = new encoders_hardware();

    public void init(HardwareMap hwMap, int pipeline){
        shooterTop = hwMap.get(DcMotorEx.class, "shooterTop");
        shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom = hwMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setDirection(DcMotorEx.Direction.REVERSE);
        turret = hwMap.get(CRServo.class, "turret");
        hood = hwMap.get(Servo.class, "hood");
        limelight = hwMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        encoders.init(hwMap);
    }
    private double PIDControlShooter(double reference, double state, double Kp, double Kf){
        double error = reference - state;

        return (error * Kp) + (reference * Kf);
    }
    public void controlOuttake(boolean reset, boolean shooterInput, boolean isRed, Pose2D initPose) {
        if (isRed) limelight.pipelineSwitch(0);
        else limelight.pipelineSwitch(2);

        LLResult llResult = limelight.getLatestResult();
        Pose2d pos = encoders.getPinpointPos(reset, initPose);
        Vector2d goal = new Vector2d(-68,72);
        double dx = Math.abs(goal.x - pos.position.x);
        double dy = Math.abs(goal.y - pos.position.y);
        double robotGoalAngle = 0;
        if (Math.abs(dx) > 1e-6 || Math.abs(dy) > 1e-6) {
            robotGoalAngle = Math.toDegrees(Math.atan(dx/dy));
        }

        if (myOpMode.gamepad2.dpad_right) manualOffset += 1;
        if (myOpMode.gamepad2.dpad_left) manualOffset -= 1;

        double desiredAngle = 0 - robotGoalAngle + Math.toDegrees(pos.heading.toDouble()) + manualOffset;
        if (desiredAngle < -135) {
            desiredAngle += 360;
        }
        if (Double.isNaN(desiredAngle)) {
            desiredAngle = 0;
        }

        desiredAngle = Range.clip(desiredAngle,-135,130);


        double distance = Math.sqrt(Math.pow((pos.position.x-goal.x),2) + Math.pow((pos.position.y-goal.y),2));
        myOpMode.telemetry.addData("distance", distance);

        if (distance > 0 && distance < 60) hoodPos = 0.9;
        if (distance >= 60 && distance < 90) hoodPos = 0.6;
        if (distance >= 90) hoodPos = 0.4;

        hood.setPosition(hoodPos);

        double targetVel = 0;
        double shooterPower = 0;
        if (distance < 100) targetVel = 0.82185 * distance + 86.39032;
        else targetVel = 190;
        shooterPower = PIDControlShooter(targetVel, encoders.getShooterVel(), 0.0325,0.00325);
        double turretPower = 0;

        if (robotGoalAngle >= 0 && robotGoalAngle < 30){
            desiredXOffset = 0;
        }
        if (robotGoalAngle >= 30 && robotGoalAngle < 50) {
            desiredXOffset = 0;
        }
        if (robotGoalAngle >= 50){
            desiredXOffset = 0;
        }
        if (shooterInput){
            shooterTop.setPower(shooterPower);
            shooterBottom.setPower(shooterPower);
            if (encoders.getTurretPos() > desiredAngle - 10 && encoders.getTurretPos() < desiredAngle + 10 && encoders.getTurretPos() > -135 && encoders.getTurretPos() < 130 && distance >=100) {
                if (llResult != null){
                    if (llResult.isValid()){
                        double xOffset = llResult.getTx();
                        myOpMode.telemetry.addData("xOffset", xOffset);
                        if (isRed) turretPower = -(PIDControlLimelight(desiredXOffset, xOffset, 0.05,0.0067,0));
                        else turretPower = -(PIDControlLimelight(-desiredXOffset, xOffset, 0.05,0.00067,0));
                    }
                    else {
                        myOpMode.telemetry.addData("target", "INVALID");
                        turretPower = 0;
                    }
                }
                else {
                    myOpMode.telemetry.addData("target", "NULL");
                    turretPower = 0;
                }
            }
            else{
                myOpMode.telemetry.addData("target", "NOT IN RANGE");
                turretPower = PIDControlTurret(desiredAngle, encoders.getTurretPos(), 0.06);
            }
        }
        else {
            turretPower = PIDControlTurret(desiredAngle, encoders.getTurretPos(), 0.06);
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        }
        if (Double.isNaN(turretPower)) turret.setPower(0);
        else turret.setPower(turretPower);
        myOpMode.telemetry.addData("timer", timer.seconds());
    }
    private double PIDControlTurret(double reference, double state, double Kp) {
        double error = reference - state;
        double minimum = 0;
        double maximum = 1;
        double output = (error * Kp);
        if (Math.abs(output) < minimum) {
            output = 0;
        }
        if (output > maximum) output = maximum;
        if (output < -maximum) output = -maximum;
        return output;
    }
    private double PIDControlLimelight(double reference, double state, double Kp, double Kd, double Kf) {
        double error = reference - state;
        double maximum = 1;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (reference * Kf);

        if (Math.abs(output) < minimum) {
            output = minimum;
        }
        if (output > maximum) output = maximum;
        if (output < -maximum) output = -maximum;
        return output;
    }

    public shooter_hardware(LinearOpMode opmode) { myOpMode = opmode; }
}
