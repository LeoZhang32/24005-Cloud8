package org.firstinspires.ftc.teamcode.decode.national.teleop.RED;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;
import org.firstinspires.ftc.teamcode.decode.national.hardware.dt_hardware;
import org.firstinspires.ftc.teamcode.decode.national.hardware.encoders_hardware;
import org.firstinspires.ftc.teamcode.decode.national.hardware.lift_hardware;
import org.firstinspires.ftc.teamcode.decode.national.hardware.shooter_hardware;
import org.firstinspires.ftc.teamcode.decode.national.hardware.transferintake_hardware;

@Config
@TeleOp (name = "A TELEOP RED PPG", group = "AAA RED")
public class teleopRED_PPG extends LinearOpMode {
    int motif = 3;
    shooter_hardware shooter = new shooter_hardware(this);
    transferintake_hardware transferAndIntake = new transferintake_hardware(this);
    encoders_hardware encoders = new encoders_hardware();
    dt_hardware dt = new dt_hardware();
    lift_hardware lift = new lift_hardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("RobotPrefs", Context.MODE_PRIVATE);

        double x = prefs.getFloat("x", 0);
        double y = prefs.getFloat("y", 0);
        double heading = prefs.getFloat("heading", 0);
        double turretEndPos = prefs.getFloat("turretPos", 0);

        Pose endPose = new Pose(x, y, heading);
        Pose2D endPoseInFTC = PoseConverter.poseToPose2D(endPose, InvertedFTCCoordinates.INSTANCE);

        shooter.init(hardwareMap, 0);
        transferAndIntake.init(hardwareMap);
        encoders.init(hardwareMap);
        dt.init(hardwareMap);
        lift.init(hardwareMap);
        CycleGamepad cyclegamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cyclegamepad2 = new CycleGamepad(gamepad2);

        waitForStart();

        //new test
        DcMotorEx turretEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100);
        //new test ends

        while (!isStopRequested() && opModeIsActive()){
            cyclegamepad1.updateLB(2);
            cyclegamepad2.updateRB(2);
            shooter.controlOuttake(gamepad1.start,cyclegamepad2.rbPressCount == 1,true, endPoseInFTC, turretEndPos);
            transferAndIntake.sortTransferAndIntake(gamepad2.x || gamepad2.y, motif);
            dt.driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x * 0.7, cyclegamepad1.lbPressCount == 1, gamepad1.start);
            lift.liftRobot();
            telemetry.update();
        }
    }
}