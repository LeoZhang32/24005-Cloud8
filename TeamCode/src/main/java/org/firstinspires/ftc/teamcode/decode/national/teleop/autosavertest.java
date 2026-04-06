package org.firstinspires.ftc.teamcode.decode.national.teleop;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@TeleOp (name = "autosavertest")
public class autosavertest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("RobotPrefs", Context.MODE_PRIVATE);

        double x = prefs.getFloat("x", 0);
        double y = prefs.getFloat("y", 0);
        double heading = prefs.getFloat("heading", 0);
        double turretPos = prefs.getFloat("turretPos",99);

        Pose endPose = new Pose(x, y, Math.toDegrees(heading));
        Pose2D endPoseInFTC = PoseConverter.poseToPose2D(endPose, InvertedFTCCoordinates.INSTANCE);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            telemetry.addData("turretInitPos", turretPos);
            telemetry.addData("endPosePedro", endPose);
            telemetry.addData("endPoseFTC", endPoseInFTC);
            telemetry.update();
        }
    }
}