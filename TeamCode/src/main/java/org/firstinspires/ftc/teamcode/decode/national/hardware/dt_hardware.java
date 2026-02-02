package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class dt_hardware {
   DcMotorEx FR;
   DcMotorEx FL;
   DcMotorEx BR;
   DcMotorEx BL;
   IMU imu;
   public void init (HardwareMap hwMap){
       FR = hwMap.get(DcMotorEx.class, "FR");
       FL = hwMap.get(DcMotorEx.class, "FL");
       BR = hwMap.get(DcMotorEx.class, "BR");
       BL = hwMap.get(DcMotorEx.class, "BL");
       FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       FL.setDirection(DcMotorSimple.Direction.REVERSE);
       BL.setDirection(DcMotorSimple.Direction.REVERSE);
       imu = hwMap.get(IMU.class, "imu");
       // Adjust the orientation parameters to match your robot
       IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
               RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
               RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
       // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
       imu.initialize(parameters);
   }
   public void driveRobot(double y, double x, double rx, boolean slowMode, boolean reset){
       double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
       if (reset) imu.resetYaw();
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

       if (slowMode){
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
   }
}
