package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.security.GuardedObject;
import java.util.List;


public class encoders_hardware {
    com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;
    DcMotorEx turretEncoder;
    DcMotorEx shooterEncoder;
    public void init (HardwareMap hwMap){
        pinpoint = hwMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(136, 36,DistanceUnit.MM);
        pinpoint.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD, com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED);
        turretEncoder = hwMap.get(DcMotorEx.class,"BL");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterEncoder = hwMap.get(DcMotorEx.class, "shooterTop");
    }
    public double getTurretPos(){
        return (double) turretEncoder.getCurrentPosition() /(8192*4) * 360;
    }
    public double getShooterVel(){
        return shooterEncoder.getVelocity(AngleUnit.DEGREES);
    }
    public Pose2d getPinpointPos(boolean reset, Pose2D initPos){
        pinpoint.update();
        if (reset) pinpoint.setPosition(initPos);
        Pose2D pinpointpos = new Pose2D(DistanceUnit.INCH, pinpoint.getPosX(DistanceUnit.INCH),pinpoint.getPosY(DistanceUnit.INCH),AngleUnit.RADIANS,pinpoint.getHeading(AngleUnit.RADIANS));
        Pose2d pos = new Pose2d(-pinpointpos.getY(DistanceUnit.INCH), pinpointpos.getX(DistanceUnit.INCH), pinpointpos.getHeading(AngleUnit.RADIANS));
        return pos;
    }
}
