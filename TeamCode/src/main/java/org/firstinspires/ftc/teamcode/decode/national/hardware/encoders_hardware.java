package org.firstinspires.ftc.teamcode.decode.national.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.R;

import java.security.GuardedObject;
import java.util.List;


public class encoders_hardware {
    com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;
    DcMotorEx turretEncoder;
    DcMotorEx shooterEncoder;
    boolean resetted = false;
    ElapsedTime resetTimer = new ElapsedTime();
    public void init (HardwareMap hwMap){
        pinpoint = hwMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(136, 36,DistanceUnit.MM);
        pinpoint.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD, com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED);
        turretEncoder = hwMap.get(DcMotorEx.class,"BL");
        shooterEncoder = hwMap.get(DcMotorEx.class, "shooterTop");
    }
    public double getTurretPos(){
        return (double) turretEncoder.getCurrentPosition() /(8192*4) * 360;
    }
//    public void resetTurretPos(boolean reset){
//        if (reset && !resetted) {
//            turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            resetTimer.reset();
//            resetted = true;
//        }
//        else {
//        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        if (resetTimer.seconds() >= 5){
//            resetted = false;
//        }
//    }
    public double getShooterVel(){
        return shooterEncoder.getVelocity(AngleUnit.DEGREES);
    }
    //initPos is in FTC Coordinates. Change it into Pinpoint to reset position.
    public Pose2D getPinpointPos(boolean reset, Pose2D initPos){
        pinpoint.update();
        //get raw pinpoint data.
        Pose2D pinpointPos = new Pose2D(INCH, pinpoint.getPosX(INCH),pinpoint.getPosY(INCH), RADIANS,pinpoint.getHeading(RADIANS));
        //convert pinpoint measured coordinates into FTC.
        Pose2D FTCPos = convertPinpointToFTC(pinpointPos);
        if (reset){
            //reset the pinpoint's position in pinpoint coordinates, converted from FTC Coordinates of initPos.
            pinpoint.setPosition(convertFTCToPinpoint(initPos));
        }
        return FTCPos;
    }
    private Pose2D convertPinpointToFTC(Pose2D PPPos){
        Pose2D FTCPos = new Pose2D(INCH, PPPos.getX(INCH), PPPos.getY(INCH), RADIANS, PPPos.getHeading(RADIANS));
        return FTCPos;
    }
    private Pose2D convertFTCToPinpoint(Pose2D FTCPos){
        Pose2D PPPos = new Pose2D(INCH, FTCPos.getX(INCH), FTCPos.getY(INCH), RADIANS, FTCPos.getHeading(RADIANS));
        return PPPos;
    }
}
