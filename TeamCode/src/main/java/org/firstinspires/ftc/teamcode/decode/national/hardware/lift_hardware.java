package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lift_hardware {
    private LinearOpMode myOpMode = null;
    public lift_hardware(LinearOpMode opmode) {myOpMode = opmode;}
    DcMotorEx lift;
    public void init(HardwareMap hwMap){
        lift = hwMap.get(DcMotorEx.class,"lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void liftRobot() {
        if (myOpMode.gamepad2.left_trigger > 0.5 && myOpMode.gamepad2.right_trigger > 0.5){
            lift.setPower(1);
        }
        else lift.setPower(0);
    }
}
