package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class dtmotortest extends LinearOpMode {
    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a) {
                FL.setPower(1);
            } else FL.setPower(0);
            if (gamepad1.b) {
                BL.setPower(1);
            } else BL.setPower(0);
            if (gamepad1.x) {
                FR.setPower(1);
            } else FR.setPower(0);
            if (gamepad1.y) {
                BR.setPower(1);
            } else BR.setPower(0);
        }
    }
}
