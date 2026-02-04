package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;
@TeleOp (name = "AAAAAAAAAA 2")
public class teleopRED2 extends LinearOpMode {
    transferintake_hardware transferAndIntake = new transferintake_hardware(this);
    shooter_hardware shooter = new shooter_hardware(this);
    encoders_hardware encoders = new encoders_hardware();
    dt_hardware dt = new dt_hardware();
    lift_hardware lift = new lift_hardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        shooter.init(hardwareMap, 0);
        transferAndIntake.init(hardwareMap);
        encoders.init(hardwareMap);
        CycleGamepad cyclegamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cyclegamepad2 = new CycleGamepad(gamepad2);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            cyclegamepad1.updateLB(2);
            cyclegamepad2.updateRB(2);
            shooter.controlOuttake(gamepad1.start,cyclegamepad2.rbPressCount == 1,true, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS,0));
            transferAndIntake.sortTransferAndIntake(false,1);
            telemetry.update();
        }
    }
}
