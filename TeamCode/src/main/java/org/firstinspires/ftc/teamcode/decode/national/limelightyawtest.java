package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@Config
@TeleOp
public class limelightyawtest extends LinearOpMode {
    Limelight3A limelight;
    CRServo turret;
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    public static double kD = 0.00025;
    public static double kF = 0;
    public static double kP = 0.0367;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        turret = hardwareMap.get(CRServo.class,"turret");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                double xOffset = llResult.getTx();
                telemetry.addData("x offset", xOffset);
                turret.setPower(-(Range.clip(PIDControlTurret(0,xOffset,kP,kD,kF),-1,1)));
            }
            else{
                if (llResult == null) telemetry.addData("target", "NULL");
                else if (!llResult.isValid()) telemetry.addData("target", "INVALID");
                turret.setPower(0);
            }
            telemetry.addData("pipeline", limelight.getStatus().getPipelineIndex());
            telemetry.addData("time since last", limelight.getTimeSinceLastUpdate());
            telemetry.addData("connected", limelight.isConnected());
            telemetry.update();
        }
    }
    private double PIDControlTurret(double reference, double state, double Kp, double Kd, double Kf) {
        double error = reference - state;
        double minimum = 0;
        double maximum = 1;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (reference * Kf);

        if (Math.abs(output) < minimum) {
            output = 0;
        }
        if (output > maximum) output = maximum;
        if (output < -maximum) output = -maximum;
        return output;
    }
}
