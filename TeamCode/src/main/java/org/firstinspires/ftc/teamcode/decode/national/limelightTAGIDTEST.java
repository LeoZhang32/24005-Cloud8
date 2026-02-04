package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Config
@TeleOp
public class limelightTAGIDTEST extends LinearOpMode {
    Limelight3A limelight;
    int tagID = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null) {
                if (llResult.isValid()){
                    List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                    telemetry.addData("target", tagID);
                    if (!fiducials.isEmpty()) {
                        tagID = fiducials.get(0).getFiducialId();
                    }
                }
                else telemetry.addData("target", "INVALID");
            }
            else telemetry.addData("target", "NULL");
            telemetry.addData("pipeline", limelight.getStatus().getPipelineIndex());
            telemetry.addData("time since last", limelight.getTimeSinceLastUpdate());
            telemetry.addData("connected", limelight.isConnected());
            telemetry.update();
            telemetry.update();
        }
    }
}
