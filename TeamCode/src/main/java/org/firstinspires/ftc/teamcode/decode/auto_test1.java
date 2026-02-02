package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.itd.nationals.positions_and_variables;

import java.util.List;

@Disabled
@Autonomous (name = "auto_test1")

public final class auto_test1 extends LinearOpMode {
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();
    positions_and_variables pos = new positions_and_variables();
    boolean sample4detected = false;
    private DcMotorEx shooter = null;
    private DcMotorEx intake =null;




    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(40, 63.5, (Math.toRadians(-90)));
//        PinpointDrive driveRobot = new PinpointDrive(hardwareMap, beginPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // drivetrain motors
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(7);
        limelight.start();
        ElapsedTime LLCorrectionTimer = new ElapsedTime();


//        //score held sample
//        TrajectoryActionBuilder go_score_sample_0 = driveRobot.actionBuilder(beginPose)
//                .strafeToSplineHeading(new Vector2d(58, 58), (Math.toRadians(-135)));
//
//        //go to sample 3
//        TrajectoryActionBuilder go_to_sample_3 = go_score_sample_0.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(50, 49), (Math.toRadians(-90)));
//
//        //return to basket
//        TrajectoryActionBuilder return_basket_3 = go_to_sample_3.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)));
//
//        //go to sample 2
//        TrajectoryActionBuilder go_to_sample_2 = return_basket_3.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(58, 48), Math.toRadians(-86));
//
//        //return to basket
//        TrajectoryActionBuilder return_basket_2 = go_to_sample_2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)));
//
//        //go to sample 1
//        TrajectoryActionBuilder go_to_sample_1 = return_basket_2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(60, 47), (Math.toRadians(-63)));
//
//        //return to basket 1
//        TrajectoryActionBuilder return_basket_1 = go_to_sample_1.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)));
//
//        //go to submersible 4a
//        TrajectoryActionBuilder go_to_sub_4a = return_basket_1.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(30,0), (Math.toRadians(-180)));
//
//        //go to submersible 4b
//        TrajectoryActionBuilder go_to_sub_4b = go_to_sub_4a.endTrajectory().fresh()
//
//                .strafeToLinearHeading(new Vector2d(27,0), (Math.toRadians(-180)));
////
////        //return from submersible 4
////        TrajectoryActionBuilder return_from_sub_4 = go_to_sub_4b.endTrajectory().fresh()
////
////                .strafeToSplineHeading(new Vector2d(40,12), (Math.toRadians(-180)));
//
//        //return to basket 4
//        TrajectoryActionBuilder return_basket_4 = go_to_sub_4a.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(35, 0), (Math.toRadians(-160)))
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)));
//
//        //go to submersible 5a
//        TrajectoryActionBuilder go_to_sub_5a = return_basket_4.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(35,10), (Math.toRadians(0)))
//                .strafeToLinearHeading(new Vector2d(24,10), (Math.toRadians(0)));


        waitForStart();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() <= 0.1 && !isStopRequested()) {
//            boolean switchPressed = !limitSwitch.getState(); // Inverted to show "pressed" as true
//            // Display the state on the telemetry
//            if (switchPressed) {
//                telemetry.addData("Limit Switch", "Pressed");
//            } else {
//                telemetry.addData("Limit Switch", "Not Pressed");
//            }
//            telemetry.update();

//            Actions.runBlocking(new SequentialAction(
//                    new ParallelAction(
//                            go_score_sample_0.build(),
//                            lift.liftUp()
//                    ),
//                    new SleepAction(0.2),
//                    oclaw.OpenOClaw(),
//                    new SleepAction(0.2),
//
//                    // sample 0 cycle completes by now. sample 3 cycle starts below
//                    new ParallelAction(
//                            go_to_sample_3.build(),
//                            oarm.LowerOArm(),
//                            lift.lifttoMiddle(),
//                            intake.SettoVision(),
//                            wrist.SettoWrist_Vision()
//                    )
//                    )
//            );
            //add limelight movement here
            LLCorrectionTimer.reset();
            while (opModeIsActive() && !isStopRequested()) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid() && result.getTa() > 0.001 && LLCorrectionTimer.seconds() <= 1) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        if (result.getTx() < -2) {
                            FL.setPower(-0.5);
                            BR.setPower(-0.5);
                            FR.setPower(0.5);
                            BL.setPower(0.5);
                        } else if (result.getTx() > 2) {
                            FL.setPower(0.5);
                            BR.setPower(0.5);
                            FR.setPower(-0.5);
                            BL.setPower(-0.5);
                        } else {
                            if (result.getTy() > 2) {
                                FL.setPower(0.5);
                                BR.setPower(0.5);
                                FR.setPower(0.5);
                                BL.setPower(0.5);
                            } else if (result.getTy() < -2) {
                                FL.setPower(-0.5);
                                BR.setPower(-0.5);
                                FR.setPower(-0.5);
                                BL.setPower(-0.5);
                            } else {
                                FL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                BL.setPower(0);
                                telemetry.addData("limelight loop 3 breaks", FL.getPower());
                                telemetry.update();
                                break;
                            }
                        }
                    }

//                    //else if (result.isValid() && result.getTa() > 0.001 && LLCorrectionTimer.seconds() > 1) {
//                        Actions.runBlocking(
//                                go_to_sample_3.build()
//                        );
//                        break;
//                    }
                else {
                        break;
                    }
                } else {
                    break;
                }
            };
            //limelight correction done
            FL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
//                    driveRobot.updatePoseEstimate();


        }
    }
}


