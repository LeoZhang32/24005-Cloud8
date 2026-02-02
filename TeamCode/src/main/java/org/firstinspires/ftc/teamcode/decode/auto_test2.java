package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;

import java.util.List;

@Disabled
@Autonomous (name = "auto_test2")

public final class auto_test2 extends LinearOpMode {
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();

    boolean targetAligned = false;

        public class Intake {
        private final DcMotorEx intake;
        private CRServo intakeCR;

            public Intake (HardwareMap hardwareMap) {
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intakeCR = hardwareMap.get(CRServo.class, "intakeCR");
                intakeCR.setDirection(CRServo.Direction.REVERSE);
            }

        //IntakeRun Function
            public class IntakeRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setPower(1);
                    intakeCR.setPower(1);
                    return false;
                }
            }
            public Action IntakeRun() {
                return new Intake.IntakeRun();
            }

            //IntakeCR Function
            public class IntakeCRRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setPower(0);
                    intakeCR.setPower(1);
                    return false;
                }
            }
            public Action IntakeCRRun() {
                return new Intake.IntakeCRRun();
            }


            //IntakeStop Function
            public class IntakeStop implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setPower(0);
                    intakeCR.setPower(0);
                    return false;
                }
            }
            public Action IntakeStop() {
                return new Intake.IntakeStop();
            }


        }




        public class Outtake {
            private final DcMotorEx shooterTop;
            private final DcMotorEx shooterBottom;
            private final DcMotorEx intake;
            private Servo trigger;
            private CRServo intakeCR;
            public Outtake (HardwareMap hardwareMap) {

                shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
                shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
                shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intakeCR = hardwareMap.get(CRServo.class, "intakeCR");
                trigger = hardwareMap.get(Servo.class, "trigger");

            }

            //OuttakeRun Function
            public class OuttakeRun implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                    if (!initialized) {
                        shooterTop.setPower(1);
                        shooterBottom.setPower(1);
                        initialized = true;
                    }

                    double shooterVelocity = 0;
                    shooterVelocity = shooterTop.getVelocity(AngleUnit.DEGREES);
                    telemetry.addData("velocity", shooterVelocity);
                    telemetry.update();

                    if (shooterVelocity >= 140) {
                        trigger.setPosition(0.68);
                        intakeCR.setPower(1);
                        return true;
                    } else if (shooterVelocity >= 70 && shooterVelocity < 140){
                        trigger.setPosition(0.95);
                        intakeCR.setPower(1);
                        intake.setPower(1);
                        return true;
                    } else {
                        trigger.setPosition(0.95);
                        intakeCR.setPower(0);
                        intake.setPower(0);
                        return false;
                    }

                }
            }
            public Action OuttakeRun() {
                return new Outtake.OuttakeRun();
            }



            //OuttakeStop Function
            public class OuttakeStop implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    shooterTop.setPower(0);
                    shooterBottom.setPower(0);
                    trigger.setPosition(0.95);
                    return false;
                }
            }
            public Action OuttakeStop() {
                return new Outtake.OuttakeStop();
            }




        }


    //trigger servo class
    public class Trigger {
        private final Servo trigger;

        public Trigger (HardwareMap hardwareMap) {
            trigger = hardwareMap.get(Servo.class, "trigger");
        }


        public class Trigger_Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                trigger.setPosition(0.65);
                return false;
            }
        }

        public Action OpenTrigger() {
            return new Trigger.Trigger_Open();
        }


        public class Trigger_Closed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                trigger.setPosition(0.95);
                return false;
            }
        }

        public Action CloseTrigger() {
            return new Trigger.Trigger_Closed();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {




        Pose2d beginPose = new Pose2d(-63, 39, (Math.toRadians(180)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);
        Actions.runBlocking(trigger.CloseTrigger());
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
        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(7);
        limelight.start();
        ElapsedTime LLCorrectionTimer = new ElapsedTime();
        ElapsedTime ShooterTimer = new ElapsedTime();


        //score held artifacts
        TrajectoryActionBuilder go_shoot_held_artifacts = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-15, 15), (Math.toRadians(145)));

        //go scan obelisk
        TrajectoryActionBuilder go_scan_obelisk = go_shoot_held_artifacts.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 12), (Math.toRadians(-170)));


        //After Scan, if April Tag shows PPG, then follow this sequence:
        //PPG - PGP

        // go to PPG
        TrajectoryActionBuilder go_from_obelisk_to_PPG = go_scan_obelisk.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 24), (Math.toRadians(-90)));

        //go collect PPG
        TrajectoryActionBuilder go_collect_PPG = go_from_obelisk_to_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 48), Math.toRadians(-90));

        //go shoot PPG
        TrajectoryActionBuilder go_shoot_PPG = go_collect_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-15, 15), (Math.toRadians(145)));

        //go to PGP
        TrajectoryActionBuilder go_from_shoot_to_PGP = go_shoot_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 24), (Math.toRadians(-90)));

        //go collect PGP
        TrajectoryActionBuilder go_collect_PGP = go_from_shoot_to_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 48), Math.toRadians(-90));

        //go shoot PGP
        TrajectoryActionBuilder go_shoot_PGP = go_collect_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-15, 15), (Math.toRadians(145)));

        //LEAVE
        TrajectoryActionBuilder go_leave = go_shoot_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, 48), (Math.toRadians(180)));


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
            ShooterTimer.reset();

            Actions.runBlocking(new SequentialAction(
                    go_shoot_held_artifacts.build()
                    )
            );

// add webcam 1 action here
// April Tag alignment for more accurate shooting

            Actions.runBlocking(new SequentialAction(
                    new SequentialAction(
                            outtake.OuttakeRun(),
                            new SleepAction(5),
                            outtake.OuttakeStop()
                    ),
                    go_scan_obelisk.build()
                    )
            );

// add webcam 1 action here
// April Tag recognition, add THREE options here
// Below is PPG Option (closest to Goal)

            Actions.runBlocking(new SequentialAction(
                    go_from_obelisk_to_PPG.build()
                    )
            );

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
            drive.updatePoseEstimate();

            Actions.runBlocking(new SequentialAction(

                    new ParallelAction(

                        go_collect_PPG.build(),
                        new SequentialAction(
                            intake.IntakeRun(),
                            new SleepAction(1.5),
                            intake.IntakeStop()
                        )
                    ),

                    go_shoot_PPG.build()
            )
            );

// add webcam 1 action here
// April Tag alignment for more accurate shooting


            ShooterTimer.reset();
            Actions.runBlocking(new SequentialAction(

                    new SequentialAction(
                            outtake.OuttakeRun(),
                            new SleepAction(5),
                            outtake.OuttakeStop()
                            ),

                    go_from_shoot_to_PGP.build()
            )
            );


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
            drive.updatePoseEstimate();

            Actions.runBlocking(new SequentialAction(

                            new ParallelAction(

                                    go_collect_PGP.build(),
                                    new SequentialAction(
                                            intake.IntakeRun(),
                                            new SleepAction(1.5),
                                            intake.IntakeStop()
                                    )
                            ),

                            go_shoot_PGP.build()
                    )
            );

// add webcam 1 action here
// April Tag alignment for more accurate shooting


            ShooterTimer.reset();
            Actions.runBlocking(new SequentialAction(

                            new SequentialAction(
                                    outtake.OuttakeRun(),
                                    new SleepAction(5),
                                    outtake.OuttakeStop()
                            ),

                            go_leave.build()
                    )
            );


        }
    }
}


