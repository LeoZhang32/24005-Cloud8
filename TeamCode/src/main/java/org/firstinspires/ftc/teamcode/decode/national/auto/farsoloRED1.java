package org.firstinspires.ftc.teamcode.decode.national.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.national.hardware.color_sensor_hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Watchable;
import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;

@Autonomous(name = "AAA FAR RED")
public class farsoloRED1 extends CommandOpMode {
    private Follower follower;

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;
    DcMotorEx turretEncoder;
    DcMotorEx intake;
    CRServo turret;
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    double home1 = 0.96;
    double home2 = 0.03;
    double home3 = 0.175;
    double score1 = 0.66;
    double score2 = 0.33;
    double score3 = 0.475;
    boolean capacityChecked = false;
    boolean shootingFinished = false;
    boolean motifChecked = false;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickCounter = 0;
    Servo hood;
    Limelight3A limelight;
    int tagID;
    int patternIndex = 0;
    int greenIndex = 0;
    int purpleIndex = 0;
    boolean leaveActivated = false;
    ArrayList<farsoloRED1.Flicker> flickOrder = new ArrayList<>();
    ArrayList<Flicker> purple = new ArrayList<>();
    ArrayList<Flicker> green = new ArrayList<>();
    color_sensor_hardware cSensors = new color_sensor_hardware();
    private final Pose startPose = new Pose(81.25, 8.7, Math.toRadians(0));
    private final Pose scorePose = new Pose (85.2, 16.7, Math.toRadians(0));
    private final Pose intakeDiagonalPose = new Pose (129, 11, Math.toRadians(345));
    private final Pose intakeHorizPose = new Pose (132, 10, Math.toRadians(0));
    private final Pose intakePickup3 = new Pose (131, 35.4, Math.toRadians(0));


    private PathChain scorePreload, goIntakeDiag, continueIntakeHoriz, scoreIntaked, goIntake3, scorePickup3;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();
        goIntakeDiag = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,intakeDiagonalPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),intakeDiagonalPose.getHeading())
                .build();
        continueIntakeHoriz = follower.pathBuilder()
                .addPath(new BezierLine(intakeDiagonalPose,intakeHorizPose))
                .setLinearHeadingInterpolation(intakeDiagonalPose.getHeading(),intakeHorizPose.getHeading())
                .build();
        scoreIntaked = follower.pathBuilder()
                .addPath(new BezierLine(intakeHorizPose, scorePose))
                .setLinearHeadingInterpolation(intakeHorizPose.getHeading(),scorePose.getHeading())
                .build();
        goIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(100.9, 37.6), intakePickup3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakePickup3.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePickup3,scorePose))
                .setLinearHeadingInterpolation(intakePickup3.getHeading(),scorePose.getHeading())
                .build();
    }
    private RunCommand spinShooter(double targetVel) {
        return new RunCommand(() -> {
            double power = PIDControlShooter(targetVel, shooterTop.getVelocity(AngleUnit.DEGREES),0.0325,0.00325);
            shooterTop.setPower(power);
            shooterBottom.setPower(power);
        });
    }
    private InstantCommand stopShooter() {
        return new InstantCommand(() -> {
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        });
    }
    private RunCommand moveTurret(double targetDegrees) {
        return new RunCommand(() -> {
            double power = PIDControlTurret(targetDegrees, (double) (turretEncoder.getCurrentPosition() * 360) / (4 * 8192),0.06);
            turret.setPower(power);
        });
    }
    private InstantCommand moveHood (double hoodPos) {
        return new InstantCommand(()->hood.setPosition(hoodPos));
    }
    private RunCommand scoreArtifacts(double nextTime, IntSupplier motif) {
        return new RunCommand(() -> {
            int motifID = motif.getAsInt();
            Flicker f1 = new Flicker(flicker1, home1, score1);
            Flicker f2 = new Flicker(flicker2, home2, score2);
            Flicker f3 = new Flicker(flicker3, home3, score3);
            Enum<color_sensor_hardware.DetectedColor> color1 = cSensors.get1FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color2 = cSensors.get2FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color3 = cSensors.get3FinalColor();

            color_sensor_hardware.DetectedColor[] motifGPP = {
                    color_sensor_hardware.DetectedColor.GREEN,
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.PURPLE
            };
            color_sensor_hardware.DetectedColor[] motifPGP = {
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.GREEN,
                    color_sensor_hardware.DetectedColor.PURPLE
            };
            color_sensor_hardware.DetectedColor[] motifPPG = {
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.GREEN
            };
            if (!capacityChecked) {
                purple.clear();
                green.clear();
                if (color1 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f1);
                if (color1 == color_sensor_hardware.DetectedColor.GREEN) green.add(f1);
                if (color2 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f2);
                if (color2 == color_sensor_hardware.DetectedColor.GREEN) green.add(f2);
                if (color3 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f3);
                if (color3 == color_sensor_hardware.DetectedColor.GREEN) green.add(f3);
                patternIndex = 0;
                greenIndex = 0;
                purpleIndex = 0;
                flickerTimer.reset();
                nextTimer.reset();
                shootingFinished = false;
                capacityChecked = true;
            }
            if (motifID == 21) {
                if (patternIndex < motifGPP.length) {

                    color_sensor_hardware.DetectedColor wanted = motifGPP[patternIndex];
                    Flicker current = null;

                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }


            }
            if (motifID == 22) {
                if (patternIndex < motifPGP.length) {

                    color_sensor_hardware.DetectedColor wanted = motifPGP[patternIndex];
                    Flicker current = null;
                    if (green.isEmpty() && purple.isEmpty()){
                        shootingFinished = true;
                    }
                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }

            }
            if (motifID == 23) {
                if (patternIndex < motifPPG.length) {

                    color_sensor_hardware.DetectedColor wanted = motifPPG[patternIndex];
                    Flicker current = null;
                    if (green.isEmpty() && purple.isEmpty()){
                        shootingFinished = true;
                    }

                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }
            }
            if (motifID != 21 && motifID != 22 && motifID != 23){
                if (patternIndex < motifGPP.length) {

                    color_sensor_hardware.DetectedColor wanted = motifGPP[patternIndex];
                    Flicker current = null;
                    if (green.isEmpty() && purple.isEmpty()){
                        shootingFinished = true;
                    }

                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }
            }
        });
    }
    private void runFlicker(Flicker f) {
        if (flickerTimer.seconds() <= 0.3) {
            f.goScore();
        } else {
            f.goHome();
        }
    }
    private InstantCommand resetFlickers() {
        return new InstantCommand(() -> {
            shootingFinished = false;
            capacityChecked = false;
            flicker1.setPosition(home1);
            flicker2.setPosition(home2);
            flicker3.setPosition(home3);
            flickerTimer.reset();
            nextTimer.reset();
        });
    }
    private InstantCommand stopTurret() {
        return new InstantCommand(() -> turret.setPower(0));
    }

    private InstantCommand spinIntake() {
        return new InstantCommand(() -> {
            Enum<color_sensor_hardware.DetectedColor> color1 = cSensors.get1FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color2 = cSensors.get2FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color3 = cSensors.get3FinalColor();
            boolean detect1 = (color1 != color_sensor_hardware.DetectedColor.UNKNOWN);
            boolean detect2 = (color2 != color_sensor_hardware.DetectedColor.UNKNOWN);
            boolean detect3 = (color3 != color_sensor_hardware.DetectedColor.UNKNOWN);
            boolean isFull = (detect1 && detect2 && detect3);
            if (isFull) intake.setPower(-1);
            else intake.setPower(1);
        });
    }
    private InstantCommand stopIntake() {
        return new InstantCommand(() -> intake.setPower(0));
    }
    private RunCommand checkMotif(){
        return new RunCommand(() -> {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                if (!fiducials.isEmpty()&&!motifChecked) {
                    tagID = fiducials.get(0).getFiducialId();
                    motifChecked = true;
                }
            }
            else telemetry.addData("target", "INVALID");
        });
    }
    private InstantCommand activateLeave(){
        return new InstantCommand(()->{
            leaveActivated = true;
        });
    }
    private double PIDControlShooter(double reference, double state, double Kp, double Kf){
        double error = reference - state;

        return (error * Kp) + (reference * Kf);
    }
    private double PIDControlTurret(double reference, double state, double Kp) {
        double error = reference - state;
        double minimum = 0;
        double maximum = 1;
        double output = (error * Kp);
        if (Math.abs(output) < minimum) {
            output = 0;
        }
        if (output > maximum) output = maximum;
        if (output < -maximum) output = -maximum;
        return output;
    }

    @Override
    public void initialize() {
        super.reset();

        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        turretEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        turretEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret = hardwareMap.get(CRServo.class, "turret");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        cSensors.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(1);
        limelight.start();
        hood = hardwareMap.get(Servo.class, "hood");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                moveHood(0.4),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        moveTurret(-100).withTimeout(1000),
                                        checkMotif().withTimeout(1000)
                                ),
                                moveTurret(-67.5).interruptOn(()-> shootingFinished)
                        ),
                        new FollowPathCommand(follower, scorePreload),
                        spinShooter(185).interruptOn(() -> shootingFinished),
                        new SequentialCommandGroup(
                                new WaitCommand(3000),
                                scoreArtifacts(1.2, ()->tagID).interruptOn(() -> shootingFinished)
                        )
                ),
                resetFlickers(),
                stopShooter(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, goIntake3),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                spinIntake()
                                        )
                                ),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                stopIntake()
                                        ),
                                        new FollowPathCommand(follower, scorePickup3),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                spinShooter(185).interruptOn(() -> shootingFinished)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2750),
                                                scoreArtifacts(1.2, ()-> tagID).interruptOn(() -> shootingFinished)
                                        )
                                ),
                                resetFlickers(),
                                stopShooter(),
                                new FollowPathCommand(follower, goIntakeDiag),
                                spinIntake(),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, continueIntakeHoriz),
                                new WaitCommand(1500),
                                stopIntake(),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, scoreIntaked),
                                        new SequentialCommandGroup(
                                                new WaitCommand(750),
                                                spinShooter(185).interruptOn(() -> shootingFinished)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2500),
                                                scoreArtifacts(1.2, ()-> tagID).interruptOn(() -> shootingFinished)
                                        )
                                ),
                                resetFlickers(),
                                stopShooter()
                        ),
                        moveTurret(-67.5).withTimeout(30000)
                )
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        follower.update();
        super.run();
        telemetry.addData("green size", green.size());
        telemetry.addData("purple size", purple.size());
        telemetry.addData("shooterVel", shooterTop.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shootingFinished", shootingFinished);
        telemetry.addData("flickCounter", flickCounter);
        telemetry.addData("capacity checked", capacityChecked);
        telemetry.addData("turretPos", (double) (turretEncoder.getCurrentPosition() * 360) / (4 * 8192));
        telemetry.addData("TAG ID", tagID);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    static class Flicker {
        Servo servo;
        double home, score;
        Flicker(Servo servo, double home, double score){
            this.servo = servo;
            this.home = home;
            this.score = score;
        }
        void goHome(){
            servo.setPosition(home);
        }
        void goScore(){
            servo.setPosition(score);
        }
    }
}
