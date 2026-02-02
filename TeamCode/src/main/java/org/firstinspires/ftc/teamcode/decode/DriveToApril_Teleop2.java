package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="MC Drive To AprilTag + TeleOp", group = "Concept")
public class DriveToApril_Teleop2 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 60; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the driveRobot motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.05  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0) pr 0.02
    final double STRAFE_GAIN =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0) pr 0.015
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0) pr 0.01

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front driveRobot wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front driveRobot wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back driveRobot wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back driveRobot wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 24;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    // New variables for enhanced teleop
    private IMU imu;                                // IMU for field-centric driving
    private boolean isFieldCentric = false;         // Drive mode flag
    private boolean slowMode = false;               // Slow mode flag
    private final double SLOW_MODE_FACTOR = 0.5;    // Slow mode power reduction

    @Override
    public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize all four motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "BR");

        // Initialize IMU for field-centric driving
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Set motor directions for mecanum driveRobot
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        if (USE_WEBCAM)
            setManualExposure(1, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData("Drive Mode", "Robot-Centric (Press START for Field-Centric)");
        telemetry.addData("Slow Mode", "OFF (Press DPAD UP to toggle)");
        telemetry.addData("Controls", "LB: Auto to Tag | START/SELECT: Mode | DPAD UP: Slow");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // Check for driveRobot mode changes
            checkDriveModeButtons();

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // Manual driving mode with enhanced features
                if (isFieldCentric) {
                    // Field-Centric driving
                    drive  = -gamepad1.left_stick_y;
                    strafe = -gamepad1.left_stick_x;
                    turn   = -gamepad1.right_stick_x;

                    // Convert to field-centric
                    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                    double robotHeading = orientation.getYaw(AngleUnit.RADIANS);

                    double temp = drive * Math.cos(-robotHeading) - strafe * Math.sin(-robotHeading);
                    strafe = drive * Math.sin(-robotHeading) + strafe * Math.cos(-robotHeading);
                    drive = temp;

                    telemetry.addData("Manual","FIELD-CENTRIC - Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {
                    // Robot-Centric driving (original behavior)
                    drive  = -gamepad1.left_stick_y;
                    strafe = -gamepad1.left_stick_x;
                    turn   = -gamepad1.right_stick_x;
                    telemetry.addData("Manual","ROBOT-CENTRIC - Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }

                // Apply slow mode if enabled
                if (slowMode) {
                    drive *= SLOW_MODE_FACTOR;
                    strafe *= SLOW_MODE_FACTOR;
                    turn *= SLOW_MODE_FACTOR;
                    telemetry.addData("Speed", "SLOW MODE (50%)");
                } else {
                    telemetry.addData("Speed", "NORMAL");
                }
            }

            // Update driveRobot mode and control info
            telemetry.addData("Drive Mode", isFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
            telemetry.addData("Controls", "LB:Auto | START/SELECT:Mode | DPAD UP:Slow");

            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Check for driveRobot mode and slow mode button presses
     */
    private void checkDriveModeButtons() {
        // Toggle field-centric mode with START button
        if (gamepad1.start) {
            isFieldCentric = true;
            // Small delay to prevent multiple toggles
            sleep(200);
        }

        // Toggle robot-centric mode with SELECT button
        if (gamepad1.back) {  // Note: SELECT button is usually mapped to 'back'
            isFieldCentric = false;
            // Small delay to prevent multiple toggles
            sleep(200);
        }

        // Toggle slow mode with DPAD UP
        if (gamepad1.dpad_up) {
            slowMode = !slowMode;
            // Small delay to prevent multiple toggles
            sleep(200);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x - y - yaw;
        double rightFrontPower   =  x + y + yaw;
        double leftBackPower     =  x + y - yaw;
        double rightBackPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}