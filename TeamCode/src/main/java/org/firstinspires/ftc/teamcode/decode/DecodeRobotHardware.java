/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.decode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named DecodeRobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */
public class DecodeRobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor FL   = null;
    private DcMotor FR  = null;
    private DcMotor BL   = null;
    private DcMotor BR  = null;
    private IMU imu = null;
    private DcMotorEx shooterTop = null;
    private DcMotorEx shooterBottom = null;
    private DcMotorEx intake =null;
    private CRServo intakeCR = null;
    private Servo trigger = null;
    private Servo gate = null;
    private Servo light = null;
    private DcMotorEx lift = null;
    private NormalizedColorSensor colorSensor;
    private View relativeLayout;

    private double integralSum = 0;
    private double Kp = 0.0325;
    private double Ki = 0;
    private double Kd = 0;
    private double Kf = 0.0032;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    private double targetVelocity = 0;


    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 78; //  this is how close the camera should get to the target (inches) 48

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the driveRobot motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.05  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0) pr 0.02
    final double STRAFE_GAIN =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0) pr 0.015
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0) pr 0.01

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 20;     // Choose the tag you want to approach or set to -1 for ANY tag. 24 FOR RED; 20 FOR BLUE.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag  //  Clip the turn speed to this max value (adjust for your robot)
    double drive, strafe, turn = 0;

    final float[] hsvValues = new float[3];

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public DecodeRobotHardware (LinearOpMode opmode) { myOpMode = opmode; }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
// Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
        if (USE_WEBCAM){
            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                myOpMode.telemetry.addData("Camera", "Waiting");
                myOpMode.telemetry.update();
                while (!myOpMode.isStopRequested() &&(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    myOpMode.sleep(20);
                }
                myOpMode.telemetry.addData("Camera", "Ready");
                myOpMode.telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!myOpMode.isStopRequested()) {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    myOpMode.sleep(50);
                }
                exposureControl.setExposure((long) 1, TimeUnit.MILLISECONDS);
                myOpMode.sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(250);
                myOpMode.sleep(20);
            }
        }

        if (visionPortal.getCameraState() != null) {
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    myOpMode.sleep(20);
                }
            }
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            myOpMode.sleep(20);
        }
        // Wait for driver to press start
        myOpMode.telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        myOpMode.telemetry.addData("Controls", "LB: Auto to Tag | START/SELECT: Mode | DPAD UP: Slow");
        myOpMode.telemetry.addData(">", "Touch Play to start OpMode");
        myOpMode.telemetry.update();

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        FL = myOpMode.hardwareMap.get(DcMotor.class, "FL");
        FR = myOpMode.hardwareMap.get(DcMotor.class, "FR");
        BL = myOpMode.hardwareMap.get(DcMotor.class, "BL");
        BR = myOpMode.hardwareMap.get(DcMotor.class, "BR");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");
        shooterTop = myOpMode.hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterBottom = myOpMode.hardwareMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeCR = myOpMode.hardwareMap.get(CRServo.class, "intakeCR");
        intakeCR.setDirection(CRServo.Direction.REVERSE);

        trigger = myOpMode.hardwareMap.get(Servo.class, "trigger");
        gate = myOpMode.hardwareMap.get(Servo.class, "gate");
        light = myOpMode.hardwareMap.get(Servo.class, "light");
        light.setPosition(0);

        lift = myOpMode.hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        int relativeLayoutId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", myOpMode.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) myOpMode.hardwareMap.appContext).findViewById(relativeLayoutId);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive_y    y-direction power (-1.0 to 1.0)
     * @param Drive_x    x-direction power (-1.0 to 1.0)
     * @param Turn       Right/Left turning power (-1.0 to 1.0)
     * @param SlowModeOn slow mode boolean
     */
    public void driveRobot(double Drive_y, double Drive_x, double Turn, boolean SlowModeOn, boolean ImuReset) {
        double BotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (ImuReset){
            imu.resetYaw();
        }
        // Rotate the movement direction counter to the bot's rotation
        double rotX = Drive_x * Math.cos(-BotHeading) - Drive_y * Math.sin(-BotHeading);
        double rotY = Drive_x * Math.sin(-BotHeading) + Drive_y * Math.cos(-BotHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(Turn), 1);
        double frontLeftPower = (rotY + rotX + Turn) / denominator;
        double backLeftPower = (rotY - rotX + Turn) / denominator;
        double frontRightPower = (rotY - rotX - Turn) / denominator;
        double backRightPower = (rotY + rotX - Turn) / denominator;

        if (SlowModeOn){
            FL.setPower(frontLeftPower * 0.5);
            BL.setPower(backLeftPower * 0.5);
            FR.setPower(frontRightPower * 0.5);
            BR.setPower(backRightPower * 0.5);
        }
        else{
            FL.setPower(frontLeftPower * 1);
            BL.setPower(backLeftPower * 1);
            FR.setPower(frontRightPower * 1);
            BR.setPower(backRightPower * 1);
        }
    }

    /** @param autoInput button to activate auto
     *  @param desiredTagID Tag ID
     **/
    public void driveToApril (boolean autoInput, double desiredTagID){
        boolean targetFound = false;
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
                    myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            myOpMode.telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            myOpMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            myOpMode.telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            myOpMode.telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            myOpMode.telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            myOpMode.telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (autoInput && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double leftFrontPower    =  drive - strafe - turn;
            double rightFrontPower   =  drive + strafe + turn;
            double leftBackPower     =  drive + strafe - turn;
            double rightBackPower    =  drive - strafe + turn;

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
            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);
            myOpMode.telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }

    /**
     *
     * @param intakeInInput -intake button
     * @param intakeOutInput -reverse intake button
     * @param intakeCRIn - intake servo in button
     * @param outtakeClose -outtake close button
     * @param outtakeFar - outake far button
     * @param outtakeMid - outtake mid
     */
    public void intakeOuttakeAction(boolean intakeInInput, boolean intakeCRIn, boolean intakeOutInput, boolean outtakeClose, boolean outtakeFar, boolean outtakeMid){
        double shooterVelocity = 0;
        shooterVelocity = shooterTop.getVelocity(AngleUnit.DEGREES);
        myOpMode.telemetry.addData("velocity",shooterVelocity);
        myOpMode.telemetry.update();

        float gain = 2;
        String lightcolor;
        boolean velocityValid = false;
        boolean velocityValid2 = false;

        double outtakePower = PIDControl(targetVelocity, shooterTop.getVelocity(AngleUnit.DEGREES));
        if (outtakeClose){
            if (outtakeMid){
                velocityValid = shooterVelocity >= 153;
                targetVelocity = 163;
                shooterTop.setPower(outtakePower);
                shooterBottom.setPower(outtakePower);
            }
            else {
                velocityValid = shooterVelocity >= 145;
                targetVelocity = 155;
                shooterTop.setPower(outtakePower);
                shooterBottom.setPower(outtakePower);
            }
        }
        else if (outtakeFar){
            velocityValid = shooterVelocity >= 178;
            targetVelocity = 188;
            shooterTop.setPower(outtakePower);
            shooterBottom.setPower(outtakePower);
        }
        else {
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        }



        if (velocityValid) {
            trigger.setPosition(0.68);
            light.setPosition(0.277); //red color
            lightcolor = "red";
        }
        else if (hsvValues[0] >= 120 && hsvValues[0] <= 180) {
            lightcolor = "green";
            trigger.setPosition(0.95);
            light.setPosition(0.5);
        }
        else if (hsvValues[0] >= 200 && hsvValues[0] <= 280) {
            lightcolor = "purple";
            trigger.setPosition(0.95);
            light.setPosition(0.722);
        }
        else {
            lightcolor = "null";
            trigger.setPosition(0.95);
            light.setPosition(0);
        }

        if (intakeOutInput){
            intake.setPower(-1);
            intakeCR.setPower(-1);
        }
        else {
            if (intakeCRIn){
                intakeCR.setPower(1);
            }
            else {
                intakeCR.setPower(0);
            }
            if (intakeInInput){
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }
        }

        // Explain basic gain information via telemetry
        // Show the gain value via telemetry
        myOpMode.telemetry.addData("Gain", gain);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        myOpMode.telemetry.addData("Alpha", "%.3f", colors.alpha);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (colorSensor instanceof DistanceSensor) {
            myOpMode.telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }


        myOpMode.telemetry.addData("light emitted", lightcolor);
        myOpMode.telemetry.update();
        // Change the Robot Controller's background color to match the color detected by the color sensor.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });

    }
    /***
     * @param liftUpInput input for upwards
     * @param liftDownInput input for downwards
     *
     * */
    public void liftAction(boolean liftUpInput, boolean liftDownInput){

    if (liftUpInput){
            if (lift.getCurrentPosition() > 2000) {
                lift.setPower(0);
                myOpMode.telemetry.addData("lift", "over limit");
                myOpMode.telemetry.update();
            }
            else {
//                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(1);
            }

        }
        else {
            lift.setPower(0);
        }
        myOpMode.telemetry.addData("lift encoder", lift.getCurrentPosition());
        myOpMode.telemetry.update();
    }

    public double PIDControl (double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}


