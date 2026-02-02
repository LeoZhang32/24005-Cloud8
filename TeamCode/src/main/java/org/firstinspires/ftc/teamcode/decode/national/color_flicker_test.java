package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.national.hardware.color_sensor_hardware;

@TeleOp
public class color_flicker_test extends LinearOpMode {
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickerCount = 0;
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    color_sensor_hardware cSensors = new color_sensor_hardware();
    String detectedColor;
    Enum color1;
    Boolean held1 = false;
    Enum color2;
    Boolean held2 = false;
    Enum color3;
    Boolean held3 = false;
    int[] flickOrder = new int[3];
    @Override
    public void runOpMode() throws InterruptedException {
        cSensors.init(hardwareMap);
        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");

        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            color1 = cSensors.get1FinalColor();
            if (color1 != color_sensor_hardware.DetectedColor.UNKNOWN)held1 = true;
            else held1 = false;

            color2 = cSensors.get2FinalColor();
            if (color1 != color_sensor_hardware.DetectedColor.UNKNOWN)held2 = true;
            else held2 = false;

            color3 = cSensors.get3FinalColor();
            if (color1 != color_sensor_hardware.DetectedColor.UNKNOWN)held3 = true;
            else held3 = false;

            if (held1){
                flickOrder[0] = 1;
            }
            else flickOrder[0] = 0;
            if (held2){
                flickOrder[1] = 2;
            }
            else flickOrder[1] = 0;
            if (held3){
                flickOrder[2] = 3;
            }
            else flickOrder[2] = 0;

            if (gamepad1.a){
                if (nextTimer.seconds() >= 2 && flickOrder[flickerCount] == 0){
                    flickerTimer.reset();
                    nextTimer.reset();
                    flickerCount += 1;
                }
                if (flickerCount == 3){
                    flickerCount = 0;
                }
                if (flickOrder[flickerCount] == 1){
                    if (flickerTimer.seconds() <= 1){
                        flicker1.setPosition(0.95);
                    }
                    else flicker1.setPosition(0.65);
                }
                else if (flickOrder[flickerCount] == 2){
                    if (flickerTimer.seconds() <= 1){
                        flicker2.setPosition(0.55);
                    }
                    else flicker2.setPosition(0.85);
                }
                else if (flickOrder[flickerCount] == 3){
                    if (flickerTimer.seconds() <= 1){
                        flicker3.setPosition(0.46);
                    }
                    else flicker3.setPosition(0.76);
                }
            }
            else {
                flicker1.setPosition(0.65);
                flicker2.setPosition(0.85);
                flicker3.setPosition(0.76);

            }
            telemetry.addData("nexttimer:", nextTimer.seconds());
            telemetry.update();
        }
        //TODO: add other color sensors and write flicker logic with array?
    }
}
