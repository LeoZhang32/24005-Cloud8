package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp
public class colortest1 extends LinearOpMode {
    color_sensor_hardware cSensors = new color_sensor_hardware();
    String detectedColor;
    ArrayList<Float> a1, a2, b1, b2, a3, b3;
    @Override
    public void runOpMode() throws InterruptedException {
        cSensors.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

//            a = cSensors.get2ADetectedColors();
//            b = cSensors.get2BDetectedColors();
//            if (a == color_sensor_hardware.DetectedColor.PURPLE ||b == color_sensor_hardware.DetectedColor.PURPLE){
//                detectedColor = "PURPLE";
//            }
//            else if(a == color_sensor_hardware.DetectedColor.GREEN ||b == color_sensor_hardware.DetectedColor.GREEN){
//                detectedColor = "GREEN";
//            }
//            else {
//                detectedColor = "UNKNOWN";
//            }
            a1 = cSensors.getA1RGB();
            a2 = cSensors.getA2RGB();
            a3 = cSensors.getA3RGB();
            b1 = cSensors.getB1RGB();
            b2 = cSensors.getB2RGB();
            b3 = cSensors.getB3RGB();

            telemetry.addData("1a RGB",a1);
            telemetry.addData("1b RGB",b1);
            telemetry.addData("2a RGB",a2);
            telemetry.addData("2b RGB",b2);
            telemetry.addData("3a RGB",a3);
            telemetry.addData("3b RGB",b3);
//
//            telemetry.addData("COLOR",detectedColor);
            telemetry.update();
        }
    }
}
