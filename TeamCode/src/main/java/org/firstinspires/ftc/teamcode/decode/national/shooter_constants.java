package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Vector;

@Config
public class shooter_constants {
//    public static Pose GOAL_POS_RED = new Pose(138,138);
    Vector2d vector = new Vector2d(2,1);

    public static double getHoodTicksFromDegrees (double degrees){
        return ((-1)/(30-48.805965)*(degrees - 30)) + 0;
    }
    public static double getMotorVelocityFromLaunchSpeed(double launchSpeed){
        return (31.32017 * launchSpeed - 8.39499);
    }
}
