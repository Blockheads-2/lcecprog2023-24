package org.firstinspires.ftc.teamcode.common;

public class Constants {
    //Drive Train Constants
    public static float LOAD_ON = 0.6f; //assumption
    public static double RPM = 312 * LOAD_ON; //690.  Not very accurate so don't rely on this number.
    public static double RPS = RPM / 60.0; //11.5 ish motor revolutions per second, with load

    public static float WHEEL_D = 0f; //wheel diameter
    public static double WHEEL_C = WHEEL_D * Math.PI; //wheel circumference (inches)

    public static float CPR = 537.7f; //537.7 clicks per revolution
    public static double CPI = CPR / WHEEL_C; //... clicks per inch.

    public static float MAX_VELOCITY_DT = 2700f; // unit is clicks/sec; not sure if this is accurate...

    //Distance Between swerve module and Center
    public static float DISTANCE_BETWEEN_MODULE_AND_CENTER = 3.406f; //3.405512

    //Camera Constants
    public static float WIDTH = 1280.0f;
    public static float HEIGHT = 720.0f;
}
