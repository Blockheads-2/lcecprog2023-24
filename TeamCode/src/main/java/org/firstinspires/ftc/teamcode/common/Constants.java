package org.firstinspires.ftc.teamcode.common;

public class Constants {
    //Drive Train Constants
    public static double LOAD_ON = 0.6; //assumption
    public static double RPM = 312 * LOAD_ON; //690.  Not very accurate so don't rely on this number.
    public static double RPS = RPM / 60.0; //11.5 ish motor revolutions per second, with load

    public static double WHEEL_D = 0; //wheel diameter
    public static double WHEEL_C = WHEEL_D * Math.PI; //wheel circumference (inches)

    public static double CPR = 537.7; //537.7 clicks per revolution
    public static double CPI = CPR / WHEEL_C; //... clicks per inch.

    public static double MAX_VELOCITY_DT = 2700; // unit is clicks/sec; not sure if this is accurate...

    //Distance Between swerve module and Center
    public static double DISTANCE_BETWEEN_MODULE_AND_CENTER = 3.406; //3.405512
}
