package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Here, store the names for each of the motors in the hardware map,
 * store the pid constants for each of the loops,
 * and store any other constants that are used in the code.
 */
public class Constants
{
    public static String leftbackMotor = "lbmotor1";
    public static String rightbackMotor = "rbmotor3";
    public static String leftfrontMotor = "lfmotor0";
    public static String rightfrontMotor = "rfmotor2";

    public static String imu = "imu";
    public static String frontWebcamera = "Webcam 1";



    public static double TICKS_TO_INCHES = 1;
    
    public static String motorPositionFolder = "/MOTORS";
    public static String teamConfigFolder = "/MYTEAM";
    public static String cameraConfigFolder = "/CAMERA";

    public static int CameraViewWidth = 640;
    public static int CameraViewHeight = 360;
    public static String frontColorSensor = null;
    public static String armMotor = "armmotor0";
    public static double armRightUp = 1400;
    public static final int ARM_POSITION_HIGHEST = 1000;
    public static final int ARM_POSITION_LOWEST = 200;
}
