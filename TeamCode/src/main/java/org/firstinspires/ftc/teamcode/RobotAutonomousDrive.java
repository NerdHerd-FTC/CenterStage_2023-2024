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

package org.firstinspires.ftc.teamcode;

//import static android.os.SystemClock.sleep;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.core.subsystems.EyeAll;
import org.firstinspires.ftc.teamcode.util.AllianceConfig;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Game Robot Autonomous", group="Robot") // Drive
//@Disabled
public class RobotAutonomousDrive extends LinearOpMode
{
    /* Declare OpMode members. */
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    //private BNO055IMU imu;
    private IMU imu         = null;      // Control/Expansion Hub IMU
    
    EyeAll eye = null;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    
    private final ElapsedTime autoTimer30Sec = new ElapsedTime();
    
    // Eye needs to access it, public!
    public static AllianceConfig.AllianceConfigData allianceConfig =
            new AllianceConfig.AllianceConfigData();

    //Four (4) white Pixels, one (1) for each set of Spike Marks. The Pixels will start centered on
    //top of the center Spike Marks
    private EyeAll.ObjectLocation propsLocation = EyeAll.ObjectLocation.UNKNOWN;


    @Override
    public void runOpMode() throws InterruptedException {
        autoTimer30Sec.reset();

        eye = new EyeAll(hardwareMap); // comment it out if not camera installed

        if(eye!= null)
        {
            eye.autoInit();
            propsLocation = EyeAll.ObjectLocation.UNKNOWN;
        }
        else
        {
            propsLocation = EyeAll.ObjectLocation.CENTER; // testing
        }

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, Constants.leftfrontMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.rightfrontMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.leftbackMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.rightbackMotor);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        // define initialization values for IMU, and then initialize it.
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        //imu = hardwareMap.get(BNO055IMU.class, Constants.imu);
        //imu.initialize(parameters);
        imu = hardwareMap.get(IMU.class, "imu");
        // define initialization values for IMU, and then initialize it.
        // The next three lines define the desired axis rotations.
        // To Do: EDIT these values to match YOUR mounting configuration.
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Reset Yaw
        imu.resetYaw();

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetHeading();

        AllianceConfig.ReadConfigFromFile(allianceConfig);

        //eye.OpenEyeToRead();
        if(eye!= null)
            eye.OpenEyeToFindProps();

        //TuningHandMotors(gamepad1); // TODO debug
        //lifterMotorRunnable();
        //rotatorMotorRunnable();
        //armMotorRunnable();
        telemetry.addData("Software Version: ",
                BuildConfig.BUILD_TIME);
        telemetry.addData("TEAM #: ",
                allianceConfig.TeamNumber);
        telemetry.addData("ALLIANCE : ",
                allianceConfig.Alliance);
        telemetry.addData("STARTING LOCATION : ",
                allianceConfig.Location);
        telemetry.addData("Route #: ",
                allianceConfig.PathRoute);

        composeTelemetry();
        telemetry.update();

        waitForStart();
        setMissionTo(Mission.SPOT_A);

        while(opModeIsActive())
        {
            loopTasks();

            composeTelemetry();
            telemetry.update();

            sleep(10);  // Pause to display last telemetry message.
        }
    }


    boolean waitabit = false;
    ElapsedTime eyeruntime = new ElapsedTime();
    private void EyeOp()
    {
        if(waitabit)
        {
            if(eyeruntime.milliseconds() > 500)
            {
                waitabit = false;
            }
            else
            {
                return;
            }
        }

        //eye.UpdateConfigDataLoop(config);
        //eye.SetOpInfoOnScreen(CurrentTask.toString());

        if(eye!= null) {
            if (allianceConfig.Alliance.equals(AllianceConfig.RED))
                propsLocation = eye.CheckPropsLocation(EyeAll.TargetObject.RED_CONE);
            else
                propsLocation = eye.CheckPropsLocation(EyeAll.TargetObject.BLUE_CONE);

            if (propsLocation != EyeAll.ObjectLocation.WAITING && propsLocation != EyeAll.ObjectLocation.UNKNOWN) {
                waitabit = true;
                eyeruntime.reset();
            }
        }
    }
    
    // SPOT is loop-able function, MOVE is onetime execution function
    private enum Mission
    {
        SPOT_A,
        MOVE_A2B,
        SPOT_B,

        EXIT
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    
    int missionStepTimeout = 10; // TODO debug
    public void loopTasks()
    {
        switch (currentMission)
        {
            case SPOT_A:
                // find self init position, read parking zone picture, drop preload to low junction
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                spotA();
                break;
            case SPOT_B:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                spotB(-520, false);
                break;
            case MOVE_A2B:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                if (allianceConfig.Alliance.equals(AllianceConfig.RED) )
                {
                    if(allianceConfig.Location.equals(AllianceConfig.LEFT) )
                    {
                        if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_LEFT)
                        {
                            RedLeft_Left();
                        }
                        else if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_RIGHT)
                        {
                            RedLeft_Right();
                        }
                        else
                        {
                            RedLeft_Center();
                        }
                    }
                    else //if (allianceConfig.Location.equals(AllianceConfig.RIGHT) ) RED
                    {
                        if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_LEFT)
                        {
                            RedRight_Left();
                        }
                        else if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_RIGHT)
                        {
                            RedRight_Right();
                        }
                        else
                        {
                            RedRight_Center();
                        }
                    }
                }
                else //if (allianceConfig.Alliance.equals(AllianceConfig.BLUE) )
                {
                    if(allianceConfig.Location.equals(AllianceConfig.LEFT) )
                    {
                        if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_LEFT)
                        {
                            RedRight_Left();
                        }
                        else if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_RIGHT)
                        {
                            RedRight_Left();
                        }
                        else
                        {
                            RedRight_Left();
                        }
                    }
                    else //if (allianceConfig.Location.equals(AllianceConfig.RIGHT) ) BLUE
                    {
                        if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_LEFT)
                        {
                            RedRight_Left();
                        }
                        else if( propsLocation == EyeAll.ObjectLocation.ON_CAMERA_RIGHT)
                        {
                            RedRight_Left();
                        }
                        else
                        {
                            RedRight_Left();
                        }
                    }

                }
                break;

            default: //EXIT
                break;
        }
        
//        lifterMotorRunnable();
//        rotatorMotorRunnable();
//        armMotorRunnable();
    
        addLog();
    
        //TuningHandMotors(gamepad1); // TODO debug
    }
    
    //EyeAll.ObjectLocation isCenterPole = EyeAll.ObjectLocation.UNKNOWN;
    
    
    private Mission currentMission = Mission.SPOT_A; // debug: TODO
    private Mission previousMission = Mission.SPOT_A;
    ElapsedTime missionRunTimer = new ElapsedTime();
    private void setMissionTo(Mission newMission)
    {
        // debug: TODO
//        if(newMission == Mission.MOVE_A2B)
//        {
//            currentMission = Mission.EXIT;
//            missionStepTimeout = 1000;
//            return;
//        }
        // end debug
    
        previousMission = currentMission;
        currentMission = newMission;
        missionRunTimer.reset();
        setTaskTo(0);
    }
    private int currentTaskID = 0;
    private int previousTaskID = 0;
    private final ElapsedTime taskRunTimeout = new ElapsedTime();
    private void setTaskTo(int newTaskID)
    {
        previousTaskID = currentTaskID;
        currentTaskID = newTaskID;
        taskRunTimeout.reset();
    }
    
    private void spotA()
    {
        if (currentTaskID == 0)
        {
            //if (taskRunTimeout.milliseconds() > 2000)
            if(propsLocation == EyeAll.ObjectLocation.ON_CAMERA_LEFT ||
            propsLocation == EyeAll.ObjectLocation.ON_CAMERA_RIGHT ||
            propsLocation == EyeAll.ObjectLocation.CENTER)
                setMissionTo(Mission.MOVE_A2B);
            else if (taskRunTimeout.seconds() > 4)
            {
                // timeout, camera problems...
                setMissionTo(Mission.MOVE_A2B);
            }
            else
            {
                EyeOp();
            }
        }
        else
        {
            if(eye != null) {
                eye.CloseEye();
                eye.stop();
            }

            setMissionTo(Mission.MOVE_A2B);
        }
    }
    

    private void RedLeft_Left()
    {
        if (currentTaskID == 0) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 24, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(1);
            }
        }
        else if (currentTaskID == 1)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, 90);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(2);
            }
        }
        else if (currentTaskID == 2)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 4, 90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(3);
            }
        }
        else if (currentTaskID == 3)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, -8, 90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(4);
            }
        }
        else if (currentTaskID == 4)
        {
            boolean done = driveStrafeLoop(24, DRIVE_SPEED, 10, 90);;

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(5);
            }
        }
        else if(currentTaskID == 5)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, -90);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(6);
            }
        }
        else if (currentTaskID == 6) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 60, -90);
            if (taskRunTimeout.seconds() >= 10) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            } else if (done) {
                setMissionTo(Mission.EXIT); // todo
            }
        }
}

    private void RedLeft_Right()
    {
        if (currentTaskID == 0) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 24, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(1);
            }
        }
        else if (currentTaskID == 1)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, -90);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(2);
            }
        }
        else if (currentTaskID == 2)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 4, -90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(3);
            }
        }
        else if (currentTaskID == 3)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, -8, -90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(4);
            }
        }
        else if (currentTaskID == 4)
        {
            boolean done = driveStrafeLoop(-24, DRIVE_SPEED, 10, -90);;

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(5);
            }
        }
        else if (currentTaskID == 5) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 60, -90);
            if (taskRunTimeout.seconds() >= 10) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            } else if (done) {
                setMissionTo(Mission.EXIT);
            }
        }
    }

    private void RedLeft_Center()
    {
        if (currentTaskID == 0) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 24, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(1);
            }
        }
        else if (currentTaskID == 1)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, -12, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(2);
            }
        }
        else if (currentTaskID == 2)
        {
            boolean done = driveStrafeLoop(24, DRIVE_SPEED, 10, 0.0);;

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(3);
            }
        }
        else if (currentTaskID == 3)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 36, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(4);
            }
        }
        else if(currentTaskID == 4)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, -90);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(5);
            }
        }
        else if (currentTaskID == 5)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 60, -90);
            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setMissionTo(Mission.EXIT);
            }
        }
    }

    private void RedRight_Left()
    {
        if (currentTaskID == 0) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 24, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(1);
            }
        }
        else if (currentTaskID == 1)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, 90);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(2);
            }
        }
        else if (currentTaskID == 2)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 4, 90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(3);
            }
        }
        else if (currentTaskID == 3)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, -24, 90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(4);
            }
        }
        else if (currentTaskID == 4)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, -45);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(5);
            }
        }
        else if (currentTaskID == 5)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, -24, -45);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done) {
                setMissionTo(Mission.EXIT);
            }
        }
    }

    private void RedRight_Right()
    {
        if (currentTaskID == 0) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 24, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(1);
            }
        }
        else if (currentTaskID == 1)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, -90);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(2);
            }
        }
        else if (currentTaskID == 2)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 4, -90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(3);
            }
        }
        else if (currentTaskID == 3)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, -8, -90);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(4);
            }
        }
        else if (currentTaskID == 4)
        {
            boolean done = driveStrafeLoop(20, DRIVE_SPEED, 10, -90);;

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(5);
            }
        }
        else if (currentTaskID == 5) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 24, -90);
            if (taskRunTimeout.seconds() >= 10) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            } else if (done) {
                setMissionTo(Mission.EXIT);
            }
        }
    }

    private void RedRight_Center()
    {
        if (currentTaskID == 0) {
            boolean done = driveStraightLoop(DRIVE_SPEED, 24, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(1);
            }
        }
        else if (currentTaskID == 1)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, -20, 0.0);

            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(2);
            }
        }
        else if(currentTaskID == 2)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, -90);
            if(taskRunTimeout.seconds() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(3);
            }
        }
        else if (currentTaskID == 3)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 40, -90);
            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setMissionTo(Mission.EXIT);
            }
        }
    }

    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private void spotB(int rotatorP, boolean isD) {
        if (currentTaskID == 0) {
            targetFound = false;    // Set to true when an AprilTag target is detected
            drive = 0;        // Desired forward power/speed (-1 to +1)
            strafe = 0;        // Desired strafe power/speed (-1 to +1)
            turn = 0;        // Desired turning power/speed (-1 to +1)

            // Initialize the Apriltag Detection process
            initAprilTag();
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

            setTaskTo(1);
        } else if (currentTaskID == 1) {
            targetFound = false;
            desiredTag = null;

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
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            if (taskRunTimeout.seconds() >= 10) {
                // timeout, bad! should not happen at all
                drive = 0;        // Desired forward power/speed (-1 to +1)
                strafe = 0;        // Desired strafe power/speed (-1 to +1)
                turn = 0;        // Desired turning power/speed (-1 to +1)

                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            } else if (targetFound &&
                    Util.inRange(drive, -.1, .1) &&
                    Util.inRange(turn, -.1, .1) &&
                    Util.inRange(strafe, -.1, .1)) {
                drive = 0;        // Desired forward power/speed (-1 to +1)
                strafe = 0;        // Desired strafe power/speed (-1 to +1)
                turn = 0;        // Desired turning power/speed (-1 to +1)

                setMissionTo(Mission.EXIT);
            }

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
        }
    }

    int targetPositionLifter;
    int targetPositionArm;
    private final PID armPID = new PID(0.0035,0,0.025,0.1, 0.5, -0.2);
    
    //-1300->0->1300: right down->horizontal->right up
    boolean linkArmWrist = false;
    private void armMotorRunnable()
    {
//        armPID.setSetpoint(targetPositionArm);
//        armMotor.setTargetPosition(targetPositionArm);
//        armMotor.setPower(armPID.update(armMotor.getCurrentPosition(),
//                getFeedForward(armPID.getF())));
//
//        if (!Util.inRange(armMotor.getCurrentPosition(),
//                Constants.ARM_POSITION_LOWEST, Constants.ARM_POSITION_HIGHEST))
//        {
//            targetPositionArm = (int)Util.trim(targetPositionArm,
//                    Constants.ARM_POSITION_LOWEST, Constants.ARM_POSITION_HIGHEST);
//            armMotor.setPower(0);
//        }
//
//        //arm:0, wrist 0
//        //arm 2200, wrist 1
//        if(linkArmWrist)
//        {
//            double ws = Util.trim((double) armMotor.getCurrentPosition() / 2400.0, 0, 1);
//            wristServo.setPosition(ws);
//        }
    }
    
    double arb_feedForward = 0;
    private double getFeedForward(double horizontalHoldoutput)
    {
//        double offset = 0; // angle degree
//        double sensorPos = armMotor.getCurrentPosition();
//        double angle = ((sensorPos-Constants.armHorizontal) / Constants.armRightUp) * 180 + offset;
//
//        double theta = Math.toRadians(angle);
//        double gravityCompensation = Math.cos(theta);
//        arb_feedForward = gravityCompensation * horizontalHoldoutput;
        return arb_feedForward;
    }
    
    int targetPositionRotator = 0;
    //ElapsedTime rotatorMotorTimer = new ElapsedTime();
    //int rotatorTimeoutMillsecs = 2000;
    //double rotatorMovePower = 0.2;
    private final PID rotatorPID = new PID(0.005,0,0,0, 0.4, -0.4);
    //private final int rotatorPositionMax = 700; // hits the ext-hub
    //private final int rotatorPositionMin = -600;
    private void rotatorMotorRunnable()
    {
//        rotatorPID.setSetpoint(targetPositionRotator);
//        rotatorMotor.setTargetPosition(targetPositionRotator);
//        rotatorMotor.setPower(rotatorPID.update(rotatorMotor.getCurrentPosition()));
//
//        if (!Util.inRange(rotatorMotor.getCurrentPosition(),
//                Constants.ROTATOR_POSITION_MOST_RIGHT, Constants.ROTATOR_POSITION_MOST_LEFT))
//        {
//            targetPositionRotator = (int)Util.trim(targetPositionRotator,
//                    Constants.ROTATOR_POSITION_MOST_RIGHT, Constants.ROTATOR_POSITION_MOST_LEFT);
//            rotatorMotor.setPower(0);
//        }
    }
    
    private void lifterMotorRunnable()
    {
        //lifterMotor.setPower(0);
    }
    
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void resetDriveLoops()
    {
        hasInitStraight = false;
        hasInitTurnTo = false;
        hasInitHold = false;
        hasInitStrafe = false;
    }
    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    *  maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    *  distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    *  heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    private boolean hasInitStraight = false;
    public boolean driveStraightLoop(double maxDriveSpeed,
                                     double distance,
                                     double heading)
    {
        if(!hasInitStraight)
        {
            hasInitStraight = true;
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
            backRightTarget = backRight.getCurrentPosition() + moveCounts;
            frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
        
            // Set Target FIRST, then turn on RUN_TO_POSITION
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);
            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
        
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, false);
        }
        
        // keep looping while we are still active, and all motors are running.
        if (backLeft.isBusy() && backRight.isBusy() &&
                frontLeft.isBusy() && frontRight.isBusy())
        {
        
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
        
            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
            {
                turnSpeed *= -1.0;
    
                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, turnSpeed, true);
            }
            else
            {
                moveRobot(maxDriveSpeed, turnSpeed, false);
            }
        
            // Display drive status for the driver.
            //sendTelemetry(true);
            return false;
        }
        else
        {
            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, false);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetDriveLoops();
            return true;
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     *  maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     *  heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    private boolean hasInitTurnTo = false;
    public boolean turnToHeadingLoop(double maxTurnSpeed, double heading)
    {
        if(!hasInitTurnTo)
        {
            hasInitTurnTo = true;
            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);
        }
    
        // keep looping while we are still active, and not on heading.
        if (Math.abs(headingError) > HEADING_THRESHOLD)
        {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed,false);

            // Display drive status for the driver.
            //sendTelemetry(false);
            return false;
        }
        else
        {
            // Stop all motion;
            moveRobot(0, 0,false);
            resetDriveLoops();
            return true;
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     *  maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     *  heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     *  holdTime   Length of time (in seconds) to hold the specified heading.
     */
    private boolean hasInitHold = false;
    ElapsedTime holdTimer = new ElapsedTime();
    public boolean holdHeadingLoop(double maxTurnSpeed, double heading, double holdTime)
    {
        if(!hasInitHold)
        {
            hasInitHold = true;
            holdTimer.reset();
        }
        
        if (holdTimer.time() < holdTime)
        {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, false);

            // Display drive status for the driver.
            //sendTelemetry(false);
            return false;
        }
        else
        {
            // Stop all motion;
            moveRobot(0, 0, false);
            resetDriveLoops();
            return true;
        }
    }
    
    private boolean hasInitStrafe = false;
    ElapsedTime runtimeStrafe = new ElapsedTime();
    // Positive Inch: move left.
    public boolean driveStrafeLoop(double Inches, double maxSpeed, int timeoutInSeconds, double heading)
    {
        if(!hasInitStrafe)
        {
            hasInitStrafe = true;
            int a, b, c, d;
            if (Inches < 0)
            {
                a = frontRight.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
                b = frontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                c = backRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                d = backLeft.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
            }
            else
            {// move to left side, stay at the robot back to see
                a = frontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                b = frontLeft.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
                c = backRight.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
                d = backLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }
            frontRight.setTargetPosition(a);
            frontLeft.setTargetPosition(b);
            backRight.setTargetPosition(c);
            backLeft.setTargetPosition(d);
    
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
            runtimeStrafe.reset();
        }
        
        if (runtimeStrafe.seconds() < timeoutInSeconds
                && backLeft.isBusy() && backRight.isBusy()
                && frontRight.isBusy() && frontLeft.isBusy()
        )
        {
            if (Inches < 0)
            {
                frontLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - heading) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - heading) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed + (getRawHeading() - heading) / 100, -1, 1));
                frontRight.setPower(Range.clip(maxSpeed - (getRawHeading() - heading) / 100, -1, 1));
            }
            else
            {
                frontLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - heading) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - heading) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed - (getRawHeading() - heading) / 100, -1, 1));
                frontRight.setPower(Range.clip(maxSpeed + (getRawHeading() - heading) / 100, -1, 1));
            }
            return false;
            /*if(!backRight.isBusy() || !backLeft.isBusy())
            {
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                return true;
            }
            else
            {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
    
                // if driving in reverse, the motor correction also needs to be reversed
                if (Inches < 0)
                    turnSpeed *= -1.0;
                
                frontLeft.setPower(-0.2);
                backRight.setPower(-0.2);
                frontRight.setPower(0.2);
                backLeft.setPower(0.2);
                return false;
            }*/
        }
        else // timeout
        {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetDriveLoops();
            return true;
        }
    }
    
    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    private double getSteeringCorrection(double desiredHeading, double proportionalGain)
    {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    private void moveRobot(double drive, double turn, boolean isBackward)
    {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        /*if(isBackward)
        {
            leftSpeed = -1*leftSpeed;
            rightSpeed = -1*rightSpeed;
        }*/
        
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);
        frontRight.setPower(rightSpeed);
        frontLeft.setPower(leftSpeed);
    }
    
    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    private double getRawHeading()
    {
        //Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return angles.firstAngle;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the "offset" heading back to zero
     */
    private void resetHeading()
    {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    
    // ****************  updating telemetry **********************
    //Orientation currentRobotAngles;
    YawPitchRollAngles currentRobotAngles;
    private void composeTelemetry()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            currentRobotAngles = imu.getRobotYawPitchRollAngles();
            //currentRobotAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });
        
        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });*/


        telemetry.addData("Props Location : ",
                propsLocation.toString());
    
        telemetry.addLine()
                .addData("Mission #", currentMission)
                .addData("Task #", currentTaskID);

        telemetry.addData("IMU heading", "%.2f", getRawHeading());
    
        //telemetry.addData("Camera On?", eye.IsOpen);
        //telemetry.addData("April Tag ID #", isCenterPole);
        //telemetry.addData("Cone Location", isConeCenter);
    

    
//        telemetry.addLine()
//                .addData("Eye", "%.2f", eyeServo.getPosition())
//                .addData("Elbow", "%.2f", elbowServo.getPosition())
//                .addData("Wrist", "%.2f", wristServo.getPosition())
//                .addData("Palm", "%.2f", palmServo.getPosition())
//                .addData("Finger", "%.2f", fingerServo.getPosition());
//
//        telemetry.addLine().addData("Lifter", lifterMotor.getCurrentPosition())
//                .addData("Rotator", rotatorMotor.getCurrentPosition())
//                .addData("Arm", armMotor.getCurrentPosition());
    }
    
    private void sendTelemetry(boolean straight)
    {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos bL:bR:fL:fR",  "%7d:%7d:%7d:%7d",
                    backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
        } else {
            telemetry.addData("Motion", "Turning");
        }
        
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Wheel Positions L:R",  "%7d:%7d",      backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
    }
    
    private String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    
    private String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
    private void addLog()
    {
        String a = String.format(
                new String(
                        //"Lm(%d : %d : %.2f) Rm(%d : %d : %.2f) Am(%d : %d : %.2f) " +
                        "IMU(%.2f) DR(%.2f:%.2f:%.2f:%.2f:%.2f:%.2f) Vi(%s:%d)"),
//P:%s/C:%s; LR:%d,FN:%d
//                lifterMotor.getCurrentPosition(),
//                lifterMotor.getTargetPosition(),
//                lifterMotor.getPower(),
//
//                rotatorMotor.getCurrentPosition(),
//                rotatorMotor.getTargetPosition(),
//                rotatorMotor.getPower(),
//
//                armMotor.getCurrentPosition(),
//                armMotor.getTargetPosition(),
//                armMotor.getPower(),
                
                getRawHeading(),
        
                frontLeft.getPower(),
                frontRight.getPower(),
                backLeft.getPower(),
                backRight.getPower(),
                driveSpeed,
                turnSpeed,
        
                currentMission.toString(),
                currentTaskID
                //isCenterPole.toString(),
                //isConeCenter.toString(),
                //error_lr,
                //error_fn
        );
        Log.println(Log.INFO,"Status", a);
    }
    
    final double TRIGGER_THRESHOLD = 0.75;     // Squeeze more than 3/4 to get rumble.
    private boolean eyeServoRightLock = false;
    private boolean eyeServoLeftLock = false;
    //private boolean wristServoRightLock = false;
    //private boolean wristServoLeftLock = false;
    private boolean palmServoLeftLock = false;
    private boolean palmServoRightLock = false;
    private boolean lifterHighLock = false;
    private boolean lifterLowLock = false;
    //private boolean leftTriggerLock = false;
    //private boolean armPositionLock = false;
    ElapsedTime runtimeManual = new ElapsedTime();
    
    private void TuningHandMotors(Gamepad gamepad)
    {
        //Eye Servo
        if (gamepad.left_trigger > TRIGGER_THRESHOLD)
        {
            if (!eyeServoRightLock)
            {
                eyeServoRightLock = true;  // Hold off any more triggers
                //eyeServo.setPosition(eyeServo.getPosition() + 0.02);
            }
        }
        else
        {
            eyeServoRightLock = false;  // We can trigger again now.
        }
        
        if (gamepad.right_trigger > TRIGGER_THRESHOLD)
        {
            if (!eyeServoLeftLock)
            {
                eyeServoLeftLock = true;  // Hold off any more triggers
                //eyeServo.setPosition(eyeServo.getPosition() - 0.02);
            }
        }
        else
        {
            eyeServoLeftLock = false;  // We can trigger again now.
        }
    
        // Elbow servo
        if (gamepad.left_stick_y > 0.3)
        {
            //elbowServo.setPosition(elbowServo.getPosition() + 0.01);
        }
        if (gamepad.left_stick_y < -0.3)
        {
            //elbowServo.setPosition(elbowServo.getPosition() - 0.05);
        }
        
        // Wrist servo
        if (gamepad.right_stick_y > 0.3)
        {
            //wristServo.setPosition(wristServo.getPosition() + 0.01);
        }
        if (gamepad.right_stick_y < -0.3)
        {
            //wristServo.setPosition(wristServo.getPosition() - 0.05);
        }
        
        // Palm servo
        if (gamepad.b)
        {
            if (!palmServoLeftLock)
            {
                palmServoLeftLock = true;
                //palmServo.setPosition(palmServo.getPosition() - 0.05);
            }
        }
        else
        {
            palmServoLeftLock = false;
        }
        if (gamepad.x)
        {
            if (!palmServoRightLock)
            {
                palmServoRightLock = true;
                //palmServo.setPosition(palmServo.getPosition() + 0.05);
            }
        }
        else
        {
            palmServoRightLock = false;
        }
        
        // finger servo
        if (gamepad.y)
        {
            //fingerServo.setPosition(1.);//open
        }
        if (gamepad.a)
        {
            //fingerServo.setPosition(0.1);// close
        }
        
        // arm motor
        /*double updown = -1 * Util.applyDeadband(gamepad.left_stick_y, 0.1);
        if( updown > 0)
            updown = updown * 60;
        else
            updown = updown * 40;
        targetPositionArm += updown;*/
        if(gamepad.dpad_up)
        {
            targetPositionArm += 10;
        }
        else if(gamepad.dpad_down)
        {
            targetPositionArm -= 10;
        }
        
        // rotator motor
        else if (gamepad.dpad_left)
        {
            targetPositionRotator += 10;
        }
        else if (gamepad.dpad_right)
        {
            targetPositionRotator -= 10;
        }
        
        //lifter motor
        if (gamepad.left_stick_button)
        {//up
            if (!lifterHighLock)
            {
                lifterHighLock = true;  // Hold off any more triggers
//                if (lifterMotor.getCurrentPosition() < 2000)
//                { // max high limit
//                    lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() + 100);
//                    lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    runtimeManual.reset();
//                    lifterMotor.setPower(0.2);
//                    while ((runtimeManual.seconds() < 1) &&
//                            (lifterMotor.isBusy()))
//                    {
//                    }
//                }
                //gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
            }
        }
        else
        {
            lifterHighLock = false;  // We can trigger again now.
        }
        
        if (gamepad.right_stick_button)
        {//down
            if (!lifterLowLock)
            {
                lifterLowLock = true;
//                if (lifterMotor.getCurrentPosition() > 100) // min low limit
//                {
//                    lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() - 100);
//                    lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    runtimeManual.reset();
//                    lifterMotor.setPower(0.2);
//                    while ((runtimeManual.seconds() < 1) &&
//                            (lifterMotor.isBusy()))
//                    {
//                    }
//                }
            }
        }
        else
        {
            lifterLowLock = false;
        }
    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

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
        this.frontLeft.setPower(leftFrontPower);
        this.frontRight.setPower(rightFrontPower);
        this.backLeft.setPower(leftBackPower);
        this.backRight.setPower(rightBackPower);
    }

    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
    }
    /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        */
    private void    setManualExposure(int exposureMS, int gain) {
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
