/* Copyright (c) 2017 FIRST. All rights reserved.
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

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.core.subsystems.EyeAll;
import org.firstinspires.ftc.teamcode.util.CameraConfig;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp(name = "Calibration Camera ", group = "Calibration")
//@Disabled
public class CalibrationCamera extends LinearOpMode
{
    EyeAll eye;
    
    enum Task
    {
        PROP_LOCATION,
        RED_COLOR,
        BLUE_COLOR,
        PATH_CENTER
    }
    
    private Task CurrentTask = Task.PROP_LOCATION;
    private Task[] TaskIdxArray = Task.values();
    
    
    boolean keylock_a = false;
    boolean keylock_b = false;
    boolean keylock_x = false;
    boolean keylock_y = false;
    
    boolean keylock_up = false;
    boolean keylock_down = false;
    boolean keylock_left = false;
    boolean keylock_right = false;
    
    boolean keylock_left_bumper = false;
    boolean keylock_righ_bumper = false;
    
    boolean keylock_start = false;
    
    CameraConfig.CameraConfigData config;
    
    String returnMsg = "";
    
    /*private DcMotor lifterMotor;
    private DcMotor rotatorMotor;
    private DcMotor armMotor;
    private Servo eyeServo;
    private Servo wristServo;
    private Servo palmServo;
    private Servo fingerServo;*/
    
    private IMU imu;
    
    @Override
    public void runOpMode()
    {
        /*fingerServo = hardwareMap.servo.get(Constants.fingerServo);
        wristServo = hardwareMap.servo.get(Constants.wristServo);
        palmServo = hardwareMap.servo.get(Constants.palmServo);
        eyeServo = hardwareMap.servo.get(Constants.eyeServo);
    
        lifterMotor = hardwareMap.get(DcMotor.class, Constants.lifterMotor);
        rotatorMotor = hardwareMap.get(DcMotor.class, Constants.rotatorMotor);
        armMotor = hardwareMap.get(DcMotor.class, Constants.armMotor);
    
        //lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    
        // reset all motors encoder to zero. remove them since we use saved cali data
        //lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
    
        // define initialization values for IMU, and then initialize it.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
//        imu = hardwareMap.get(BNO055IMU.class, Constants.imu);
//        imu.initialize(parameters);
        imu = hardwareMap.get(IMU.class, Constants.imu);
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
        
        config = new CameraConfig.CameraConfigData();
    
        if( null == CameraConfig.ReadConfigFromFile(config))
        {
            config.ConePointAX = 325;
            config.ConePointAY = 10;
            config.ConePointBX = 425;
            config.ConePointBY = 350;

            config.Red = 180;
            config.Blue = 180;
    
            config.ConeWidth = 250;
            config.PathCenterX = 320;
    
            CameraConfig.SaveConfigToFile(config);
            telemetry.addData("Created", "New Config");
        }
        else
        {
            telemetry.addData("Loaded", "Existing Config");
        }
        telemetry.update();
    
        eye = new EyeAll(hardwareMap);
        eye.autoInit();
        eye.OpenEyeToFindProps();
        
        waitForStart();
        
        while (opModeIsActive())
        {
            if (gamepad1.left_bumper)
            {
                if (!keylock_left_bumper)
                {
                    keylock_left_bumper = true;
                    int currentStageNum = CurrentTask.ordinal();
                    int nextStageNum = currentStageNum + 1;
                    if (nextStageNum >= TaskIdxArray.length)
                    {
                        nextStageNum = 0;
                    }
                    CurrentTask = TaskIdxArray[nextStageNum];
                }
            }
            else
            {
                keylock_left_bumper = false;
            }
            if (gamepad1.right_bumper)
            {
                if (!keylock_righ_bumper)
                {
                    keylock_righ_bumper = true;
                    int currentStageNum = CurrentTask.ordinal();
                    int nextStageNum = currentStageNum - 1;
                    if (nextStageNum < 0)
                    {
                        nextStageNum = TaskIdxArray.length - 1;
                    }
                    CurrentTask = TaskIdxArray[nextStageNum];
                }
            }
            else
            {
                keylock_righ_bumper = false;
            }
    
            if(CurrentTask == Task.PROP_LOCATION)
            {
                objectLocationTuning();
            }
            else
            {
                objectParametersTuning(CurrentTask);
            }
    
            EyeOp();
            
            telemetry.addData("Current Task", CurrentTask);
    
            if (gamepad1.start)
            {
                if(!keylock_start)
                {
                    keylock_start = true;
    
                    returnMsg = CameraConfig.SaveConfigToFile(config);

                    if (returnMsg != null)
                    {
                        telemetry.addLine(returnMsg);
                        int[] data = CameraConfig.ReadConfigFromFile(config);
                        if (data != null)
                        {
                            telemetry.addLine("Saved Config: " +
                                    data[0] + " " + data[1] + " " +
                                    data[2] + " " + data[3] + " " +
                                    data[4] + " " + data[5] + " " +
                                    data[6] + " " + data[7]);
                        }
                    }
                }
            }
            else
            {
                keylock_start = false;
            }

            YawPitchRollAngles currentRobotAngles = imu.getRobotYawPitchRollAngles();
    
            telemetry.addLine("Prop- Ax:" +
                    config.ConePointAX + " Ay:" + config.ConePointAY +
                    " Bx:" + config.ConePointBX + " By:" + config.ConePointBY + " W:" + config.ConeWidth);

            telemetry.addLine("Colors- R:" +
                    config.Red + " B:" + config.Blue);
    
            telemetry.addData("Path Center", "%d", config.PathCenterX);
    
            telemetry.addData("IMU heading", "%.2f", currentRobotAngles.getYaw(AngleUnit.DEGREES));
    
            /*telemetry.addLine()
                    .addData("Eye", "%.2f", eyeServo.getPosition())
                    .addData("Wrist", "%.2f", wristServo.getPosition())
                    .addData("Palm", "%.2f", palmServo.getPosition())
                    .addData("Finger", "%.2f", fingerServo.getPosition());
    
            telemetry.addLine().addData("Lifter", lifterMotor.getCurrentPosition())
                    .addData("Rotator", rotatorMotor.getCurrentPosition())
                    .addData("Arm", armMotor.getCurrentPosition());*/
            
            telemetry.update();
    
            sleep(10);   // optional pause after each move.
        }
    }
    
    ElapsedTime eyeruntime = new ElapsedTime();
    boolean waitabit = false;
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
    
        eye.UpdateConfigDataLoop(config);
        eye.SetOpInfoOnScreen(CurrentTask.toString());
        
        EyeAll.ObjectLocation location = EyeAll.ObjectLocation.UNKNOWN;
        
        if(CurrentTask == Task.PROP_LOCATION)
        {
            location = eye.CheckPropsLocation(EyeAll.TargetObject.RED_CONE);
        }
        else
        {
            if(CurrentTask == Task.BLUE_COLOR)
            {
                location = eye.CheckPropsLocation(EyeAll.TargetObject.BLUE_CONE);
            }
            else if(CurrentTask == Task.RED_COLOR)
            {
                location = eye.CheckPropsLocation(EyeAll.TargetObject.RED_CONE);
            }
        }
        
        if(location != EyeAll.ObjectLocation.WAITING && location != EyeAll.ObjectLocation.UNKNOWN)
        {
            waitabit = true;
            eyeruntime.reset();
        }
    }
    
    private void objectParametersTuning(Task tObject)
    {
        int value = 0;
        if(tObject == Task.BLUE_COLOR)
            value = config.Blue;
        else if(tObject == Task.RED_COLOR)
            value = config.Red;
        else //PATH_CENTER
            value = config.PathCenterX;
    
        value += (int)(Util.applyDeadband(gamepad1.left_stick_y, 0.2)*(-1));
        if(tObject == Task.PATH_CENTER)
            value = (int)Util.trim(value, 0, Constants.CameraViewWidth);
        else
            value = (int)Util.trim(value, 0, 255);
    
        if(tObject == Task.BLUE_COLOR)
            config.Blue = value;
        else if(tObject == Task.RED_COLOR)
            config.Red = value;
        else //PATH_CENTER
            config.PathCenterX = value;
    }
    
    private void objectLocationTuning()
    {
        // bottom line updating
        if (gamepad1.a)
        {
            //if (!keylock_a)
            {
                keylock_a = true;
                
                if (gamepad1.dpad_up)
                {
                    if (!keylock_up)
                    {
                        keylock_up = true;

                        config.ConePointBY -= 5;
                        // TODO with limitation of AY
                    }
                }
                else
                {
                    keylock_up = false;
                }
                
                if (gamepad1.dpad_down)
                {
                    if (!keylock_down)
                    {
                        keylock_down = true;

                        config.ConePointBY += 5;
                    }
                }
                else
                {
                    keylock_down = false;
                }
            }
        }
        else
        {
            keylock_a = false;
            if (!keylock_y)
            {
                keylock_up = false;
                keylock_down = false;
            }
        }
        
        // top line updating
        if (gamepad1.y)
        {
            //if (!keylock_y)
            {
                keylock_y = true;
                
                if (gamepad1.dpad_up)
                {
                    if (!keylock_up)
                    {
                        keylock_up = true;

                        config.ConePointAY -= 5;
                    }
                }
                else
                {
                    keylock_up = false;
                }
                
                if (gamepad1.dpad_down)
                {
                    if (!keylock_down)
                    {
                        keylock_down = true;
                        config.ConePointAY += 5;
                    }
                }
                else
                {
                    keylock_down = false;
                }
            }
        }
        else
        {
            keylock_y = false;
            if (!keylock_a)
            {
                keylock_up = false;
                keylock_down = false;
            }
        }
        
        // left line updating
        if (gamepad1.x)
        {
            //if (!keylock_x)
            {
                keylock_x = true;
                
                if (gamepad1.dpad_left)
                {
                    if (!keylock_left)
                    {
                        keylock_left = true;
                        config.ConePointAX -= 5;
                    }
                }
                else
                {
                    keylock_left = false;
                }
                
                if (gamepad1.dpad_right)
                {
                    if (!keylock_right)
                    {
                        keylock_right = true;
                        config.ConePointAX += 5;
                    }
                }
                else
                {
                    keylock_right = false;
                }
            }
        }
        else
        {
            keylock_x = false;
            
            if (!keylock_b)
            {
                keylock_left = false;
                keylock_right = false;
            }
        }
        
        // right line updating
        if (gamepad1.b)
        {
            //if (!keylock_b)
            {
                keylock_b = true;
                
                if (gamepad1.dpad_left)
                {
                    if (!keylock_left)
                    {
                        keylock_left = true;
                        config.ConePointBX -= 5;
                    }
                }
                else
                {
                    keylock_left = false;
                }
                
                if (gamepad1.dpad_right)
                {
                    if (!keylock_right)
                    {
                        keylock_right = true;
                        config.ConePointBX += 5;
                    }
                }
                else
                {
                    keylock_right = false;
                }
            }
        }
        else
        {
            keylock_b = false;
            if (!keylock_x)
            {
                keylock_left = false;
                keylock_right = false;
            }
        }
    }
}