/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FOCCLAW extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;
    private Servo gripperRight = null;
    private Servo gripperLeft = null;
    private int armDownTargetPosition = 100;
    private int armUpTargetPosition = 11500;
    private int armStartPosition = 0;
    private int armCurrentPosition = 0;
    private int wristCurrentPosition = 0;
    private int wristStartPosition = 0;
    private int wristUpTargetPosition = 0;
    private int wristScoreTargetPosition = -400;
    private int wristDownTargetPosition = -2300;
    private int gripperleft_Open = 0;
    private int gripperleft_Close = 0;
    private int gripperleftCurrentPosition = 0;
    private double gripperLeftClosedPosition = 0.425;
    private double gripperRightClosedPosition = 0.6;
    private double gripperLeftOpenPosition = 0.6;
    private double gripperRightOpenPosition = 0.45;

    static final double     ARM_UP_SPEED   = 1.0;
    static final double     ARM_DOWN_SPEED   = 0.5;
    static final double     WRIST_UP_SPEED = 0.5;
    static final double     WRIST_DOWN_SPEED = 0.5;
    static final double     GRIPPER_SPEED = 0.1;
    double arm_move_power =  0;
    double wrist_move_power = 0;
    double gripper_move_power = 0;

    double wristkP = .01;
    double wristkD = 0;
    double wristkI = 0;
    double armkP = .01;
    double armkD = 0;
    double armkI = 0;
    double wristPower = 0;
    double armPower = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor lfmotor0 = hardwareMap.dcMotor.get("lfmotor0");
        DcMotor lbmotor1 = hardwareMap.dcMotor.get("lbmotor1");
        DcMotor rfmotor2 = hardwareMap.dcMotor.get("rfmotor2");
        DcMotor rbmotor3 = hardwareMap.dcMotor.get("rbmotor3");

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor");
        gripperLeft = hardwareMap.get(Servo.class, "gripperLeft");
        gripperRight = hardwareMap.get(Servo.class, "gripperRight");

        gripperLeft.setDirection(Servo.Direction.FORWARD);
        gripperRight.setDirection(Servo.Direction.FORWARD);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Using our own PIDF, RUN_WITHOUT_ENCODER
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStartPosition = armMotor.getCurrentPosition();

        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Using our own PIDF, RUN_WITHOUT_ENCODER
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristStartPosition = wristMotor.getCurrentPosition();


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        lfmotor0.setDirection(DcMotorSimple.Direction.REVERSE);
        lbmotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        PIDController wristPID = new PIDController(wristkP, wristkI, wristkD);
        wristPID.setTolerance(30, 10);

        PIDController armPID = new PIDController(armkP, armkI, armkD);
        armPID.setTolerance(300, 10);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double arm_move = -gamepad2.left_stick_y;
            double wrist_move = -gamepad2.right_stick_y;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.back) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            lfmotor0.setPower(frontLeftPower);
            lbmotor1.setPower(backLeftPower);
            rfmotor2.setPower(frontRightPower);
            rbmotor3.setPower(backRightPower);

            if (gamepad2.left_bumper)
            {
                gripperLeft.setPosition(gripperLeftOpenPosition );
            } else
            {
                gripperLeft.setPosition(gripperLeftClosedPosition);
            }

            if (gamepad2.right_bumper)
            {
                gripperRight.setPosition(gripperRightOpenPosition );
            } else
            {
                gripperRight.setPosition(gripperRightClosedPosition);
            }

            armCurrentPosition = armMotor.getCurrentPosition();
            wristCurrentPosition = wristMotor.getCurrentPosition();

            /*
            if ((arm_move > 0) && (armCurrentPosition < armUpTargetPosition))
            {
                arm_move_power = arm_move * ARM_UP_SPEED;
                armMotor.setTargetPosition(armUpTargetPosition);
                armMotor.setPower(arm_move_power);
            } else if ((arm_move < 0) && (armCurrentPosition > armDownTargetPosition))
            {
                arm_move_power = arm_move * ARM_DOWN_SPEED;
                armMotor.setTargetPosition(armDownTargetPosition);
                armMotor.setPower(arm_move_power);
            } else
            {
                armMotor.setPower(0);
            }
            */

/*
            if ((wrist_move > 0) && (wristCurrentPosition < wristUpTargetPosition))
            //if (wrist_move > 0)
            {
                wrist_move_power = wrist_move * WRIST_UP_SPEED;
                wristMotor.setTargetPosition(wristUpTargetPosition);
                wristMotor.setPower(wrist_move_power);
            }
            else if ((wrist_move < 0) && (wristCurrentPosition > wristDownTargetPosition))
            //else if (wrist_move < 0)
            {
                wrist_move_power = wrist_move * WRIST_DOWN_SPEED;
                wristMotor.setTargetPosition(wristDownTargetPosition);
                wristMotor.setPower(wrist_move_power);
            } else
            {
                wristMotor.setPower(0);
            }*/

            /*
            if (gamepad2.a && (armCurrentPosition < 3000))
            {
                armCurrentPosition = armMotor.getCurrentPosition();
                while(armCurrentPosition > (armDownTargetPosition + 50))
                {
                    armMotor.setPower(-ARM_DOWN_SPEED);
                    armCurrentPosition = armMotor.getCurrentPosition();
                }

                wristCurrentPosition = wristMotor.getCurrentPosition();
                while(wristCurrentPosition > (wristDownTargetPosition + 50))
                {
                    wristMotor.setPower(-WRIST_DOWN_SPEED);
                    wristCurrentPosition = wristMotor.getCurrentPosition();
                }

            }

            if(gamepad2.y && (armCurrentPosition > 6000))
            {

               armCurrentPosition = armMotor.getCurrentPosition();
                while(armCurrentPosition < (armUpTargetPosition - 250))
                {
                    armMotor.setPower(0.1);
                    armCurrentPosition = armMotor.getCurrentPosition();
                    telemetry.addData("Current Arm Position", "%4d,", armCurrentPosition );
                    telemetry.update();
                }

                wristCurrentPosition = wristMotor.getCurrentPosition();
                while(wristCurrentPosition < (wristUpTargetPosition - 750))
                {
                    wristMotor.setPower(WRIST_UP_SPEED);
                    wristCurrentPosition = wristMotor.getCurrentPosition();
                }

            }
            */

            /*
            if(gamepad2.x || gamepad2.b)
            {
                if(gamepad2.x)
                {
                    wristPID.setSetPoint(wristDownTargetPosition);
                }
                else
                {
                    wristPID.setSetPoint(wristUpTargetPosition);
                }

                while (!wristPID.atSetPoint())
                {
                    wristCurrentPosition = wristMotor.getCurrentPosition();
                    double output = wristPID.calculate(wristCurrentPosition);
                    wristMotor.setPower(output);
                }
                wristMotor.setPower(0); // stop the motor
            }
            */

            if(!(arm_move == 0))
            {
                armCurrentPosition = armMotor.getCurrentPosition();
                armPID.setSetPoint(armCurrentPosition + (50 * arm_move));
            }
            if(!(wrist_move == 0))
            {
                wristCurrentPosition = wristMotor.getCurrentPosition();
                wristPID.setSetPoint(wristCurrentPosition + (50 * wrist_move));
            }
            if(gamepad2.a)
            {
                wristPID.setSetPoint(wristDownTargetPosition);
                armPID.setSetPoint(armDownTargetPosition);
            }

            if(gamepad2.y)
            {
                wristPID.setSetPoint(wristScoreTargetPosition);
                armPID.setSetPoint(armUpTargetPosition);
            }

            //while (!wristPID.atSetPoint()) {
            wristCurrentPosition = wristMotor.getCurrentPosition();
            wristPower = wristPID.calculate(wristCurrentPosition);

            armCurrentPosition = armMotor.getCurrentPosition();
            armPower = armPID.calculate(armCurrentPosition);
            //}
            wristMotor.setPower(wristPower);
            armMotor.setPower(armPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Left", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Both  Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Current Arm Position", "%4d,", armCurrentPosition );
            telemetry.addData("Current Wrist Position", "%4d", wristCurrentPosition);
            // telemetry.addData("Gripper Left Command", "%4.2f", gripperleft_move);
            telemetry.update();
        }
    }
}