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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Util;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
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
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, Constants.leftfrontMotor);
        leftBackDrive  = hardwareMap.get(DcMotor.class, Constants.leftbackMotor);
        rightFrontDrive = hardwareMap.get(DcMotor.class, Constants.rightfrontMotor);
        rightBackDrive = hardwareMap.get(DcMotor.class, Constants.rightbackMotor);

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

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Software Version: ",
                BuildConfig.BUILD_TIME);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        PIDController wristPID = new PIDController(wristkP, wristkI, wristkD);
        wristPID.setTolerance(30, 10);

        PIDController armPID = new PIDController(armkP, armkI, armkD);
        armPID.setTolerance(300, 10);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double arm_move = -gamepad2.left_stick_y;
            double wrist_move = -gamepad2.right_stick_y;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

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


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Current Arm Position", "%4d,", armCurrentPosition );
            telemetry.addData("Current Wrist Position", "%4d", wristCurrentPosition);
            telemetry.update();
        }
    }

    double arb_feedForward = 0;
    private double getFeedForward(double horizontalHoldoutput)
    {
        double offset = 0; // angle degree
        double sensorPos = armMotor.getCurrentPosition();
        double angle = ((sensorPos) / Constants.armRightUp) * 90 + offset;

        double theta = Math.toRadians(angle);
        double gravityCompensation = Math.cos(theta);
        arb_feedForward = gravityCompensation * horizontalHoldoutput;
        return arb_feedForward;
    }

    private void runnableArmMotor()
    {
        armPID.setSetpoint(armTargetPosition);
        armMotor.setTargetPosition(armTargetPosition);
        armMotor.setPower(armPID.update(armMotor.getCurrentPosition(),
                getFeedForward(armPID.getF())));

        if (Util.inRange(armMotor.getCurrentPosition(), armPositionMin, armPositionMax) == false)
        {
            armTargetPosition = (int)Util.trim(armTargetPosition, armPositionMin, armPositionMax);
            armMotor.setPower(0);
        }
    }
}
