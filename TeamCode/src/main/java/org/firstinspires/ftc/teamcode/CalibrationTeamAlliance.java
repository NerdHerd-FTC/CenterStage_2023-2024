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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceConfig;

@TeleOp(name = "Calibration Team Alliance ", group = "Calibration")
//@Disabled
public class CalibrationTeamAlliance extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        AllianceConfig.AllianceConfigData config = new AllianceConfig.AllianceConfigData();
    
        if( null == AllianceConfig.ReadConfigFromFile(config))
        {
            config.TeamNumber = "12345";
            config.Alliance = AllianceConfig.RED;
            config.Location = AllianceConfig.LEFT;
            config.PathRoute = "0";
    
            AllianceConfig.SaveConfigToFile(config);
            telemetry.addData("Created", "New Config");
        }
        else
        {
            telemetry.addData("Loaded", "Existing Config");
        }
        
        telemetry.addLine("Saved Init Config: " +
                config.TeamNumber + " " + config.Alliance.toString() +
                " " + config.Location.toString() + " " + config.PathRoute);
                
        telemetry.update();
        
        // Wait until we're told to go
        waitForStart();
        
        boolean keylock_a = false;
        boolean keylock_b = false;
        boolean keylock_x = false;
        boolean keylock_y = false;
        
        boolean keylock_up = false;
        boolean keylock_left = false;
        boolean keylock_right = false;
        
        String returnMsg = "";
        
        // Loop and update the dashboard
        while (opModeIsActive())
        {
            if (gamepad1.dpad_left)
            {
                if (!keylock_left)
                {
                    keylock_left = true;
                    config.PathRoute = "1";
                }
            }
            else
            {
                keylock_left = false;
            }
            
            if (gamepad1.dpad_up)
            {
                if (!keylock_up)
                {
                    keylock_up = true;
                    config.PathRoute = "2";
                }
            }
            else
            {
                keylock_up = false;
            }
    
            if (gamepad1.dpad_right)
            {
                if (!keylock_right)
                {
                    keylock_right = true;
                    config.PathRoute = "3";
                }
            }
            else
            {
                keylock_right = false;
            }
            
            if (gamepad1.a)
            {
                if (!keylock_a)
                {
                    keylock_a = true;
                    config.Location = AllianceConfig.RIGHT;
                }
            }
            else
            {
                keylock_a = false;
            }
            
            if (gamepad1.b)
            {
                if (!keylock_b)
                {
                    keylock_b = true;
                    config.Alliance = AllianceConfig.RED;
                }
            }
            else
            {
                keylock_b = false;
            }
            
            if (gamepad1.x)
            {
                if (!keylock_x)
                {
                    keylock_x = true;
                    config.Alliance = AllianceConfig.BLUE;
                }
            }
            else
            {
                keylock_x = false;
            }
            
            if (gamepad1.y)
            {
                if (!keylock_y)
                {
                    keylock_y = true;
                    config.Location = AllianceConfig.LEFT;
                }
            }
            else
            {
                keylock_y = false;
            }
            
            if( keylock_a|| keylock_b||keylock_x|| keylock_y ||
                    keylock_left || keylock_right || keylock_up)
            {
                telemetry.addLine("Saving Config: " +
                        config.TeamNumber + " " + config.Alliance.toString() +
                        " " + config.Location.toString() + " " + config.PathRoute);
    
                returnMsg = AllianceConfig.SaveConfigToFile(config);
                if( returnMsg != null )
                {
                    telemetry.addLine(returnMsg);
                    String[] data = AllianceConfig.ReadConfigFromFile(config);
                    if( data!= null)
                    {
                        telemetry.addLine("Read Config: " +
                                data[0] + " " + data[1] +
                                " " + data[2] + " " + data[3]);
                    }
                }
            }
            
            telemetry.update();
            
            //sleep(250);   // optional pause after each move.
        }
    }
}
