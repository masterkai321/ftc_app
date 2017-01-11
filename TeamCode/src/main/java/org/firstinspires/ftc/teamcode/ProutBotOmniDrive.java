/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Telop driving for a ProutBot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a ProutBot
 * It raises and lowers the tilt using the Gampad Y and A buttons respectively.
 * It also opens and closes the tilts slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ProutBot: OmniBot", group="ProutBot")
@Disabled
public class ProutBotOmniDrive extends OpMode{

    /* Declare OpMode members. */
    HardwareProutBotOmni robot       = new HardwareProutBotOmni(); // use the class created to define a ProutBot's hardware
                                                         // could also use HardwareProutBotMatrix class.
    double          tiltOffset  = 0.0 ;                  // Servo mid position
    final double    TILT_SPEED  = 0.02 ;                 // sets rate to move servo


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Yaw Left
        while (gamepad1.left_bumper) {
            robot.YawLeft();
        }
        //Yaw Right
        while (gamepad1.right_bumper) {
            robot.YawRight();
        }
        //Move at 45 Degrees
        while (gamepad1.dpad_up && gamepad1.dpad_right) {
            robot.GoNE();
        }
        //Move at 135 Degrees
        while (gamepad1.dpad_up && gamepad1.dpad_left) {
            robot.GoNW();
        }
        //Move at 225 Degrees
        while (gamepad1.dpad_down && gamepad1.dpad_left) {
            robot.GoSW();
        }
        //Move at 315 Degrees
        if (gamepad1.dpad_down && gamepad1.dpad_right) {
            robot.GoSE();
        }
        //Move Forward
        while (gamepad1.dpad_up) {
            robot.GoForward();
        }
        //Move Backward
        while (gamepad1.dpad_down) {
            robot.GoBackward();
        }
        //Move Right
        while (gamepad1.dpad_right) {
            robot.GoRight();
        }
        //Move Left
        while (gamepad1.dpad_left) {
            robot.GoLeft();
        }
        while (gamepad1.a) {
            robot.ShootParticle();
        }

        if (gamepad1.b)
            robot.brushMotor.setPower(robot.BRUSH_POWER);
        else
            robot.brushMotor.setPower(0.0);
        if (gamepad1.y)
            robot.pitchMotor.setPower(robot.PITCH_POWER);
        else
            robot.pitchMotor.setPower(0.0);

        robot.StopMovement();




        /* Use gamepad left & right Bumpers to tilt object tray
        if (gamepad1.right_bumper)
            tiltOffset += TILT_SPEED;
        else if (gamepad1.left_bumper)
            tiltOffset -= TILT_SPEED;

        // Move tilt servo to new position.
        tiltOffset = Range.clip(tiltOffset, -0.5, 0.5);
        robot.tiltServo.setPosition(robot.MID_SERVO + tiltOffset);
*/
        // Use gamepad buttons to turn on pitcher (Y)
        /*if (gamepad1.y)
            robot.catMotor.setPower(robot.CAT_POWER);
        else
            robot.catMotor.setPower(0.0);
*/
        //Use gamepad buttons to turn on brush (B)

        // Send telemetry message to signify robot running;
        //telemetry.addData("pitcher", "%.2f", robot.pitchMotor.getPower())
        //telemetry.addData("brush", "%.2f", robot.brushMotor.getPower())
            //telemetry.addData("tilt servo",  "Offset = %.2f", tiltOffset);

        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
