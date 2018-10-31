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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gamepad", group="Linear Opmode")
//1`@Disabled
public class Gamepad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor;
    boolean spinCollector = false, buttonPressed = false;
    double armMin = 0.5d, armMax = 2.9d;
    double servo1Timer;
    private AnalogInput armAngle;
    AndroidTextToSpeech tts;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    private Servo servo2, servo3;
    private Servo servo1;
    private double servoPosition0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.dcMotor.get("motor_0");
        rightDrive = hardwareMap.dcMotor.get("motor_1");
        armMotor = hardwareMap.dcMotor.get("motor_2");
        servo1 = hardwareMap.servo.get("servo_1");
        servo2 = hardwareMap.servo.get("servo_2");
        servo3 = hardwareMap.servo.get("servo_3");
        armAngle = hardwareMap.analogInput.get("arm_angle");

        tts = new AndroidTextToSpeech();
        tts.initialize();
        tts.setLanguage("eng");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tts.speak("Initialize Complete.");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            User1();

            User2();

            if (gamepad2.x || gamepad1.x) {
                if (!buttonPressed) {
                    spinCollector = !spinCollector;
                    buttonPressed = true;
                }
            } else {
                buttonPressed = false;
            }

            if (spinCollector) {

            } else {

            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Servo 1", "pos (%.2f)", servo1.getPosition());
            telemetry.addData("Arm", "pos (%.2f) pow (%.2f)", armAngle.getVoltage(), armMotor.getPower());
            telemetry.update();
        }
    }

    void User1()
    {
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double maxSpeed = 0.3d;
        if (gamepad1.left_bumper) {
            maxSpeed = 0.5d;
        } else if (gamepad1.right_bumper) {
            maxSpeed = 1d;
        }
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.scale(drive + turn, -1d, 1d, -maxSpeed, maxSpeed) ;
        rightPower   = Range.scale(drive - turn, -1d, 1d, -maxSpeed, maxSpeed) ;

        // Tank Mode uses one stick to control each wheel. dfdddfd2q
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void User2() {
        double p = 0d;
        double s1 = servo1.getPosition();

        if (gamepad2.left_trigger > 0d && armAngle.getVoltage() > armMin) { //Arm going up
            double m = armAngle.getVoltage() < 0.5d ? 0.5d : 1d;
            p = Range.scale(gamepad2.left_trigger, 0d, 1d, 0d, m);
        } else if (gamepad2.right_trigger > 0d && armAngle.getVoltage() < armMax) { //Arm going down
            double m = armAngle.getVoltage() > 2.4d ? 0.5d : 1d;
            p = -Range.scale(gamepad2.right_trigger, 0d, 1d, 0d, m);
        }

        armMotor.setPower(p);

        if (gamepad2.left_trigger > 0d || gamepad2.right_trigger > 0d) {
            if (armAngle.getVoltage() > 2.9d || armAngle.getVoltage() < 0.8d) {
                s1 = 0d;
            } else if (armAngle.getVoltage() > 1d && armAngle.getVoltage() < 1.78d) {
                s1 = 1d;
            } else if (armAngle.getVoltage() > 1.78d && armAngle.getVoltage() < 2.9d) {
                s1 = Range.scale(armAngle.getVoltage(), 1.78d, 2.9d, 0.95d, 0.05d);
            }
        }

        if (gamepad2.y && (runtime.milliseconds() - servo1Timer) > 35d) {
            servo1Timer = runtime.milliseconds();
            s1 = servo1.getPosition() + 0.015d;
        } else if (gamepad2.a && (runtime.milliseconds() - servo1Timer) > 35d) {
            servo1Timer = runtime.milliseconds();
            s1 = servo1.getPosition() - 0.015d;
        }

        servo1.setPosition(s1);

        double s2 = Range.scale(gamepad2.left_stick_x, -1d, 1d, 0.0d, 1d);
        double s3 = 0.2d, s3_mid = 0.2d;

        if (armAngle.getVoltage() < 1.18d && armAngle.getVoltage() > 0.9d) {
            s3 = s3_mid = Range.scale(armAngle.getVoltage(), 1.18d, 0.9d, 0.4d, 0.7d);
        }

        if (gamepad2.left_stick_y > 0d) {
            s3 = Range.scale(gamepad2.left_stick_y, 0d, 1d, s3_mid, 1d);
        } else if (gamepad2.left_stick_y < 0d) {
            s3 = Range.scale(gamepad2.left_stick_y, -1d, 0d, 0.0d, s3_mid);
        }

        if (armAngle.getVoltage() < 0.8d) {
            s3 = 1d;
        }

        servo2.setPosition(s2);
        servo3.setPosition(s3);
    }
}
