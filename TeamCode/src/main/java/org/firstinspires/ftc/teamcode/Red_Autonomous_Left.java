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

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red_Autonomous_Left")

public class Red_Autonomous_Left extends LinearOpMode {
    public DcMotor FleftDrive = null;
    public DcMotor FrightDrive = null;
    public DcMotor BleftDrive = null;
    public DcMotor BrightDrive = null;

    private Servo RelicServo = null;

    private Servo Blockservo_left = null;
    private Servo Blockservo_right = null;

    public Servo Color_sensor_servo = null;
    ModernRoboticsI2cColorSensor Color_sensor = null;

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ElapsedTime timer = new ElapsedTime();

    public double speed = .5;

    private double Current_Heading = 0;
    private double Target_Heading = 0;

    private double Current_State = 0;

    void stopDrive() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }

    void forward(long time) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(-speed);
        sleep(time);
    }

    void reverse(long time) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }

    void strafeleft(long time) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }

    void straferight(long time) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);
        sleep(time);
    }

    void TurnLeft(long time) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);
        //sleep(time);
    }
    void TurnRight(long time) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
        //sleep(time);
    }

    int GetHeading(){
        int heading = modernRoboticsI2cGyro.getHeading();
        return heading;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FleftDrive = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");

        RelicServo = hardwareMap.get(Servo.class, "RelicServo");

        Blockservo_left = hardwareMap.get(Servo.class, "Blockservo_left");
        Blockservo_right = hardwareMap.get(Servo.class, "Blockservo_right");

        Color_sensor_servo = hardwareMap.get(Servo.class, "Color_sensor_servo");

        Color_sensor = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "Color_sensor");
        Color_sensor.enableLight(true);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        telemetry.addData("Red", Color_sensor.red());
        telemetry.addData("Blue", Color_sensor.red());
        telemetry.addData("Heading", GetHeading());
        telemetry.update();

        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        Color_sensor.red();   // Red channel value
        Color_sensor.green(); // Green channel value
        Color_sensor.blue();  // Blue channel value

        Color_sensor.alpha(); // Total luminosity
        Color_sensor.argb();  // Combined color value

        Color_sensor.enableLight(true);

        waitForStart();

        Color_sensor.enableLight(true);

        Blockservo_left.setPosition(.5);
        Blockservo_right.setPosition(.5);
        RelicServo.setPosition(0.5);

        Armdown(2);
        Jewel(0.5);
    }

    public void Jewel(double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        telemetry.addData("Heading", GetHeading());
        telemetry.update();

        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (Color_sensor.blue() >= 1) {
                Target_Heading = 50;
                Current_Heading = GetHeading();
                if(Current_Heading > 180){
                    Current_Heading = Current_Heading - 360;
                }
                while (Current_Heading < Target_Heading) {
                    TurnLeft(200);
                    Current_Heading = GetHeading();
                    if (Current_Heading > 180) {
                        Current_Heading = Current_Heading - 360;
                    }
                }
                Color_sensor_servo.setPosition(.83);
                Target_Heading = 10;
                Current_Heading = GetHeading();
                if (Current_Heading > 180) {
                    Current_Heading = Current_Heading - 360;
                }
                while (Current_Heading < Target_Heading) {
                    TurnRight(200);
                    Current_Heading = GetHeading();
                    if (Current_Heading > 180) {
                        Current_Heading = Current_Heading - 360;
                    }
                }
            } else if (Color_sensor.red() >= 1) {
                TurnRight(100);
                Color_sensor_servo.setPosition(.83);
                TurnLeft(200);
            }
            FleftDrive.setPower(0);
            FrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
        }
    }

    public void Armdown(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            Color_sensor_servo.setPosition(0);
        }
    }

}









