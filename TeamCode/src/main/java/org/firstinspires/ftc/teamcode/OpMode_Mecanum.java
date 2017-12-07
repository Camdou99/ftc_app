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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode Mecanum", group="Linear Opmode")

public class OpMode_Mecanum extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FleftDrive = null;
    private DcMotor FrightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    private DcMotor ArmMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ArmMotor = hardwareMap.get(DcMotor.class, "Arm_motor");
        Claw claw = new Claw(hardwareMap.get(Servo.class, "servo_claw_left"));

        FleftDrive  = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");

        ArmMotor.setDirection(DcMotor.Direction.FORWARD);

        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);

        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;

            FleftDrive.setPower(v1);
            FrightDrive.setPower(v2);
            BleftDrive.setPower(v3);
            BrightDrive.setPower(v4);

            double ArmPower;
            ArmPower = gamepad2.right_stick_y;
            ArmMotor.setPower(ArmPower);

            if(gamepad1.right_bumper) {
                claw.open();
            }
            else if(gamepad1.left_bumper) {
                claw.close();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

class Claw {
    private double openPos = 0.6;
    private double closedPos = 0.3;

    private Servo servo;

    Claw(Servo servo) {
        this.servo = servo;
    }


    public void open() {
        servo.setPosition(openPos);
    }

    public void close() {
        servo.setPosition(closedPos);
    }
}

