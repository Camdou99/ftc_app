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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class OpMode_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FleftDrive = null;
    private DcMotor FrightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    private DcMotor BlockMotor = null;

    private Servo Blockservo_left = null;
    private Servo Blockservo_right = null;

    private DcMotor RelicMotor = null;
    private Servo RelicServo = null;

    private Servo Color_sensor_servo = null;

    private double leftopen = 0.6;
    private double leftclosed = 0.3;
    private double rightopen = 0.6;
    private double rightclosed = 0.3;

    private double armpower = 1;

    private int current_state = 1;

    void Blockopen(){
        Blockservo_left.setPosition(leftopen);
        Blockservo_right.setPosition(rightopen);
    }

    void Blockclose(){
        Blockservo_left.setPosition(leftclosed);
        Blockservo_right.setPosition(rightclosed);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BlockMotor = hardwareMap.get(DcMotor.class, "Block_motor");

        Blockservo_left = hardwareMap.get(Servo.class, "Blockservo_left");
        Blockservo_right = hardwareMap.get(Servo.class, "Blockservo_right");

        RelicMotor = hardwareMap.get(DcMotor.class, "RelicMotor");
        RelicServo = hardwareMap.get(Servo.class, "RelicServo");

        Color_sensor_servo = hardwareMap.get(Servo.class, "Color_sensor_servo");

        FleftDrive  = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");

        BlockMotor.setDirection(DcMotor.Direction.REVERSE);

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

            if(gamepad1.dpad_up){
                if (current_state != 2){
                    current_state += 1;
                }
            }
            if(gamepad1.dpad_down){
                if (current_state != 1){
                    current_state -= 1;
                }
            }

            switch (current_state){
                case 1:
                    if(gamepad1.y){
                        BlockMotor.setPower(armpower);
                    }
                    else if (gamepad1.a){
                        BlockMotor.setPower(-armpower);
                    }
                    else{
                        BlockMotor.setPower(0);
                    }

                    if(gamepad1.right_bumper) {
                        Blockservo_left.setPosition(0);
                        Blockservo_right.setPosition(1);
                    }
                    else if(gamepad1.left_bumper) {
                        Blockservo_left.setPosition(1);
                        Blockservo_right.setPosition(0);
                    }
                    else{
                        Blockservo_left.setPosition(0.5);
                        Blockservo_right.setPosition(0.5);
                    }

                    /*if(gamepad1.a) {
                        Color_sensor_servo.setPosition(1);
                    } */
                    if(gamepad1.x) {
                        Color_sensor_servo.setPosition(0);
                    }
                    else{
                        Color_sensor_servo.setPosition(0.83);
                    }
                    break;
                case 2:
                    if(gamepad1.right_bumper){
                        RelicMotor.setPower(armpower);
                    }
                    else if (gamepad1.left_bumper){
                        RelicMotor.setPower(-armpower);
                    }
                    else{
                        RelicMotor.setPower(0);
                    }

                    if(gamepad1.x) {
                        RelicServo.setPosition(.75);
                    }
                    /*else if(gamepad1.b) {
                        RelicServo.setPosition(0.25);
                    } */
                    else{
                        RelicServo.setPosition(0.5);
                    }

                    break;
            }

            telemetry.addData("Color_sensor_servo", Color_sensor_servo.getPosition());
            telemetry.addData("RelicServo", RelicServo.getPosition());
            telemetry.addData("State",current_state);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}