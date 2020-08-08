package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TankDrive")
public class TankDrive extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor middleWheelMotor;

    double leftPower;
    double rightPower;

    double gamepad1LeftStickY;
    double gamepad1LeftStickX;
    double gamepad1RightStickX;
    double gamepad1RightStickY;

    @Override
    public void init() {

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        middleWheelMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        leftPower = 0;
        rightPower = 0;

        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1RightStickX = gamepad1.right_stick_x;
        gamepad1RightStickX /= 2;

        if(gamepad1LeftStickX > 0.05 || gamepad1LeftStickX < -.05) {

            middleWheelMotor.setPower(gamepad1LeftStickX);
        }

        if(gamepad1LeftStickY > 0.05 || gamepad1LeftStickY < -.05) {

            leftPower = gamepad1LeftStickY;
            rightPower = gamepad1LeftStickY;

        }

        if(gamepad1RightStickX > 0.05){
            if(leftPower > 1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX - 1 + gamepad1RightStickX;
                rightPower = 1;

            } else if(leftPower < -1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX + 1 - gamepad1RightStickX;
                leftPower = -1;

            } else {

                rightPower += gamepad1RightStickX;
                leftPower -= gamepad1RightStickX;

            }
        } else if(gamepad1RightStickX < -0.05) {
            if(leftPower > 1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX - 1 - gamepad1RightStickX;
                leftPower = 1;

            } else if(leftPower < -1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX + 1 + gamepad1RightStickX;
                rightPower = -1;

            } else {

                leftPower -= gamepad1RightStickX;
                rightPower += gamepad1RightStickX;

            }
        }


        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);
    }
}