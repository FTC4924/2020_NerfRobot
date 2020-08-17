package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="ArcDrive")
public class ArcDrive extends OpMode {

    private DcMotor middleMotor;

    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private double middlePower;
    private double leftPower;
    private double rightPower;

    private double gamepad1LeftStickY;
    private double gamepad1LeftStickX;
    private double gamepad1RightStickX;
    private double gamepad1RightStickY;

    @Override
    public void init() {

        middleMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        middlePower = 0;
        leftPower = 0;
        rightPower = 0;

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x / 2;
        gamepad1RightStickY = gamepad1.right_stick_y;

        if(gamepad1LeftStickX > 0.05 || gamepad1LeftStickX < -.05) {

            middlePower = gamepad1LeftStickX;
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

        middleMotor.setPower(middlePower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        telemetry.addData("middlePower", middlePower);
        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);
    }
}