package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TopDown2DDrive")
public class TopDown2DDrive extends OpMode {

    private DcMotor middleWheelMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private double[] powers;

    private double gamepad1LeftStickX;
    private double gamepad1LeftStickY;

    private double angle;
    private double radius;

    @Override
    public void init() {

        middleWheelMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        powers = new double[2];

    }

    @Override
    public void loop() {

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;

        radius = xyToRadius(gamepad1LeftStickX, gamepad1LeftStickY);

        angle = xyToAngle(gamepad1LeftStickX, gamepad1LeftStickY);
        angle = angleReducer(angle);

        powers = angleToBasePower(angle);

        for(int i = 0; i < powers.length; i++) {

            powers[i] *= radius;

        }

        middleWheelMotor.setPower(powers[0]);
        rightMotor.setPower(powers[1]);
        leftMotor.setPower(powers[1]);

    }

    private double xyToRadius(double x, double y) {

        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    }

    private double xyToAngle(double x, double y) {

        return Math.atan2(x, y) - Math.toRadians(90);

    }

    private double angleReducer(double radians) {

        double degrees = Math.toDegrees(radians);

        degrees = degrees % 360;
        if(degrees < -180) {

            degrees = 360 + degrees;

        } else if(degrees > 180) {

            degrees = 360 - degrees;

        }

        return Math.toRadians(degrees);

    }

    private double[] angleToBasePower(double radians) {

        double degrees;
        double x;
        double y;

        double[] powers = new double[2];

        degrees = Math.toDegrees(radians);

        if(degrees >= -45 && degrees <= 45) {

            x = 1;
            y = Math.tan(-radians);

        } else if (degrees > 45 && degrees <= 135) {

            x = Math.tan(-radians + Math.toRadians(90));
            y = -1;

        } else if (degrees >= -135 && degrees < -45) {

            x = Math.tan(radians + Math.toRadians(90));
            y = 1;

        } else {

            x = -1;
            y = Math.tan(radians);

        }

        powers[0] = x;
        powers[1] = y;

        return powers;

    }
}

