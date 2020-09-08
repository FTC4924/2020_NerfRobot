package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TopDown2DDrive")
public class TopDown2DDrive extends OpMode {

    BNO055IMU imu;
    Orientation angles;

    private DcMotor middleWheelMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private double[] powers;

    private double gamepad1LeftStickX;
    private double gamepad1LeftStickY;
    private double gamepad1RightStickX;

    private double angle;
    private double radius;

    @Override
    public void init() {

        middleWheelMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        powers = new double[3];

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    @Override
    public void loop() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x/2;

        radius = xyToRadius(gamepad1LeftStickX, gamepad1LeftStickY);

        angle = xyToAngle(gamepad1LeftStickX, gamepad1LeftStickY);
        angle = angleReducer(angle, angles.firstAngle);

        powers = angleToBasePower(angle);

        for(int i = 0; i < powers.length; i++) {

            powers[i] *= radius;

        }

        if(gamepad1RightStickX > 0.05){
            if(powers[2] > 1 - gamepad1RightStickX) {

                powers[2] -= powers[1] + gamepad1RightStickX - 1 + gamepad1RightStickX;
                powers[1] = 1;

            } else if(powers[2] < -1 + gamepad1RightStickX) {

                powers[1] -= powers[2] - gamepad1RightStickX + 1 - gamepad1RightStickX;
                powers[2] = -1;

            } else {

                powers[1] += gamepad1RightStickX;
                powers[2] -= gamepad1RightStickX;

            }
        } else if(gamepad1RightStickX < -0.05) {
            if (powers[2] > 1 + gamepad1RightStickX) {

                powers[1] -= powers[2] - gamepad1RightStickX - 1 - gamepad1RightStickX;
                powers[2] = 1;

            } else if (powers[2] < -1 - gamepad1RightStickX) {

                powers[2] -= powers[1] + gamepad1RightStickX + 1 + gamepad1RightStickX;
                powers[1] = -1;

            } else {

                powers[2] -= gamepad1RightStickX;
                powers[1] += gamepad1RightStickX;

            }
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

    private double angleReducer(double radians, double currentAngle) {

        double degrees = Math.toDegrees(radians);

        degrees -= currentAngle;

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

        double[] powers = new double[3];

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
        powers[2] = y;

        return powers;

    }
}

