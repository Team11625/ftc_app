package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="LatchMotorControl", group="Test")

public class LatchMotorControl extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null; //if you see this slap yourself
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    private DcMotor arm = null; //the arm that captures the blocks and balls, controlled by left hand motor
    private DcMotor armActivatorLeft = null;
    private DcMotor armActivatorRight = null;

    private DcMotor latchMotor = null;

    private Servo markerArm;

    private BNO055IMU imu;

    boolean bDebug = false;

    @Override
    public void init() {

        try {
            leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);


            leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
            rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightbackDrive.setDirection(DcMotor.Direction.REVERSE);

            arm = hardwareMap.get(DcMotor.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotor.Direction.REVERSE);

            armActivatorLeft = hardwareMap.get(DcMotor.class, "armActivatorLeft");
            armActivatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armActivatorLeft.setDirection(DcMotor.Direction.REVERSE);

            armActivatorRight = hardwareMap.get(DcMotor.class, "armActivatorRight");
            armActivatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            markerArm = hardwareMap.get(Servo.class, "markerArm");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {
            //---Latch code by Kale---\\
            if (gamepad1.dpad_up = true) {
                runtime.reset();
                while (runtime < 1) {
                    latchMotor.setPower(.5);
                }
                latchMotor.setPower(0);
            }
            if (gamepad1.dpad_down = true) {
                runtime.reset();
                while (runtime < 1) {
                    latchMotor.setPower(-.5);
                }
                latchMotor.setPower(0);
            }
    }
}