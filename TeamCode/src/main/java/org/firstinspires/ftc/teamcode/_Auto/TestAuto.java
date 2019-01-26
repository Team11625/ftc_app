package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;

import java.util.Locale;

@Autonomous(name="TestAuto", group="Autonomous")
public class TestAuto extends LinearOpMode {

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    private DcMotor lift = null;

    private DcMotor armActivatorLeft = null;
    private DcMotor armActivatorRight = null;

    private BNO055IMUHeadingSensor mIMU;

    @Override
    public void runOpMode() {

        leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.get(DcMotor.class, "Lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(7);  // 7: Rev Hub face down with the word Rev facing back

        armActivatorLeft = hardwareMap.get(DcMotor.class, "armActivator");
        armActivatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(0);

        waitForStart();

        //releaseMarker();

        telemetry.addData("ticks left", armActivatorLeft.getCurrentPosition());
        telemetry.addData("ticks right", armActivatorRight.getCurrentPosition());

        requestOpModeStop();

    }

    public void releaseMarker (){
        armActivatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armActivatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armActivatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armActivatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armActivatorLeft.setTargetPosition(50);
        armActivatorRight.setTargetPosition(50);

        armActivatorLeft.setPower(.3);
        armActivatorRight.setPower(.3);

        while(armActivatorLeft.isBusy() && opModeIsActive()) {

        }

        /*armActivatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armActivatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armActivatorLeft.setTargetPosition(-200);
        armActivatorRight.setTargetPosition(-200);

        armActivatorLeft.setPower(1);
        armActivatorRight.setPower(1);

        while(armActivatorLeft.isBusy() && armActivatorRight.isBusy() && opModeIsActive()) {

        } */

        armActivatorLeft.setPower(0);
        armActivatorRight.setPower(0);

        armActivatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armActivatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveEncoder(){
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftfrontDrive.setTargetPosition(1120);
        rightfrontDrive.setTargetPosition(1120);
        leftbackDrive.setTargetPosition(1120);
        rightbackDrive.setTargetPosition(1120);

        leftfrontDrive.setPower(0.2);
        rightfrontDrive.setPower(0.2);
        leftbackDrive.setPower(0.2);
        rightbackDrive.setPower(0.2);

        while(leftfrontDrive.isBusy() && rightfrontDrive.isBusy() && leftbackDrive.isBusy() && rightbackDrive.isBusy() && opModeIsActive()) {

        }

        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
        leftbackDrive.setPower(0);
    }

    public void testLiftUp(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(1440);
        lift.setPower(1);

        while(lift.isBusy() && opModeIsActive()) {

        }

        lift.setPower(0);
    }

    public void testLiftDown(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(-1440);
        lift.setPower(1);

        while(lift.isBusy() && opModeIsActive()) {

        }

        lift.setPower(0);
    }
}
