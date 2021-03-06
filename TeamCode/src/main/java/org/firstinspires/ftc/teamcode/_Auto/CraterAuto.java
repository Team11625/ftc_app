package org.firstinspires.ftc.teamcode._Auto;

import android.view.ViewDebug;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
import java.util.Locale;

@Autonomous(name="CraterAuto", group="Autonomous")
public class CraterAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AUtF2Mz/////AAABmVqtyuOyskXgngp/uu1XtYRjRMd+3hU6T11OBqIw25lRNEO4Pk10Aya7fTPYH1kVFBOHxM6GCp9jIDy0HWCgeDjZzhzvYVRtRNZZigK8B04WWi+xSDYD7zJyl10v8XBm/r7EzScuIxkxalxJFTvI9Oq55eWuTOtvuYI1z7cy9etaXFcIsiXHRoNwjyn2lB0krajK3SfVSaCONBXOYqZB7vOZKf9fc8R7ZCoxvkdEMUotuVORuaVzgkLPr++dUrDlA7Z1PtgjHdNkAOv9Ai6l47gTPSpy20b+TfdbSMGBg8/dbncLUtzDYTWQBomreK/CwRAyapZLiujaSTJnvNaIjnDFZihxkSbSHBHwFkkBhEz6";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor armActivator = null;
    private DcMotor liftTop = null;
    private DcMotor liftBottom = null;

    private Servo markerArm;

    private BNO055IMUHeadingSensor mIMU;

    boolean bDebug = false;
    boolean ranFailSafe = false;

    @Override
    public void runOpMode() {
        //this code runs after the init button is pressed
        try {
            leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftbackDrive.setDirection(DcMotor.Direction.REVERSE);

            rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
            rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftTop = hardwareMap.get(DcMotor.class, "liftTop");
            liftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftBottom = hardwareMap.get(DcMotor.class, "liftBottom");
            liftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            markerArm = hardwareMap.get(Servo.class, "markerArm");

            armActivator = hardwareMap.get(DcMotor.class, "armActivator");
            armActivator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(7);  // 7: Rev Hub face down with the word Rev facing back

        liftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftTop.setTargetPosition(liftTop.getCurrentPosition());
        liftBottom.setTargetPosition(liftBottom.getCurrentPosition());

        liftTop.setPower(1);
        liftBottom.setPower(1);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        waitForStart(); //the rest of the code begins after the play button is pressed

        liftTop.setPower(0);
        liftBottom.setPower(0);

        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        unlatch();

        sleep(1000);

        sample();

        requestOpModeStop(); //end of autonomous
    }

    public void goldLeft(){
        leftTurn(34.0f);
        driveEncoder(3000, 0.75);
        leftTurn(35);
        driveEncoder(1500, 0.75);
        leftTurn(62.0f);
        driveEncoder(5000,0.75);
        sleep(1000);
        releaseMarker();
        sleep(1000);
        driveEncoder(-6200, 0.75);
    }

    public void goldMiddle(){
        driveEncoder(2500, 0.75);
        driveEncoder(-1200, 0.75);
        leftTurn(90.0f);
        driveEncoder(4500,0.75);
        leftTurn(42.5f);
        driveEncoder(1750,0.75);
        sleep(1000);
        releaseMarker();
        sleep(1000);
        driveEncoder(-4750, 0.75);
    }

    public void goldRight() {
        rightTurn(30.0f);
        driveEncoder(3000, 0.75);
        sleep(1000);
        driveEncoder(-1250, 0.75);
        leftTurn(120.0f);
        driveEncoder(5000, 0.75);
        leftTurn(44.0f);
        driveEncoder(2000, 0.75);
        sleep(1000);
        releaseMarker();
        sleep(1000);
        driveEncoder(-4500, 0.75);
    }

    public void driveEncoder(int ticks, double pow){
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftfrontDrive.setTargetPosition(ticks);
        rightfrontDrive.setTargetPosition(ticks);
        leftbackDrive.setTargetPosition(ticks);
        rightbackDrive.setTargetPosition(ticks);

        leftfrontDrive.setPower(pow);
        rightfrontDrive.setPower(pow);
        leftbackDrive.setPower(pow);
        rightbackDrive.setPower(pow);

        while(leftfrontDrive.isBusy() && rightfrontDrive.isBusy() && leftbackDrive.isBusy() && rightbackDrive.isBusy() && opModeIsActive()) {

        }


        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);

        leftfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void unlatch(){
        liftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftTop.setTargetPosition(100);
        liftBottom.setTargetPosition(100);

        liftTop.setPower(1);
        liftBottom.setPower(1);

        while(liftTop.isBusy() && liftBottom.isBusy() && opModeIsActive()) {

        }

        liftTop.setPower(0);
        liftBottom.setPower(0);

        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveEncoder(200, 0.5);

        liftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftTop.setTargetPosition(-225);
        liftBottom.setTargetPosition(-225);

        liftTop.setPower(1);
        liftBottom.setPower(1);

        while(liftTop.isBusy() && liftBottom.isBusy() && opModeIsActive()) {

        }

        liftTop.setPower(0);
        liftBottom.setPower(0);

        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveEncoder(-300, 0.2);
    }

    public void leftTurn (float turnAngle){ //left turn

        float targetAngle = mIMU.getHeading() + turnAngle;

        while(opModeIsActive()){

            if(mIMU.getHeading() >= targetAngle - 1.0f) {
                telemetry.update();
                leftfrontDrive.setPower(0);
                leftbackDrive.setPower(0);
                rightfrontDrive.setPower(0);
                rightbackDrive.setPower(0);
                break;
            }

            leftfrontDrive.setPower(-.3);
            leftbackDrive.setPower(-.3);
            rightfrontDrive.setPower(.3);
            rightbackDrive.setPower(.3);
        }
    }

    public void rightTurn (float turnAngle){

        float targetAngle = mIMU.getHeading() - turnAngle;

        while(opModeIsActive()){

            if(mIMU.getHeading() <= targetAngle + 1.0f) {
                leftfrontDrive.setPower(0);
                leftbackDrive.setPower(0);
                rightfrontDrive.setPower(0);
                rightbackDrive.setPower(0);
                break;
            }

            leftfrontDrive.setPower(.3);
            leftbackDrive.setPower(.3);
            rightfrontDrive.setPower(-.3);
            rightbackDrive.setPower(-.3);
        }
    }

    public void releaseMarker (){
        armActivator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armActivator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armActivator.setTargetPosition(300);

        armActivator.setPower(.25);

        while(armActivator.isBusy()) {

        }
        armActivator.setPower(0);
        armActivator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(500);

        armActivator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armActivator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armActivator.setTargetPosition(-215);

        armActivator.setPower(.25);

        while(armActivator.isBusy()) {

        }
        armActivator.setPower(0);
        armActivator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sample(){
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            runtime.reset();
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if(runtime.milliseconds() >= 3500 && updatedRecognitions.size() != 2 && ranFailSafe == false){
                            driveEncoder(30, 0.5);
                            ranFailSafe = true;
                        }
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 || silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX != -1) {
                                    telemetry.addData("Gold Mineral Position", "left");
                                    goldLeft();
                                } else if (goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "middle");
                                    goldMiddle();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "right");
                                    goldRight();
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
