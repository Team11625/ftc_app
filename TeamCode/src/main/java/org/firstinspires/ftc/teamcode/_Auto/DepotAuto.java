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

@Autonomous(name="DepotAuto", group="Autonomous")
public class DepotAuto extends LinearOpMode {

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
    private DcMotor armActivatorLeft = null;
    private DcMotor armActivatorRight = null;
    private DcMotor lift = null;

    private Servo markerArm;

    private BNO055IMUHeadingSensor mIMU;

    boolean bDebug = false;

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

            lift = hardwareMap.get(DcMotor.class, "Lift");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotor.Direction.REVERSE);

            markerArm = hardwareMap.get(Servo.class, "markerArm");

            armActivatorLeft = hardwareMap.get(DcMotor.class, "armActivatorLeft");
            armActivatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armActivatorRight = hardwareMap.get(DcMotor.class, "armActivatorRight");
            armActivatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(7);  // 7: Rev Hub face down with the word Rev facing back

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        waitForStart(); //the rest of the code begins after the play button is pressed

        //sleep(1000);

        //unlatch();

        //sleep(1000);

        //sample();

        goldRight();

        requestOpModeStop(); //end of autonomous
    }

    public void goldLeft(){
        leftTurn(32.5f);
        driveEncoder(3750, 0.5);
        rightTurn(80.0f);
        driveEncoder(1250, 0.5);
        //sleep(1000);
        //releaseMarker();
        sleep(1000);
        driveEncoder(-5000, 0.5);
    }

    public void goldMiddle(){
        driveEncoder(4250, 0.5);
        sleep(1000);
        //releaseMarker();
        sleep(1000);
        driveEncoder(-2750, 0.5);
        rightTurn(75.0f);
        driveEncoder(-5000, 0.5);
    }

    public void goldRight(){
        rightTurn(30.0f);
        driveEncoder(4000, 0.5);
        leftTurn(75.0f);
        driveEncoder(2800, 0.5);
        rightTurn(90.0f);
        driveEncoder(-1750, 0.5);
        sleep(1000);
        //releaseMarker();
        sleep(1000);
        driveEncoder(-5100, 0.5);
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
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(2000);
        lift.setPower(1);

        while(lift.isBusy() && opModeIsActive()) {

        }

        lift.setPower(0);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        armActivatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armActivatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armActivatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armActivatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armActivatorLeft.setTargetPosition(2000);
        armActivatorRight.setTargetPosition(2000);

        armActivatorLeft.setPower(.5);
        armActivatorRight.setPower(.5);

        while(armActivatorLeft.isBusy() && armActivatorRight.isBusy() && opModeIsActive()) {

        }

        armActivatorLeft.setPower(0);
        armActivatorRight.setPower(0);

        armActivatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armActivatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sample(){
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
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
