package org.firstinspires.ftc.teamcode._Auto;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name="TestSampling", group="Autonomous")
public class TestSampling extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    boolean bDebug = false;

    private BNO055IMUHeadingSensor mIMU;

    float targetAngle;

    private static final String VUFORIA_KEY = "AUtF2Mz/////AAABmVqtyuOyskXgngp/uu1XtYRjRMd+3hU6T11OBqIw25lRNEO4Pk10Aya7fTPYH1kVFBOHxM6GCp9jIDy0HWCgeDjZzhzvYVRtRNZZigK8B04WWi+xSDYD7zJyl10v8XBm/r7EzScuIxkxalxJFTvI9Oq55eWuTOtvuYI1z7cy9etaXFcIsiXHRoNwjyn2lB0krajK3SfVSaCONBXOYqZB7vOZKf9fc8R7ZCoxvkdEMUotuVORuaVzgkLPr++dUrDlA7Z1PtgjHdNkAOv9Ai6l47gTPSpy20b+TfdbSMGBg8/dbncLUtzDYTWQBomreK/CwRAyapZLiujaSTJnvNaIjnDFZihxkSbSHBHwFkkBhEz6";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

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

        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }

        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(7);  // 7: Rev Hub face down with the word Rev facing back

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

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
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    leftTurn(30.0f);
                                    drive(1.5, 0.5);
                                } else if (goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    drive(1.5, 0.5);
                                } else {
                                    telemetry.addData("Gold Mineral Position", "The third object");
                                    rightTurn(30.0f);
                                    drive(1.5, 0.5);
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

        requestOpModeStop();
    }

    public void leftTurn (float turnAngle){

        targetAngle = mIMU.getHeading() + turnAngle;

        while(opModeIsActive()){

            if(targetAngle - mIMU.getHeading() < 0.0) {
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

        targetAngle = mIMU.getHeading() - turnAngle;

        while(opModeIsActive()){

            if(targetAngle - mIMU.getHeading() < 0.0) {
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

    public void drive(double time, double power){
        runtime.reset();
        while(runtime.seconds() < time){
            leftfrontDrive.setPower(power);
            leftbackDrive.setPower(power);
            rightfrontDrive.setPower(power);
            rightbackDrive.setPower(power);
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
