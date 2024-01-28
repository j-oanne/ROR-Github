package org.firstinspires.ftc.teamcode;
import android.util.Size;
import java.util.*;

// Position
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

// Opmodes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Hardware
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

// Road Runner
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// Vision
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "blueFront")
@Config

public class blueFront extends LinearOpMode {

    // Configurations
    public static double DELAY = 0.2;
    public static boolean SLIDE = true;
    public static boolean CAMERA = true;


    public static double DIFF = -1;
    private double x_offset = 2.375;
    private double y_offset = 2.675;
    private Pose2d start;

    // Hardware configuration
    private SampleMecanumDrive drive;
    public Servo outtake;
    public DcMotorEx linearSlide;
    public TouchSensor touch;

    // Vision and tensorflow
    private static final boolean USE_WEBCAM = true;          // True for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/blueProp4.tflite";
    private static final String[] LABELS = {"blueProp",};     // Define the labels recognized in the model for TFOD (must be in training order!)
    private TfodProcessor tfod;                 // The variable to store our instance of the TensorFlow Object Detection processor.
    private VisionPortal visionPortal;          // The variable to store our instance of the vision portal.

    private static final Vector2d[][] FIELD = initField();

    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        drive = new SampleMecanumDrive(hardwareMap);
        outtake = hardwareMap.get(Servo.class, "yury");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        touch = hardwareMap.touchSensor.get("touchSensor");

        // Reset hardware positions and power
        outtake.setPosition(1);
        linearSlide.setPower(0);

        // Determine location of spike mark using vision
        String dir = "Middle";

        if (CAMERA) {

            initTfod();

            while (!opModeIsActive()) {
                dir = telemetryTfod();
                telemetry.update();
                sleep(20);
            }

            telemetry.update();
            visionPortal.close();
        }

        // Start program
        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // Set initial starting position
        start = new Pose2d(convertLocToVec("A4").plus(new Vector2d(x_offset,y_offset)), Math.toRadians(270));
        drive.setPoseEstimate(start);

        // Run autonomous path
        if (dir.equals("Middle")) { // middle
            middle();
        }
        else if (dir.equals("Left")) { // Left
            left();
        }
        else { // Right: default
            right();
        }

        telemetry.addData("Pose: ", drive.getPoseEstimate());
        telemetry.update();
    }




    // ***************************************************************************************
    private void middle() {

        if (DIFF == -1) {
            DIFF = 5;
        }

        Vector2d spike = convertLocToVec("B4").plus(new Vector2d(x_offset,-11));
        Vector2d board = convertLocToVec("B6").plus(new Vector2d(-14,.25));

        // Trajectory from start to backdrop
        TrajectorySequence t = drive.trajectorySequenceBuilder(start)

                // Move toward spike mark
                .lineTo(spike)
                .waitSeconds(DELAY)

                // Back to starting coordinate
                .lineToSplineHeading(new Pose2d(convertLocToVec("A4").plus(new Vector2d(x_offset, 0)), Math.toRadians(270)))
                .waitSeconds(DELAY)

                // Turn
                .turn(Math.toRadians(-100))
                .waitSeconds(DELAY)

                // Move to the front of the backboard
                .lineToLinearHeading(new Pose2d(board, Math.toRadians(180+DIFF)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // Raise the linear slide
        if (SLIDE) {
            raiseSlide();
        }

        // Path to move closer to the backdrop
        t = drive.trajectorySequenceBuilder(t.end())
                .lineToLinearHeading(new Pose2d(board.plus(new Vector2d(8,0)), Math.toRadians(180+DIFF)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // allow it to steady
        sleep(1000);

        // Outtake the yellow pixel onto the backdrop
        rotateOuttakeServo();
        rotateOuttakeServo();
        rotateOuttakeServo();

        // allow it to steady
        sleep(1000);

        // Path to move away from the backdrop
        t = drive.trajectorySequenceBuilder(t.end())
                .lineToLinearHeading(new Pose2d(board, Math.toRadians(180)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // allow it to steady
        sleep(1000);

        // Lower the linear slide
        if (SLIDE) {
            lowerSlide();
        }

        // allow it to steady
        sleep(1000);

        // Start of path from backdrop to parking
        t = drive.trajectorySequenceBuilder(t.end())

                // Back up to another coordinate square
                .lineToLinearHeading(new Pose2d(convertLocToVec("A5").plus(new Vector2d(0,1.5)), Math.toRadians(180)))
                .waitSeconds(DELAY)

                // Park on the edge
                .lineTo(convertLocToVec("A6").plus(new Vector2d(0,2)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);
    }
    // ***************************************************************************************


    // ***************************************************************************************
    private void left() {

        if (DIFF == -1) {
            DIFF = 3;
        }

        Vector2d spike = convertLocToVec("B4").plus(new Vector2d(14,-6));
        Vector2d board = convertLocToVec("B6").plus(new Vector2d(-14,8));

        // Trajectory from start to backdrop
        TrajectorySequence t = drive.trajectorySequenceBuilder(start)

                // Move toward spike mark
                .lineToLinearHeading(new Pose2d(spike, Math.toRadians(270)))
                .waitSeconds(DELAY)

                .back(10)
                .waitSeconds(DELAY)

                // Back to starting coordinate
                .lineToSplineHeading(new Pose2d(convertLocToVec("A4").plus(new Vector2d(x_offset, 0)), Math.toRadians(270)))
                .waitSeconds(DELAY)

                // Turn
                .turn(Math.toRadians(-100))
                .waitSeconds(DELAY)

                // Move to the front of the backboard
                .lineToLinearHeading(new Pose2d(board, Math.toRadians(180+DIFF)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // Raise the linear slide
        if (SLIDE) {
            raiseSlide();
        }

        // Path to move closer to the backdrop
        t = drive.trajectorySequenceBuilder(t.end())
                .lineToLinearHeading(new Pose2d(board.plus(new Vector2d(8,0)), Math.toRadians(180+DIFF)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // Allow it to steady
        sleep(1000);

        // Outtake the yellow pixel onto the backdrop
        rotateOuttakeServo();
        rotateOuttakeServo();
        rotateOuttakeServo();

        // Allow it to steady
        sleep(1000);

        // Path to move away from the backdrop
        t = drive.trajectorySequenceBuilder(t.end())
                .lineToLinearHeading(new Pose2d(board, Math.toRadians(180)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // Allow it to steady
        sleep(1000);

        // Lower the linear slide
        if (SLIDE) {
            lowerSlide();
        }

        // Allow it to steady
        sleep(1000);

        // Start of path from backdrop to parking
        t = drive.trajectorySequenceBuilder(t.end())

                // Back up to another coordinate square
                .lineToLinearHeading(new Pose2d(convertLocToVec("A5").plus(new Vector2d(0,1.5)), Math.toRadians(180)))
                .waitSeconds(DELAY)

                // Park on the edge
                .lineTo(convertLocToVec("A6").plus(new Vector2d(0,2)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);
    }
    // ***************************************************************************************



    // ***************************************************************************************
    private void right() {

        Vector2d spike = convertLocToVec("B4").plus(new Vector2d(-5,0));
        Vector2d board = convertLocToVec("B6").plus(new Vector2d(-7,-8));

        // Trajectory from start to backdrop
        TrajectorySequence t = drive.trajectorySequenceBuilder(start)

                .lineTo(convertLocToVec("B4").plus(new Vector2d(x_offset,2)))

                // Move toward spike mark
                .turn(Math.toRadians(-90))
                .waitSeconds(DELAY)

                .lineToSplineHeading(new Pose2d(spike, Math.toRadians(180)))

                // Move to the front of the backboard
                .lineToLinearHeading(new Pose2d(board, Math.toRadians(180)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // Raise the linear slide
        if (SLIDE) {
            raiseSlide();
        }

        // Path to move closer to the backdrop
        t = drive.trajectorySequenceBuilder(t.end())
                .lineToLinearHeading(new Pose2d(board.plus(new Vector2d(5,0)), Math.toRadians(180)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // Outtake the yellow pixel onto the backdrop
        rotateOuttakeServo();

        // Path to move away from the backdrop
        t = drive.trajectorySequenceBuilder(t.end())
                .lineToLinearHeading(new Pose2d(board, Math.toRadians(180)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);

        // Lower the linear slide
        if (SLIDE) {
            lowerSlide();
        }

        // Start of path from backdrop to parking
        t = drive.trajectorySequenceBuilder(t.end())

                // Back up to another coordinate square
                .lineToLinearHeading(new Pose2d(convertLocToVec("A5"), Math.toRadians(180)))
                .waitSeconds(DELAY)

                // Park on the edge
                .lineTo(convertLocToVec("A6").plus(new Vector2d(3,0)))
                .waitSeconds(DELAY)
                .build();
        drive.followTrajectorySequence(t);
    }
    // ***************************************************************************************


    // ***************************************************************************************
    private void raiseSlide() {
        while(!touch.isPressed()) {
            linearSlide.setPower(.60);
        }
        while(touch.isPressed()){
            linearSlide.setPower(-.1);
        }
        linearSlide.setPower(0);
    }
    private void lowerSlide() {
        while (!touch.isPressed()) {
            linearSlide.setPower(-.4);
        }
        while (touch.isPressed()) {
            linearSlide.setPower(.1);
        }
        linearSlide.setPower(0);
    }
    private void rotateOuttakeServo() {
        outtake.setPosition(0);
        sleep(200);
        outtake.setPosition(1);
        sleep(200);
    }
    // ***************************************************************************************



    // ***************************************************************************************
    // Initialize the TensorFlow Object Detection processor.
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.6f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    // Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
    private String telemetryTfod() {

        // Right is default location
        String dir = "Right";

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        if (currentRecognitions.size() == 0){
            telemetry.addData("Position", "Right");
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if(x < 360) {
                telemetry.addData("Position", "Left");
                dir = "Left";
            } else {
                telemetry.addData("Position", "Middle");
                dir = "Middle";
            }
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        return dir;
    }   // end method telemetryTfod()
    // ***************************************************************************************


    // ***************************************************************************************
    private Vector2d convertLocToVec(String loc) {

        // Reference field images
        int row = 6-Integer.parseInt(loc.substring(1));
        int col = (int)loc.charAt(0)-65; //A=0, B=1, C=2, ...

        return FIELD[row][col];
    } // end of convertLocToVec(String loc)
    private static Vector2d[][] initField() {

        Vector2d[][] field = new Vector2d[6][6]; // Create field matrix
        Vector2d start = new Vector2d(58.75, 58.75); // Starting loc: A6

        Vector2d cur = start;
        for (int row = 0; row < 6; row++) {
            for (int col = 0; col < 6; col++) {
                field[row][col] = cur;
                cur = cur.minus(new Vector2d(0,24.25));
            }
            start = start.minus(new Vector2d(24, 0));
            cur = start;
        }

        return field;
    } // end of initField()
    // ***************************************************************************************

} // end class