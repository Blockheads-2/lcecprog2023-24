package org.firstinspires.ftc.teamcode.auto.cv;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class visionProDemo extends LinearOpMode {
    OpenCvCamera camera;
    aprilTagPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;

    //The following is the calibration for the C270 Webcam at 1280x720.
    double fx = 1553.14;
    double fy = 1553.14;
    double cx = 507.111;
    double cy = 363.954;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void runOpMode()
    {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagProcessor myAprilTagProcessor;

        myAprilTagProcessor = new AprilTagProcessor.Builder() // Create a new AprilTag Processor Builder object.
                .setDrawTagID(true) // Default: true, for all detections.
                .setDrawTagOutline(true) // Default: true, when tag size was provided (thus eligible for pose estimation).
                .setDrawAxes(true) // Default: false.
                .setDrawCubeProjection(true) // Default: false.
                .build(); // Create an AprilTagProcessor by calling build()

        // Optional: specify a custom Library of AprilTags.
//        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);   // The OpMode must have already created a Library

        TfodProcessor myTfodProcessor;
        myTfodProcessor = new TfodProcessor.Builder() // Create a new TFOD Processor Builder object.
                .setMaxNumRecognitions(10) // Max. number of recognitions the network will return
                .setUseObjectTracker(true) // Whether to use the object tracker
                .setTrackerMaxOverlap((float) 0.2) // Max. % of box overlapped by another box at recognition time
                .setTrackerMinSize(16) // Min. size of object that the object tracker will track
                .build(); // Create a TFOD Processor by calling build()

        VisionPortal myVisionPortal;

        myVisionPortal = new VisionPortal.Builder() // Create a new VisionPortal Builder object.
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Specify the camera to be used for this VisionPortal.
                .addProcessors(myAprilTagProcessor, myTfodProcessor) // Add the AprilTag Processor to the VisionPortal Builder.
                .setCameraResolution(new Size(1280, 720)) // Each resolution, for each camera model, needs calibration values for good pose estimation.
                .setStreamFormat(VisionPortal.StreamFormat.YUY2) // MJPEG format uses less bandwidth than the default YUY2.
                .enableLiveView(true) // Enable LiveView (RC preview).
                .setAutoStopLiveView(true) // Automatically stop LiveView (RC preview) when all vision processors are disabled.
                .build(); // Create a VisionPortal by calling build()

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new aprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }
    }
}