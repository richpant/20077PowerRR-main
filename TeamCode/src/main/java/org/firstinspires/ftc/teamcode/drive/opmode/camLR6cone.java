/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name = "camLR6cone", group = "Concept") //Concept: TensorFlow Object Detection Webcam

public class camLR6cone extends LinearOpMode {

    private DigitalChannel touch;
    private DistanceSensor distanceSensor;
    double open = 0.36;
    double close = 0.5;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .60, correction;// this is where you change power for all IMU moves

    private static final String TFOD_MODEL_ASSET = "newCone.tflite";


    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "sideOne",
            "sideTwo",
            "sideThree"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ATZ2+2H/////AAABmUF68Oc56EwYrSTKYk9XtRaIhO6cdNIpwubhqKwiAvRJ440uCFpOfHLptONMzPRinvi+vxPWmzqAl4cGEpfi5irMQduEmy3JJHRqIMaMqDLkK8hzPqkoiL8+VOLv+e26RN1Bcgu8k0WD0ydq2N9KlDCgO9OLcZw4jf7P2a2EJ0o5a+6B3t0ArOLaNvPeHW+qVBPn8Ya24O8suSJwIuzZMZrbnYS1zZfaqZwxW6FUhWZzEwWq91RBHfdlISisZ97ynB+rDW01a1yAOTTcP5E5vgrvFn+Mvc2YcPHDhjWCClIYhNw8es3JbbkGJY5YQ4vGPuQJiCTs+FdM/30pThK+ZOcTiWt9D/YE8O9X3e9mcwmp";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);



        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int forward = 0;
        int zone = 0;
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        int i = 0; //added
                        boolean isOneDetected = false; //added
                        boolean isTwoDetected = false; //added
                        boolean isThreeDetected = false; //added
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                            i++; //added
                            //check label to see if camera sees bolt, this whole if statement added
                            if (recognition.getLabel().equals("sideOne")) {
                                isOneDetected = true;

                                zone = 700;
                                forward = -24;
                                telemetry.addData("Side Detected", "Side One");
                            } else {
                                isOneDetected = false; //added
                            }
                            if (recognition.getLabel().equals("sideTwo")) {
                                isTwoDetected = true;
                                forward = 21;
                                zone = 2;

                                telemetry.addData("Side Detected", "Side Two");
                            } else {
                                isTwoDetected = false; //added
                            }
                            if (recognition.getLabel().equals("sideThree")) {
                                isThreeDetected = true;
                                forward = -1;
                                zone = -700;
                                telemetry.addData("Side Detected", "side Three");
                            } else {
                                isThreeDetected = false; //added
                            }

                        }
                        telemetry.update();
                    }
                }

                Pose2d startPose = new Pose2d(-34, -70, 0);
                drive.setPoseEstimate(startPose);

                //lift.getZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                        .addTemporalMarker(() -> drive.close())
                        .waitSeconds(.1)

                        .addTemporalMarker(() -> drive.lift(3100))
                        .lineToLinearHeading(new Pose2d(-32,-19.5, Math.toRadians(-45)))
                        //.strafeLeft(63.5)
                        //.addTemporalMarker(() -> drive.lifthold(200))
                        .waitSeconds(.1)
                        .forward(9)
                        .addTemporalMarker(() -> drive.open())
                        .waitSeconds(.1)
                        .back(6.7)
                        //.strafeRight(11)
                        //.lineToLinearHeading(new Pose2d(-25,0, Math.toRadians(0)))
                        .addTemporalMarker(() -> drive.lift(650))
                        .lineToLinearHeading(new Pose2d(-60,-16, Math.toRadians(-180) + 1e-6))
                        .forward(5)
                        //.splineTo(new Vector2d(-72, -20), Math.toRadians(-180) + 1e-6)
                        .addTemporalMarker(()-> drive.close())
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> drive.lift(800))
                        .back(2)
                        .addTemporalMarker(() -> drive.lift(1800))
                        .lineToLinearHeading(new Pose2d(-47,-17, Math.toRadians(-90)))
                        .forward(8.5)
                        .addTemporalMarker(() -> drive.open())
                        //.waitSeconds(.1)
                        .back(5.5)
                        .addTemporalMarker(() -> drive.lift(650))
                        .lineToLinearHeading(new Pose2d(-60,-16, Math.toRadians(-180) + 1e-6))
                        .forward(5)
                        //.splineTo(new Vector2d(-72, -20), Math.toRadians(-180) + 1e-6)
                        .addTemporalMarker(()-> drive.close())
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> drive.lift(750))
                        .back(2)
                        .addTemporalMarker(() -> drive.lift(1950))
                        .lineToLinearHeading(new Pose2d(-47.5,-17, Math.toRadians(-90)))
                        .forward(8.5)
                        .addTemporalMarker(() -> drive.open())
                        //.waitSeconds(.1)
                        .back(5.5)
                        .addTemporalMarker(() -> drive.lift(450))
                        .lineToLinearHeading(new Pose2d(-60,-16, Math.toRadians(-180) + 1e-6))
                        .forward(5)
                        //.splineTo(new Vector2d(-72, -20), Math.toRadians(-180) + 1e-6)
                        .addTemporalMarker(()-> drive.close())
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> drive.lift(550))
                        .back(2)
                        .addTemporalMarker(() -> drive.lift(2050))
                        .lineToLinearHeading(new Pose2d(-47.5,-16.5, Math.toRadians(-90)))
                        .forward(8.75)
                        .addTemporalMarker(() -> drive.open())
                        .waitSeconds(.1)
                        .back(6)
                        .addTemporalMarker(() -> drive.lift(300))
                        .lineToLinearHeading(new Pose2d(-36.5,-21, Math.toRadians(-185)))
                        .back(forward)
                        .waitSeconds(40)


                        /*.addTemporalMarker(() -> drive.lift(470))
                        .lineToLinearHeading(new Pose2d(-60,-13, Math.toRadians(-180)))
                        .forward(6)
                        .addTemporalMarker(()-> drive.close())
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> drive.lift(4250))
                        .lineToLinearHeading(new Pose2d(-26,-15.5, Math.toRadians(90)))
                        .forward(5.5)
                        .waitSeconds(.4)
                        .addTemporalMarker(() -> drive.open())
                        .waitSeconds(.1)
                        .back(5.9)
                        .addTemporalMarker(() -> drive.lift(400))
                        .lineToLinearHeading(new Pose2d(-63,-12, Math.toRadians(-180)))
                        .forward(7.4)
                        .addTemporalMarker(()-> drive.close())
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> drive.lift(1300))
                        .back(forward)
                        .addTemporalMarker(() -> drive.lift(5))

                        /*.lineToLinearHeading(new Pose2d(-50,-19, Math.toRadians(-90)))
                        .forward(5)
                        .waitSeconds(.4)
                        .addTemporalMarker(() -> drive.open())
                        .back(5)*/
                        .waitSeconds(40)



                        //.lineToLinearHeading(new Pose2d(-37,-15, Math.toRadians(0)))


                        .build();
                waitForStart();

                if (!isStopRequested())
                    drive.followTrajectorySequence(trajSeq);

            }
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        //Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        //tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }


}
