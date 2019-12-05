package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

@TeleOp
public class WebcamSkystoneEasyOpenCV extends LinearOpMode
{
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        SamplePipeline pipeline = new SamplePipeline();
        pipeline.useDefaults();
        webcam.setPipeline(pipeline);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
         * than 30FPS is not currently supported, although this will likely be addressed in a future
         * release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if(gamepad1.x)
            {
                webcam.pauseViewport();
            }
            else if(gamepad1.y)
            {
                webcam.resumeViewport();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        public DogeCVColorFilter blackFilter;
        public DogeCVColorFilter yellowFilter;
        public int blobDistanceThreshold;
        public int minimumArea;

        private Rect foundRect = new Rect();

        private Mat rawImage = new Mat();
        private Mat workingMat = new Mat();
        private Mat displayMat = new Mat();
        private Mat blackMask = new Mat();
        private Mat yellowMask = new Mat();
        private Mat hierarchy  = new Mat();

        boolean found = false;


        public SamplePipeline() {
            //detectorName = "Skystone Detector";
        }

        public Rect foundRectangle() {
            return foundRect;
        }

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(rawImage);
            input.copyTo(workingMat);
            input.copyTo(displayMat);
            input.copyTo(blackMask);
            input.copyTo(yellowMask);

            List<MatOfPoint> contoursYellow = findContours(yellowFilter, yellowMask);
            List<Rect> rectsYellow = contoursToRects(contoursYellow);
            List<List<Rect>> listOfYellowBlobs = groupIntoBlobs(rectsYellow, blobDistanceThreshold);
            Rect yellowBoundingRect = chooseBestYellow(listOfYellowBlobs);

            List<MatOfPoint> contoursBlack = findContours(blackFilter, blackMask);
            List<Rect> rectsBlack = contoursToRects(contoursBlack);
            List<Rect> rectsBlackInYellow = filterByBound(rectsBlack, yellowBoundingRect);
            List<List<Rect>> listOfBlackBlobs = groupIntoBlobs(rectsBlackInYellow, blobDistanceThreshold);
            Rect bestSkystoneRect = chooseBestBlack(listOfBlackBlobs);

            draw(contoursYellow, new Scalar(255, 150, 0));
            draw(contoursBlack, new Scalar(80, 80, 80));
            draw(yellowBoundingRect, new Scalar(255, 255, 0));

            found = bestSkystoneRect.area() > minimumArea;
            if (found) {
                draw(bestSkystoneRect, new Scalar(0, 255, 0));
                draw(getCenterPoint(bestSkystoneRect), new Scalar(0, 255, 0));
                foundRect = bestSkystoneRect;
            }

            return displayMat;
        }

        public void useDefaults() {
            blackFilter = new GrayscaleFilter(0, 50);
            yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 90);
            blobDistanceThreshold = 50;
            minimumArea = 200;
        }


        private double distance(Point a, Point b) {
            return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
        }

        private Point getCenterPoint(Rect rect) {
            return new Point(rect.x + rect.width/2, rect.y + rect.height/2);
        }

        private Rect boundingRect(List<Rect> rects) {
            int minX = 999;
            int minY = 999;
            int maxX = 0;
            int maxY = 0;
            for (Rect rect : rects) {
                minX = Math.min(rect.x, minX);
                minY = Math.min(rect.y, minY);
                maxX = Math.max(rect.x + rect.width, maxX);
                maxY = Math.max(rect.y + rect.height, maxY);
            }

            return new Rect(minX, minY, maxX - minX, maxY - minY);
        }

        private List<MatOfPoint> findContours(DogeCVColorFilter filter, Mat mask) {
            filter.process(workingMat.clone(), mask);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            return contours;
        }


        private void draw(Rect rect, Scalar color) {
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), color, 2);
        }

        private void draw(Point point, Scalar color) {
            Imgproc.circle(displayMat, point, 2, color);
        }

        private void draw(List<MatOfPoint> contours, Scalar color) {
            // Not draw contours for now, since can look messy
            Imgproc.drawContours(displayMat, contours, -1, color, 2);
        }


        private Rect chooseBestYellow(List<List<Rect>> listOfYellowBlobs) {
            Rect bestYellowRect = new Rect();
            for (List<Rect> blob : listOfYellowBlobs) {
                Rect blobBound = boundingRect(blob);
                draw(blobBound, new Scalar(255 , 100, 0));

                if (blobBound.area() > bestYellowRect.area()) {
                    bestYellowRect = blobBound;
                }
            }
            return bestYellowRect;
        }

        private Rect chooseBestBlack(List<List<Rect>> listOfBlackBlobs) {
            Rect bestBlackRect = new Rect();
            for (List<Rect> blob : listOfBlackBlobs) {
                Rect blobBound = boundingRect(blob);
                draw(blobBound, new Scalar(0, 150, 0));

                if (blobBound.area() > bestBlackRect.area()) {
                    bestBlackRect = blobBound;
                }
            }
            return bestBlackRect;
        }

        private  List<Rect> contoursToRects(List<MatOfPoint> contours) {
            List<Rect> rects = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                rects.add(Imgproc.boundingRect(contour));
            }
            return rects;
        }

        private List<List<Rect>> groupIntoBlobs(List<Rect> rects, int blobDistanceThreshold) {
            List<List<Rect>> listOfBlobs = new ArrayList<>();
            List<Rect> unusedRects = new ArrayList<>(rects);

            while (!unusedRects.isEmpty()) {
                LinkedList<Rect> toProcess = new LinkedList<>();
                toProcess.add(unusedRects.remove(0));
                List<Rect> currentBlob = new ArrayList<>();
                while (!toProcess.isEmpty()) {
                    Rect currentRect = toProcess.poll();
                    currentBlob.add(currentRect);

                    for (int i = 0; i < unusedRects.size(); i++) {
                        if (distance(getCenterPoint(currentRect), getCenterPoint(unusedRects.get(i))) < blobDistanceThreshold) {
                            toProcess.add(unusedRects.remove(i));
                            i--;
                        }
                    }
                }
                listOfBlobs.add(currentBlob);
            }

            return listOfBlobs;
        }

        private List<Rect> filterByBound(List<Rect> rects, Rect boundingRect) {
            List<Rect> rectsInsideBound = new ArrayList<>();
            for (Rect rect : rects) {
                if (boundingRect.contains(getCenterPoint(rect))) {
                    rectsInsideBound.add(rect);
                }
            }
            return rectsInsideBound;
        }
    }
}
