package org.firstinspires.ftc.teamcode.core.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.Subsystem;
//import org.firstinspires.ftc.teamcode.util.CameraConfig;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Util;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class EyeAll extends Subsystem
{
    private OpenCvWebcam webcam;
    private PropsDetectionPipeline propsDetectionPipeline;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private HardwareMap hardwareMap;
    
    //public static CameraConfig.CameraConfigData cameraConfig = new CameraConfig.CameraConfigData();
    
    public boolean IsOpen = false;
    
    public enum TargetObject
    {
        RED_CONE,
        BLUE_CONE,
        BLACK_DISK,
        NONE
    }
    
    public EyeAll(HardwareMap map)
    {
        super(map);
        hardwareMap = map;
    }
    
    @Override
    public void teleopInit(Subsystem otherSys)
    {
    }
    
    @Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        /*if(gamepad1.a)
        {
            webcam.stopStreaming();
            //webcam.closeCameraDevice();
        }*/
    }
    
    @Override
    public void autoInit()
    {
        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
    
        // UNITS ARE METERS
        double tagsize = 0.166;
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, Constants.frontWebcamera), cameraMonitorViewId);
    
//        if( null == CameraConfig.ReadConfigFromFile(cameraConfig))
//        {
//            // do the camera calibration first
//            stop();
//        }
    
        propsDetectionPipeline = new PropsDetectionPipeline();
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    
    }
    
    @Override
    public void autoControls(boolean isOpActive)
    {
    
    }
    
    @Override
    public void stop()
    {
        IsOpen = false;
        webcam.closeCameraDevice();
    }
    
    @Override
    public void crossSubsystemCheck()
    {
    
    }
    
    @Override
    public String addTelemetry()
    {
        /*
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
    
        if(isYellow)
        {
            telemetry.addData("Pole Position", yellowPipeline.getLocation_leftright());
            telemetry.addData("top_left", yellowPipeline.top_left_x);
            telemetry.addData("top_right", yellowPipeline.top_right_x);
            telemetry.addData("bottom_left", yellowPipeline.bottom_left_x);
            telemetry.addData("bottom_right", yellowPipeline.bottom_right_x);
            telemetry.addData("pole angle", yellowPipeline.line_angle);
        }
        else
        {
            telemetry.addData("Num contours found", redPipeline.getNumContoursFound());
        }*/
        return null;
    }
    
    public void OpenEyeToReadAprilTag()
    {
        if(IsOpen)
            return;
        
        webcam.setPipeline(aprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(Constants.CameraViewWidth, Constants.CameraViewHeight,
                        OpenCvCameraRotation.UPSIDE_DOWN);
                IsOpen = true;
            }
    
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                IsOpen = false;
            }
        });
    }
    
    public void OpenEyeToFindProps()
    {
        if(IsOpen)
            return;
        
        webcam.setPipeline(propsDetectionPipeline);
        
        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(Constants.CameraViewWidth, Constants.CameraViewHeight,
                        OpenCvCameraRotation.UPSIDE_DOWN);
                IsOpen = true;
            }
            
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                IsOpen = false;
            }
        });
    }
    
    public void CloseEye()
    {
        IsOpen = false;
        //webcam.closeCameraDeviceAsync();
        webcam.stopStreaming();
    }
    
    // backdoor to pock the data into system
//    public void UpdateConfigDataLoop(CameraConfig.CameraConfigData config)
//    {
//        cameraConfig.Copy(config);
//    }
    
    public enum ObjectLocation
    {
        UNKNOWN, // init, or something wrong
        CENTER,
        ON_CAMERA_LEFT,
        ON_CAMERA_RIGHT,
        FAR_TO_CAMERA,
        NEAR_TO_CAMERA,
        ABOVE_THE_CLAW,
        BELOW_THE_CLAW,
        WAITING // still within loop of the checking
    }
    
    public static class AnalyzedObject
    {
        public TargetObject name = TargetObject.NONE;
        public double angle_avg = -90;
        public int center_x_avg = -1;
        public int center_y_avg = -1;
        public int width_avg = 0;
        public int height_avg = 0;
        ObjectLocation lrPosition = ObjectLocation.UNKNOWN;
        ObjectLocation fbPosition = ObjectLocation.UNKNOWN;
        
        public int retryIdx = 0;
        public int[] width = new int[RETRY_MAX];
        public int[] center_x = new int[RETRY_MAX];
    }
    final static int RETRY_MAX = 3;
    
    public static AnalyzedObject objectFoundInfo = new AnalyzedObject();
    
    public void SetOpInfoOnScreen(String mode)
    {
        propsDetectionPipeline.SetStatusInfo(mode);
    }
    
    static final double FEET_PER_METER = 3.28084;
    
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    
    // UNITS ARE METERS
    double tagsize = 0.166;
    
    int numFramesWithoutDetection = 0;
    
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    
    public int ReadParkingID()// need timeout
    {
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
    
        // If there's been a new frame...
        if(detections == null)
        {
            return -1;
        }
        else
        {
            //telemetry.addData("FPS", camera.getFps());
            //telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
            //telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
        
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
                return -1;
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
            
                /*for(AprilTagDetection detection : detections)
                {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                }*/
                return detections.get(0).id;
            }
        }
    }
    
//    public ObjectLocation CheckObjectLocation(TargetObject newtarget)
//    {
//        ObjectLocation result = ObjectLocation.UNKNOWN;
//
//        if(objectFoundInfo.name != newtarget)
//        {
//            objectFoundInfo.name = newtarget;
//        }
//
//        if(newtarget == TargetObject.RED_CONE || newtarget == TargetObject.BLUE_CONE)
//        {
//            result = CheckConeOnCenterLoop();
//        }
//
//        return result;
//    }

    public ObjectLocation CheckPropsLocation(TargetObject newtarget)
    {
        ObjectLocation result = ObjectLocation.UNKNOWN;

        if(objectFoundInfo.name != newtarget)
        {
            objectFoundInfo.name = newtarget;
        }

        if(newtarget == TargetObject.RED_CONE )  //newtarget == TargetObject.BLUE_CONE
        {
            result = CheckPropsLocationLoop();
        }
        else // to be fixed, testing only
        {
            result = ObjectLocation.ON_CAMERA_LEFT;
        }

        return result;
    }
    
    int countTotalFB = 0;
    int countCenterFB = 0;
    int countTotalLR = 0;
    int countCenterLR = 0;
    int countRIGHT = 0;
    int countLEFT = 0;
    int countFRONT = 0;
    int countBACK = 0;
    private void resetCount()
    {
        /*countTotalFB = 0;
        countCenterFB = 0;
        countTotalLR = 0;
        countCenterLR = 0;
        countRIGHT = 0;
        countLEFT = 0;
        countFRONT = 0;
        countBACK = 0;*/
        objectFoundInfo.angle_avg = -90;
        objectFoundInfo.center_x_avg = 0;
        objectFoundInfo.center_y_avg = 0;
        objectFoundInfo.width_avg = 0;
        objectFoundInfo.height_avg = 0;
        objectFoundInfo.lrPosition = ObjectLocation.UNKNOWN;
        objectFoundInfo.fbPosition = ObjectLocation.UNKNOWN;
        objectFoundInfo.retryIdx = 0;
        for(int i = 0; i < RETRY_MAX; i++)
        {
            objectFoundInfo.width[i] = 0;
            objectFoundInfo.center_x[i] = 0;
        }
    }

    static final int ConePointAX = 100; // to be calibrated
    static final int ConePointBX = 300; // to be calibrated

    static final int ConePointAY = 0;
    static final int ConePointBY = 0;
    private ObjectLocation CheckPropsLocationLoop()
    {
        if(!doneInitCone)
        {
            doneInitCone = true;
            resetCount();
        }
        countTotalLR++;

        int centero = Util.inBoundary(objectFoundInfo.center_x_avg,
                ConePointAX, ConePointBX);
        if( centero == 0)
        {
            countCenterLR++;
        }
        else if(centero== -1 )
        {
            countLEFT++;
        }
        else
        {
            countRIGHT++;
        }

        if(countTotalLR < RETRY_MAX)
        {
            objectFoundInfo.lrPosition = ObjectLocation.WAITING;
        }
        else
        {
            if (countCenterLR >= countTotalLR * 0.6)
            {
                objectFoundInfo.lrPosition = ObjectLocation.CENTER;
            }
            else
            {
                if(countRIGHT > countLEFT)
                {
                    objectFoundInfo.lrPosition = ObjectLocation.ON_CAMERA_RIGHT;
                }
                else
                {
                    objectFoundInfo.lrPosition = ObjectLocation.ON_CAMERA_LEFT;
                }
            }
            countTotalLR = 0;
            countCenterLR = 0;
            countRIGHT = 0;
            countLEFT = 0;
        }
        return objectFoundInfo.lrPosition;
    }


    boolean doneInitCone = false;
//    private ObjectLocation CheckConeOnCenterLoop()
//    {
//        if(!doneInitCone)
//        {
//            doneInitCone = true;
//            resetCount();
//        }
//        countTotalLR++;
//
//        int centero = Util.inBoundary(objectFoundInfo.center_x_avg,
//                cameraConfig.ConePointAX, cameraConfig.ConePointBX);
//        if( centero == 0)
//        {
//            countCenterLR++;
//        }
//        else if(centero== -1 )
//        {
//            countLEFT++;
//        }
//        else
//        {
//            countRIGHT++;
//        }
//
//        if(countTotalLR < RETRY_MAX)
//        {
//            objectFoundInfo.lrPosition = ObjectLocation.WAITING;
//        }
//        else
//        {
//            if (countCenterLR >= countTotalLR * 0.6)
//            {
//                objectFoundInfo.lrPosition = ObjectLocation.CENTER;
//            }
//            else
//            {
//                if(countRIGHT > countLEFT)
//                {
//                    objectFoundInfo.lrPosition = ObjectLocation.ON_CAMERA_RIGHT;
//                }
//                else
//                {
//                    objectFoundInfo.lrPosition = ObjectLocation.ON_CAMERA_LEFT;
//                }
//            }
//            countTotalLR = 0;
//            countCenterLR = 0;
//            countRIGHT = 0;
//            countLEFT = 0;
//        }
//
//        countTotalFB++;
//        int widthp = Util.inBoundary(objectFoundInfo.width_avg,
//                cameraConfig.ConeWidth, cameraConfig.ConeWidth*1.2);
//        if( 0 == widthp)
//        {
//            countCenterFB++;
//        }
//        else if( -1 == widthp)
//        {
//            countFRONT++;
//        }
//        else
//        {
//            countBACK++;
//        }
//
//        if(countTotalFB < RETRY_MAX)
//        {
//            objectFoundInfo.fbPosition = ObjectLocation.WAITING;
//        }
//        else
//        {
//            if (countCenterFB >= countTotalFB * 0.6)
//            {
//                objectFoundInfo.fbPosition = ObjectLocation.CENTER;
//            }
//            else
//            {
//                if(countBACK > countFRONT)
//                {
//                    objectFoundInfo.fbPosition = ObjectLocation.NEAR_TO_CAMERA;
//                }
//                else
//                {
//                    objectFoundInfo.fbPosition = ObjectLocation.FAR_TO_CAMERA;
//                }
//            }
//            countTotalFB = 0;
//            countCenterFB = 0;
//            countBACK = 0;
//            countFRONT = 0;
//        }
//
//        if(objectFoundInfo.lrPosition != ObjectLocation.CENTER )
//            return objectFoundInfo.lrPosition;
//        else
//        {
//            return  objectFoundInfo.fbPosition;
//        }
//    }
    
    
    public static class PropsDetectionPipeline extends TimestampedOpenCvPipeline //OpenCvPipeline
    {
        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar YELLOW = new Scalar(255, 255, 0);
        
        //Point stageTextAnchor;
        //Point timeTextAnchor;
        Point coneConfigTextAnchor;
        Point configStateTextAnchor;
        
        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 170;  // 102, 80,180
        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
        static final int CONTOUR_LINE_THICKNESS = 2;
        
        String KeypadAdjTask = "";
        public void SetStatusInfo(String info)
        {
            KeypadAdjTask = info;
        }
        
        /*
         * Our working image buffers
         */
        Mat yCbCrChanMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();
        
        
    
        //private TargetOjbect currentObjectToFinding = TargetOjbect.NONE;
        //public void SetFindingObjectTarget(TargetOjbect objectType)
        //{
        //    currentObjectToFinding = objectType;
        //}
        
        int numContoursFound;
    
        /*
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109, 98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181, 98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253, 98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;
        
        int avg1, avg2, avg3;
        
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    
        Point targetConePointA;
        Point targetConePointB;

        
        Point locationTextAnchorLRPoint;
        Point locationTextAnchorFBPoint;
        
        Point pathCenterTextAnchorPoint;
        Point pathLinePointA;
        Point pathLinePointB;
        
        ElapsedTime runtime = new ElapsedTime();
        
        @Override
        public void init(Mat mat)
        {
            targetConePointA = new Point(
                    ConePointAX,
                    ConePointAY);
            targetConePointB = new Point(
                    ConePointBX,
                    ConePointBY);
            //timeTextAnchor = new Point(0, 15);
            //stageTextAnchor = new Point(540, mat.height()-10);
            coneConfigTextAnchor = new Point(0, 30);
            configStateTextAnchor = new Point(400, mat.height()-15);
            locationTextAnchorLRPoint = new Point(0, 150);
            locationTextAnchorFBPoint = new Point(0, 165);
            pathCenterTextAnchorPoint = new Point(Constants.CameraViewWidth-150,
                    Constants.CameraViewHeight/2);
//            pathLinePointA = new Point(cameraConfig.PathCenterX, 0);
//            pathLinePointB = new Point(cameraConfig.PathCenterX, Constants.CameraViewHeight);
            runtime.reset();
        }
    
        @Override
        public void onViewportTapped()
        {
        }
    
        enum Stage
        {
            YCbCr_CHAN,
            THRESHOLD,
            MORPHED,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }
    
        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN;
        private Stage[] stages = Stage.values();
        
        @Override
        public Mat processFrame(Mat input, long captureTimeNanos)
        {
            // don't use it while game. TODO debug
            if (runtime.seconds() > 3)
            {
                int currentStageNum = stageToRenderToViewport.ordinal();
                int nextStageNum = currentStageNum + 1;
                if (nextStageNum >= stages.length)
                {
                    nextStageNum = 0;
                }
                stageToRenderToViewport = stages[nextStageNum];
                runtime.reset();
            }
    
            if(objectFoundInfo.name == TargetObject.NONE)
            {
                return input;
            }
            else //(currentFinding == TargetOjbect.CONE)
            {
                for (MatOfPoint contour : findContoursCone(input))
                {
                    analyzeContour(contour, input);
                }
            }
    
            // don't use it while game. TODO debug
            DisplayInfo(input);
    
            //https://gist.github.com/razimgit/d9c91edfd1be6420f58a74e1837bde18
            
            
            switch (stageToRenderToViewport)
            {
                /*case YCbCr_CHAN:
                {
                    return yCbCrChanMat;
                }
    
                case THRESHOLD:
                {
                    return thresholdMat;
                }
    
                case MORPHED:
                {
                    return morphedThreshold;
                }
    
                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    return contoursOnPlainImageMat;
                }*/
    
                default: //RAW_IMAGE
                {
                    return input;
                }
            }
        }
    
        private void DisplayInfo(Mat input)
        {
            targetConePointA = new Point(
                    ConePointAX,
                    ConePointAY);
            targetConePointB = new Point(
                    ConePointBX,
                    ConePointBY);
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    targetConePointA, // First point which defines the rectangle
                    targetConePointB, // Second point which defines the rectangle
                    PURPLE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
    
            String lb = String.format("A(%d,%d)", ConePointAX, ConePointAY);
            Imgproc.putText(input, lb,
                    targetConePointA, Imgproc.FONT_HERSHEY_PLAIN, // Font
                    0.7, // Font size
                    GREEN, // Font color
                    1);
            lb = String.format("B(%d,%d)", ConePointBX, ConePointBY);
            Imgproc.putText(input, lb,
                    targetConePointB, Imgproc.FONT_HERSHEY_PLAIN, // Font
                    0.7, // Font size
                    GREEN, // Font color
                    1);

    
//            lb = String.format("Cone Target - Width: %d-%d, Red Color: %d, Blue Color: %d",
//                    cameraConfig.ConeWidth, (int)(cameraConfig.ConeWidth*1.2),
//                    cameraConfig.Red, cameraConfig.Blue);
//            Imgproc.putText(input, lb,
//                    coneConfigTextAnchor,
//                    Imgproc.FONT_HERSHEY_PLAIN, // Font
//                    1, // Font size
//                    PURPLE, // Font color
//                    1);

    
            lb = String.format("Task: " + KeypadAdjTask);
            Imgproc.putText(input, lb,
                    configStateTextAnchor,
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    RED, // Font color
                    1);
    
            lb = String.format("Left/Right: " + objectFoundInfo.lrPosition);
            Imgproc.putText(input, lb,
                    locationTextAnchorLRPoint,
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    RED, // Font color
                    1);
            lb = String.format("Front/Back: " + objectFoundInfo.fbPosition);
            Imgproc.putText(input, lb,
                    locationTextAnchorFBPoint,
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    RED, // Font color
                    1);
//
//            lb = String.format("Path Center: " + cameraConfig.PathCenterX);
//            Imgproc.putText(input, lb,
//                    pathCenterTextAnchorPoint,
//                    Imgproc.FONT_HERSHEY_PLAIN, // Font
//                    1, // Font size
//                    GREEN, // Font color
//                    1);
//            pathLinePointA = new Point(cameraConfig.PathCenterX, 0);
//            pathLinePointB = new Point(cameraConfig.PathCenterX, Constants.CameraViewHeight);
//            Imgproc.line(input, pathLinePointA, pathLinePointB, GREEN);
        }
        
        /*Color Y Value Cr Value Cb Value
        White 235 (EB) 128 (80) 128 (80)
        Black 16 (10) 128 (80) 128 (80)
        Red 81 (51) 240 (F0) 90 (5A)
        Green 145 (91) 34 (22) 54 (36)
        Blue 41 (29) 110 (6E) 240 (F0)
        Yellow 210 (D2) 146 (92) 16 (10)
        Cyan 170 (AA) 16 (10) 166 (A6)
        Magenta 106 (6A) 222 (DE) 202 (CA)
        
        Note that:
        1. The Y value ranges from 16 to 235 (220 levels), with 16 being black.
        2. The Cr and Cb values range from 16 to 240 with 128 as the mid-point.
        */
    
        /*
         * The elements we use for noise reduction
         */
        Mat erodeElementRed = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(3, 3));//(13, 13));
        Mat dilateElementRed = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(3, 3));
    
        Mat erodeElementBlue = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(3, 3));
        Mat dilateElementBlue = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(3, 3));
    
        private List<MatOfPoint> findContoursCone(Mat input)
        {
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();
            //contoursList.clear(); // better to new it?
    
            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, yCbCrChanMat, Imgproc.COLOR_RGB2YCrCb);
    
            // Threshold the Cb channel to form a mask, then run some noise reduction
            //if (RobotAutonomousDrive.allianceConfig.Alliance == AllianceConfig.RED)
            if(objectFoundInfo.name == TargetObject.RED_CONE)
            {
                Core.extractChannel(yCbCrChanMat, yCbCrChanMat, 1);
                
                final int CB_CHAN_MASK_THRESHOLD_RED = 180; //cameraConfig.Red
                Imgproc.threshold(yCbCrChanMat, thresholdMat,
                        CB_CHAN_MASK_THRESHOLD_RED, 255, Imgproc.THRESH_BINARY);
    
                /*
                 * Apply some erosion and dilation for noise reduction
                 */
                Imgproc.erode(thresholdMat, morphedThreshold, erodeElementRed);
                Imgproc.erode(morphedThreshold, morphedThreshold, erodeElementRed);
    
                Imgproc.dilate(morphedThreshold, morphedThreshold, dilateElementRed);
                Imgproc.dilate(morphedThreshold, morphedThreshold, dilateElementRed);
            }
            else
            {
                Core.extractChannel(yCbCrChanMat, yCbCrChanMat, 2);
                
                final int CB_CHAN_MASK_THRESHOLD_BLUE = 160; // cameraConfig.Blue
                Imgproc.threshold(yCbCrChanMat, thresholdMat,
                        CB_CHAN_MASK_THRESHOLD_BLUE, 255, Imgproc.THRESH_BINARY);
    
                Imgproc.erode(thresholdMat, morphedThreshold, erodeElementBlue);
                Imgproc.erode(morphedThreshold, morphedThreshold, erodeElementBlue);
    
                Imgproc.dilate(morphedThreshold, morphedThreshold, dilateElementBlue);
                Imgproc.dilate(morphedThreshold, morphedThreshold, dilateElementBlue);
            }
    
            // Ok, now actually look for the contours! We only look for external contours.
            //Imgproc.findContours(morphedThreshold, contoursList, new Mat(),
            // Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(),
                    Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
    
            // remove noisy contours
            List<MatOfPoint> outputContours = new ArrayList<>();
            filterContours(contoursList, outputContours, 1000, 150,
                    30, 40);
    
            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, outputContours, -1, BLUE,
                    CONTOUR_LINE_THICKNESS, 8);
    
            return outputContours;
        }

    
        //private int minX, minY = Integer.MAX_VALUE;
        //private int maxX, maxY = -1 * Integer.MAX_VALUE;
    
        private void filterContours(List<MatOfPoint> contours, List<MatOfPoint> outputContours,
                                    double minContourArea, double minContourPerimeter,
                                    double minContourWidth, double minContourHeight)
        {
            MatOfPoint theLargestOne = null;
            double areaTheOne = 0;
            
            for (MatOfPoint contour : contours)
            {
                Rect rect = Imgproc.boundingRect(contour);
                int x = rect.x;
                int y = rect.y;
                int w = rect.width;
                int h = rect.height;
                double area = rect.area();
    
                if (w < minContourWidth)
                    continue;
                if (rect.area() < minContourArea)
                    continue;
                if ((2 * w + 2 * h) < minContourPerimeter)
                    continue;
                //if (h < minContourHeight)
                //    continue;
                
                if(area > areaTheOne)
                {
                    areaTheOne = area;
                    theLargestOne = contour;
                }
            }
            
            if(theLargestOne != null)
            {
                outputContours.add(theLargestOne);
            
                /*if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x + w > maxX) maxX = x + w;
                if (y + h > maxY) maxY = y + h;*/
            }
        }
    
        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
    
        private void analyzeContour(MatOfPoint contour, Mat input)
        {
            // Transform the contour to a different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
    
            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);
    
            objectFoundInfo.angle_avg = rotatedRectFitToContour.angle;
            objectFoundInfo.center_x_avg = (int)rotatedRectFitToContour.center.x;
            objectFoundInfo.center_y_avg = (int)rotatedRectFitToContour.center.y;
            objectFoundInfo.height_avg = (int)rotatedRectFitToContour.size.width;
            objectFoundInfo.width_avg = (int)rotatedRectFitToContour.size.height;
            
            String lb = String.format(new String("Deg:%d, A:%d, H:%d, W:%d"),
                    (int) Math.round(objectFoundInfo.angle_avg),
                    (int) (rotatedRectFitToContour.size.width*rotatedRectFitToContour.size.height),
                    objectFoundInfo.height_avg,
                    objectFoundInfo.width_avg);
            Imgproc.putText(
                    input, // The buffer we're drawing on
                    lb, // The text we're drawing
                    new Point( // The anchor point for the text
                            rotatedRectFitToContour.center.x - 50,  // x anchor point
                            rotatedRectFitToContour.center.y + 25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    GREEN, // Font color
                    1); // Font thickness
    
            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            //region1_Cb = yCbCrChanMat.submat(new Rect(region1_pointA, region1_pointB));
            //region2_Cb = yCbCrChanMat.submat(new Rect(region2_pointA, region2_pointB));
            //region3_Cb = yCbCrChanMat.submat(new Rect(region3_pointA, region3_pointB));
    
            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            //avg1 = (int) Core.mean(region1_Cb).val[0];
            //avg2 = (int) Core.mean(region2_Cb).val[0];
            //avg3 = (int) Core.mean(region3_Cb).val[0];
    
            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines*/
    
            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines*/
    
            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines*/
    
    
            
    
            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);
        }
        
        /*private Mat crop(Mat image, Point topLeftCorner, Point bottomRightCorner) {
            Rect cropRect = new Rect(topLeftCorner, bottomRightCorner);
            return new Mat(image, cropRect);
        }*/
    
        private static void drawRotatedRect(RotatedRect rect, Mat drawOn)
        {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */
        
            Point[] points = new Point[4];
            rect.points(points);
        
            for (int i = 0; i < 4; ++i)
            {
                String lb = String.format("%d(%4.0f,%4.0f)", i, points[i].x, points[i].y);
                Imgproc.line(drawOn, points[i], points[(i + 1) % 4], RED, 2);
                Imgproc.putText(drawOn, lb,
                        points[i], Imgproc.FONT_HERSHEY_PLAIN, // Font
                        0.5, // Font size
                        GREEN, // Font color
                        1);
            }
        }
    }
    
    public class AprilTagDetectionPipeline extends OpenCvPipeline
    {
        private long nativeApriltagPtr;
        private Mat grey = new Mat();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();
        
        private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
        private final Object detectionsUpdateSync = new Object();
        
        Mat cameraMatrix;
        
        Scalar blue = new Scalar(7,197,235,255);
        Scalar red = new Scalar(255,0,0,255);
        Scalar green = new Scalar(0,255,0,255);
        Scalar white = new Scalar(255,255,255,255);
        
        double fx;
        double fy;
        double cx;
        double cy;
        
        // UNITS ARE METERS
        double tagsize;
        double tagsizeX;
        double tagsizeY;
        
        private float decimation;
        private boolean needToSetDecimation;
        private final Object decimationSync = new Object();
        
        public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy)
        {
            this.tagsize = tagsize;
            this.tagsizeX = tagsize;
            this.tagsizeY = tagsize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            
            constructMatrix();
            
            // Allocate a native context object. See the corresponding deletion in the finalizer
            nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }
        
        @Override
        public void finalize()
        {
            // Might be null if createApriltagDetector() threw an exception
            if(nativeApriltagPtr != 0)
            {
                // Delete the native context we created in the constructor
                AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
                nativeApriltagPtr = 0;
            }
            else
            {
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
            }
        }
        
        @Override
        public Mat processFrame(Mat input)
        {
            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
            
            synchronized (decimationSync)
            {
                if(needToSetDecimation)
                {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                    needToSetDecimation = false;
                }
            }
            
            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);
            
            synchronized (detectionsUpdateSync)
            {
                detectionsUpdate = detections;
            }
            
            // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
            // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
            for(AprilTagDetection detection : detections)
            {
                Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
            }
            
            return input;
        }
        
        public void setDecimation(float decimation)
        {
            synchronized (decimationSync)
            {
                this.decimation = decimation;
                needToSetDecimation = true;
            }
        }
        
        public ArrayList<AprilTagDetection> getLatestDetections()
        {
            return detections;
        }
        
        public ArrayList<AprilTagDetection> getDetectionsUpdate()
        {
            synchronized (detectionsUpdateSync)
            {
                ArrayList<AprilTagDetection> ret = detectionsUpdate;
                detectionsUpdate = null;
                return ret;
            }
        }
        
        void constructMatrix()
        {
            //     Construct the camera matrix.
            //
            //      --         --
            //     | fx   0   cx |
            //     | 0    fy  cy |
            //     | 0    0   1  |
            //      --         --
            //
            
            cameraMatrix = new Mat(3,3, CvType.CV_32FC1);
            
            cameraMatrix.put(0,0, fx);
            cameraMatrix.put(0,1,0);
            cameraMatrix.put(0,2, cx);
            
            cameraMatrix.put(1,0,0);
            cameraMatrix.put(1,1,fy);
            cameraMatrix.put(1,2,cy);
            
            cameraMatrix.put(2, 0, 0);
            cameraMatrix.put(2,1,0);
            cameraMatrix.put(2,2,1);
        }
        
        /**
         * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
         *
         * @param buf the RGB buffer on which to draw the marker
         * @param length the length of each of the marker 'poles'
         * @param rvec the rotation vector of the detection
         * @param tvec the translation vector of the detection
         * @param cameraMatrix the camera matrix used when finding the detection
         */
        void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(0,0,0),
                    new Point3(length,0,0),
                    new Point3(0,length,0),
                    new Point3(0,0,-length)
            );
            
            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();
            
            // Draw the marker!
            Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);
            
            Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
        }
        
        void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
            //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])
            
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(-tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2,-tagHeight/2,-length),
                    new Point3(-tagWidth/2,-tagHeight/2,-length));
            
            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();
            
            // Pillars
            for(int i = 0; i < 4; i++)
            {
                Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
            }
            
            // Base lines
            //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
            //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
            //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
            //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);
            
            // Top lines
            Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
            Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
            Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
            Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
        }
        
        /**
         * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
         * original size of the tag.
         *
         * @param points the points which form the trapezoid
         * @param cameraMatrix the camera intrinsics matrix
         * @param tagsizeX the original width of the tag
         * @param tagsizeY the original height of the tag
         * @return the 6DOF pose of the camera relative to the tag
         */
        Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
        {
            // The actual 2d points of the tag detected in the image
            MatOfPoint2f points2d = new MatOfPoint2f(points);
            
            // The 3d points of the tag in an 'ideal projection'
            Point3[] arrayPoints3d = new Point3[4];
            arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
            arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
            MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);
            
            // Using this information, actually solve for pose
            Pose pose = new Pose();
            Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);
            
            return pose;
        }
        
        /*
         * A simple container to hold both rotation and translation
         * vectors, which together form a 6DOF pose.
         */
        class Pose
        {
            Mat rvec;
            Mat tvec;
            
            public Pose()
            {
                rvec = new Mat();
                tvec = new Mat();
            }
            
            public Pose(Mat rvec, Mat tvec)
            {
                this.rvec = rvec;
                this.tvec = tvec;
            }
        }
    }
}
