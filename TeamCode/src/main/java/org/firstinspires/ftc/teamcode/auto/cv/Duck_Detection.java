//package org.firstinspires.ftc.teamcode.auto.cv;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//
//public class Duck_Detection extends OpenCvPipeline {
//    Telemetry telemetry;
//    Mat mat = new Mat();
//    private int lowHue = 40;
//    private int highHue = 50;
//    public enum Location {
//        LEFT,
//        RIGHT,
//        MID,
//        NOT_FOUND
//    }
//    private Location location;
//
//    static final Rect MID_ROI = new Rect(
//            new Point(700, 300),
//            new Point(1000, 500));
//    static final Rect LEFT_ROI = new Rect( //You don't matter
//            new Point(140,400),
//            new Point(420,600));
//
//
//    static double PERCENT_COLOR_THRESHOLD = 0.4;
//
//    public Duck_Detection (Telemetry t) { telemetry = t; }
//
//    @Override
//    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Scalar lowHSV = new Scalar(lowHue, 30, 42); //hue = 40
//        Scalar highHSV = new Scalar(highHue, 255, 255); //hue = 50
//
//        Core.inRange(mat, lowHSV, highHSV, mat);
//
//        Mat left = mat.submat(LEFT_ROI);
//        Mat mid = mat.submat(MID_ROI);
//
//        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
//        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;
//
//        left.release();
//        mid.release();
//
//        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
//        boolean stoneMid = midValue > PERCENT_COLOR_THRESHOLD;
//
//        if (stoneMid) {
//            location = Location.MID;
//            telemetry.addData("Duck Location", "mid");
//        }
//        else if (stoneLeft) {
//            location = Location.LEFT;
//            telemetry.addData("Duck Location", "left");
//        }
//        else {
//            location = Location.RIGHT;
//            telemetry.addData("Duck Location", "right");
//        }
//
//        telemetry.addData("Hue Low", getLowHue());
//        telemetry.addData("Hue High", getHighHue());
//        telemetry.addData("Block Seen", isSeen());
//        telemetry.update();
//
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
//
//        Scalar colorStone = new Scalar(255, 0, 0);
//        Scalar colorSkystone = new Scalar(0, 255, 0);
//        Scalar colorDuck = new Scalar(0,0,255);
//
//        Imgproc.rectangle(mat, MID_ROI, location == Location.MID? colorDuck:colorDuck);
//
//        return mat;
//    }
//
//    public Location getLocation() {
//        return location;
//    }
//
//    public void changeHue(int counter){
//        highHue += counter;
//        lowHue += counter;
//    }
//
//    public int getLowHue(){
//        return lowHue;
//    }
//    public int getHighHue(){
//        return highHue;
//    }
//
//    public boolean isSeen(){
//        if (getLocation() == Location.MID){
//            return true;
//        }
//        return false;
//    }
//}
//
//
//    //Scalar lowHSV = new Scalar(23, 30, 42);
//    //Scalar highHSV = new Scalar(32, 255, 255);
