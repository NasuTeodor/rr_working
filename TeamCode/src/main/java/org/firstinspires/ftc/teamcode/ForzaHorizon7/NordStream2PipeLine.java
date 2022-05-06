package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class NordStream2PipeLine extends OpenCvPipeline {

    Rect LeftROI = new Rect(
            new Point(0,60),
            new Point(100 ,160)
    );
    Rect CenterROI = new Rect(
            new Point(100,60),
            new Point(210,160)
    );
    Rect RightROI = new Rect(
            new Point(210,60),
            new Point(320,160)
    );

    public static Scalar low = new Scalar(20,50,40);
    public static Scalar high = new Scalar(100,255,255);

    double leftValue,centerValue,rightValue;
    int acolo=0;

    @Override
    public Mat processFrame(Mat frame){

        Mat mask = new Mat();

        Imgproc.cvtColor(frame, mask, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mask, low, high, mask);
//
//        Imgproc.rectangle(frame, CenterROI, new Scalar(100,100,100), 2);
//        Imgproc.rectangle(frame, LeftROI, new Scalar(100,100,100), 2);
//        Imgproc.rectangle(frame, RightROI, new Scalar(100,100,100), 2);

        Mat leftMask = mask.submat(LeftROI);
        Mat centerMask = mask.submat(CenterROI);
        Mat rightMask = mask.submat(RightROI);

        leftValue = Core.sumElems(leftMask).val[0] / 255;
        centerValue = Core.sumElems(centerMask).val[0] / 255;
        rightValue = Core.sumElems(rightMask).val[0] /255;

        if(leftValue > centerValue && leftValue > rightValue)
            acolo = 1;
        else if(centerValue > leftValue && centerValue > rightValue)
            acolo = 2;
        else acolo = 3;

        leftMask.release();
        centerMask.release();
        rightMask.release();

        return mask;
    }

    public int gasesteMarker(){ return acolo; }

}
