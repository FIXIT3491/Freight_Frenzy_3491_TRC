package teamcode.Season_Setup;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;


    /**
     * Pipeline Class
     */
public class Freight_Frenzy_Pipeline extends OpenCvPipeline
{
    // Variable Declaration
    public int outputElementPosition;

    public static class ElementInfo
    {
        public int elementPosition = 0; // 1 = Left (level one,) 2 = Center (level two,) 3 = Right (level three)

        public double leftValue;
        public double centerValue;
        public double rightValue;

        public ElementInfo (ElementInfo other)
        {
            if (other != null)
            {
                this.elementPosition = other.elementPosition;
                this.leftValue = other.leftValue;
                this.centerValue = other.centerValue;
                this.rightValue = other.rightValue;
            }
        }
    }

    // Camera View set-up
    boolean viewportPaused;
    public OpenCvCamera webcam;

    private final ElementInfo elementInfo = new ElementInfo(null);


    // Viewport setup
    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }

    // Color constant
    final Scalar WHITE = new Scalar(255, 255, 255);
    final Scalar GREEN = new Scalar(0, 255, 0);

    // The core values which define the location and size of the sample regions
    static final int REGION_HEIGHT = 100;        static final int REGION_WIDTH = 70;

    static final int CENTER_BARCODE_TOP_LEFT_X_ANCHOR_POINT = 320/2 - REGION_WIDTH/2;

    static final Point LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT =   new Point(0, 100);
    static final Point CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT = new Point(CENTER_BARCODE_TOP_LEFT_X_ANCHOR_POINT, 100);
    static final Point RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT =  new Point(320-REGION_WIDTH, 100);


    ////* Creating the sample regions *////
    /* pointA = The Top left point of the region */
    /* pointB = The Bottom right point of the region */

    // Left Barcode
    static final Rect LEFT_BARCODE = new Rect(
            new Point(LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
                    LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.y),
            new Point(LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT));

    // Center Barcode
    static final Rect CENTER_BARCODE = new Rect(
            new Point(CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
                    CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.y),
            new Point(CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT));

    // Right Barcode
    static final Rect RIGHT_BARCODE = new Rect(
            new Point(RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
                    RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.y),
            new Point(RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT));

    // Variable Declaration
    Mat leftBarcode;
    Mat centerBarcode;
    Mat rightBarcode;


    /**
     * This function draw a Rectangle that is than displayed on the screen
     * @param input Camera input
     */
    void drawRectangle (Mat input, Rect rect, Scalar colour, int thickness) {
        // Left Barcode Rectangle
        Imgproc.rectangle(
                input, // Buffer to draw on
                rect, // Where the Rectangle is drawn
                colour, // The colour of the rectangle is drawn in
                thickness); // Thickness of the rectangle lines
    }

    //Outputs
    private final Mat resizeImageOutput = new Mat();
    private final Mat cvCvtcolorOutput = new Mat();
    private final Mat cvExtractchannelOutput = new Mat();
    private final Mat cvThresholdOutput = new Mat();


    /**
     * Scales and image to an exact size.
     * @param input The image on which to perform the Resize.
     * @param width The width of the output in pixels.
     * @param height The height of the output in pixels.
     * @param interpolation The type of interpolation.
     * @param output The image in which to store the output.
     */
    private void resizeImage(Mat input, double width, double height,
                             int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    /**
     * Converts an image from one color space to another.
     * @param src Image to convert.
     * @param code conversion code.
     * @param dst converted Image.
     */
    private void cvCvtcolor(Mat src, int code, Mat dst) {
        Imgproc.cvtColor(src, dst, code);
    }

    /**
     * Extracts given channel from an image.
     * @param src the image to extract.
     * @param channel zero indexed channel number to extract.
     * @param dst output image.
     */
    private void cvExtractchannel(Mat src, double channel, Mat dst) {
        Core.extractChannel(src, dst, (int)channel);
    }

    /**
     * Apply a fixed-level threshold to each array element in an image.
     * @param src Image to threshold.
     * @param threshold threshold value.
     * @param maxVal Maximum value for THRES_BINARY and THRES_BINARY_INV
     * @param type Type of threshold to appy.
     * @param dst output Image.
     */
    private void cvThreshold(Mat src, double threshold, double maxVal, int type,
                             Mat dst) {
        Imgproc.threshold(src, dst, threshold, maxVal, type);
    }


    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public void process(Mat source0) {
        // Step Resize_Image0:
        double resizeImageWidth = 320.0;
        double resizeImageHeight = 240.0;
        int resizeImageInterpolation = Imgproc.INTER_CUBIC;
        resizeImage(source0, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

        // Step CV_cvtColor0:
        int cvCvtcolorCode = Imgproc.COLOR_RGB2HSV;
        cvCvtcolor(resizeImageOutput, cvCvtcolorCode, cvCvtcolorOutput);

        // Step CV_extractChannel0:
        double cvExtractchannelChannel = 0.0;
        cvExtractchannel(cvCvtcolorOutput, cvExtractchannelChannel, cvExtractchannelOutput);

        // Step CV_Threshold0:
        double cvThresholdThresh = 50.0;
        double cvThresholdMaxval = 255.0;
        int cvThresholdType = Imgproc.THRESH_BINARY;
        cvThreshold(cvExtractchannelOutput, cvThresholdThresh, cvThresholdMaxval, cvThresholdType, cvThresholdOutput);
    }

    @Override
    public synchronized Mat processFrame(Mat input) {

        process(input);

        leftBarcode = cvThresholdOutput.submat(LEFT_BARCODE);
        centerBarcode = cvThresholdOutput.submat(CENTER_BARCODE);
        rightBarcode = cvCvtcolorOutput.submat(RIGHT_BARCODE);

        drawRectangle(input, LEFT_BARCODE, WHITE,2); // Left Barcode Rectangle
        drawRectangle(input, CENTER_BARCODE, WHITE,2); // Left Barcode Rectangle
        drawRectangle(input, RIGHT_BARCODE, WHITE,2); // Left Barcode Rectangle


        synchronized (elementInfo) {
            // Setting variable values
            elementInfo.leftValue = Core.mean(leftBarcode).val[0];
            elementInfo.centerValue = Core.mean(centerBarcode).val[0];
            elementInfo.rightValue = Core.mean(rightBarcode).val[0];

            double elementThreshold = 0.0; // Needs to be tuned.
            int elementPositionLocal = 0;

            if (elementInfo.leftValue > elementThreshold)
            {
                elementPositionLocal = 1;
            }
            else if (elementInfo.centerValue > elementThreshold)
            {
                elementPositionLocal = 2;
            }
            else if (elementInfo.rightValue > elementThreshold)
            {
                elementPositionLocal = 3;
            }


            if (elementPositionLocal == 1)
            {
                drawRectangle(input, LEFT_BARCODE, GREEN,3); // Left Barcode Rectangle
            }
            else if (elementPositionLocal == 2)
            {
                drawRectangle(input, CENTER_BARCODE, GREEN,3); // Center Barcode Rectangle
            }
            else if (elementPositionLocal == 3)
            {
                drawRectangle(input, RIGHT_BARCODE, GREEN,3); // Right Barcode Rectangle
            }


            setOutputElementPosition(elementPositionLocal);


            if (elementPositionLocal != 0)
            {
                elementInfo.elementPosition = elementPositionLocal;
            }
        }


        // Return Input
        return input;
    }

    /**
     * Returns the position of the element. 1 = Left (level one,) 2 = Center (level two,) 3 = Right (level three)
     * @return elementPosition
     */
    public ElementInfo getElementInfo ()
    {
        synchronized (elementInfo)
        {
            return new ElementInfo(elementInfo);
        }
    }

    /**
     * Turns off and disables camera view.
     */
    public void disableWebcam () {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public void setOutputElementPosition (int position)
    {
        outputElementPosition = position;
    }

    public int getOutputElementPosition ()
    {
        return outputElementPosition;
    }
}

