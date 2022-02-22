package teamcode.Season_Setup;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;


public class Freight_Frenzy_Pipeline
{

    // Variable Declaration
    public static volatile Pipeline.ElementPosition positionOfTeamShippingElement = Pipeline.ElementPosition.LEFT;

    public static double analysisLeft = 0.0;
    public static double analysisCenter = 0.0;
    public static double analysisRight = 0.0;


    /**
     * Pipeline Class
     */
    public static class Pipeline extends OpenCvPipeline
    {
        // Camera View set-up
        boolean viewportPaused;
        public OpenCvCamera webcam;

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


        // An enum to define the Team Shipping Element's position on the Barcode
        public enum ElementPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        public ElementPosition elementPosition;

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
        Mat mat = new Mat(); // Storing the Colour Channel selected (BGR in this case)
        Mat bLeft = new Mat(); // Storing the extract "b" (blue) channel for Left
        Mat bCenter = new Mat(); // Storing the extract "b" (blue) channel for Center
        Mat bRight = new Mat(); // Storing the extract "b" (blue) channel for Right
        Mat holdLeft = new Mat(); // Storing left Mat
        Mat holdCenter = new Mat(); // Storing center Mat
        Mat holdRight = new Mat(); // Storing right Mat
        public double leftValue;
        public double centerValue;
        public double rightValue;


        /**
         * This function takes the RGB frame, converts to BGR,
         * and extracts the B channel to the "B" variable
         * @param input Camera input
         */
        void inputToBGR(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            // Extract the H channel from HSV
            Core.extractChannel(mat, bLeft, 0);
            Core.extractChannel(mat, bCenter, 0);
            Core.extractChannel(mat, bRight, 0);

            // Limiting to certain Threshold
            Imgproc.threshold(bLeft, holdLeft, 89, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(bLeft, holdCenter, 89, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(bLeft, holdRight, 89, 255, Imgproc.THRESH_BINARY_INV);
        }


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


        /**
         * @param input Initializing the frame
         */
        @Override
        public void init(Mat input) {
            inputToBGR(input);
            leftBarcode = bLeft.submat(LEFT_BARCODE);
            centerBarcode = bCenter.submat(CENTER_BARCODE);
            rightBarcode = bRight.submat(RIGHT_BARCODE);
        }


        @Override
        public Mat processFrame(Mat input) {

            // Initializing the frame
            inputToBGR(input);

            // Drawing the Rectangles
            drawRectangle(input, LEFT_BARCODE, WHITE,2); // Left Barcode Rectangle
            drawRectangle(input, CENTER_BARCODE, WHITE,2); // Center Barcode Rectangle
            drawRectangle(input, RIGHT_BARCODE, WHITE,2); // Right Barcode Rectangle


            // Setting variable values
            leftValue = (int) Core.mean(leftBarcode).val[0];
            centerValue = (int) Core.mean(centerBarcode).val[0];
            rightValue = (int) Core.mean(rightBarcode).val[0];


            // Record out analysis
            if (leftValue > centerValue && leftValue > rightValue) {
                elementPosition = ElementPosition.LEFT;
                drawRectangle(input, LEFT_BARCODE, GREEN,3); // Left Barcode Rectangle

            } else if (centerValue > leftValue && centerValue > rightValue) {
                elementPosition = ElementPosition.CENTER;
                drawRectangle(input, CENTER_BARCODE, GREEN,3); // Left Barcode Rectangle

            } else if (rightValue > leftValue && rightValue > centerValue) {
                elementPosition = ElementPosition.RIGHT;
                drawRectangle(input, RIGHT_BARCODE, GREEN,3); // Left Barcode Rectangle

            } else {
                elementPosition = ElementPosition.RIGHT;
                drawRectangle(input, LEFT_BARCODE, GREEN,3); // Left Barcode Rectangle

            }


            // Setting Analysis Value for Telemetry
            analysisLeft = leftValue;
            analysisCenter = centerValue;
            analysisRight = rightValue;

            // Setting Position of Element for Telemetry
            positionOfTeamShippingElement = elementPosition;

            // Return Input
            return input;
        }
    }
}
