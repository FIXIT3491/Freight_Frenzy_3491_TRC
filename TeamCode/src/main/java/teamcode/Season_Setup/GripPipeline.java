package teamcode.Season_Setup;

import org.opencv.core.*;
import org.opencv.imgproc.*;

/**
 * GripPipeline class.
 *
 * <p>An OpenCV pipeline generated by GRIP.
 *
 * @author GRIP
 */
@SuppressWarnings("unused")
public class GripPipeline {

    //Outputs
    private final Mat resizeImageOutput = new Mat();
    private final Mat cvCvtcolorOutput = new Mat();
    private final Mat cvExtractchannelOutput = new Mat();
    private final Mat cvThresholdOutput = new Mat();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
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
        double cvThresholdThresh = 89.0;
        double cvThresholdMaxval = 255.0;
        int cvThresholdType = Imgproc.THRESH_BINARY;
        cvThreshold(cvExtractchannelOutput, cvThresholdThresh, cvThresholdMaxval, cvThresholdType, cvThresholdOutput);
    }

    /**
     * This method is a generated getter for the output of a Resize_Image.
     * @return Mat output from Resize_Image.
     */
    public Mat resizeImageOutput() {
        return resizeImageOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_cvtColor.
     * @return Mat output from CV_cvtColor.
     */
    public Mat cvCvtcolorOutput() {
        return cvCvtcolorOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_extractChannel.
     * @return Mat output from CV_extractChannel.
     */
    public Mat cvExtractchannelOutput() {
        return cvExtractchannelOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_Threshold.
     * @return Mat output from CV_Threshold.
     */
    public Mat cvThresholdOutput() {
        return cvThresholdOutput;
    }


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
}
