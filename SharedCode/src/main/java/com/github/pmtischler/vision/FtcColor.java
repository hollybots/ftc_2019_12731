package com.github.pmtischler.vision;

import com.github.pmtischler.base.Color;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Utilities for operating with FTC colors over images.
 * Example:
 *   Mat img = ...;
 *   FtcColor.simplifyImg(img);
 */
public class FtcColor {
    // Simplifies the image into red, blue, or black pixels.
    // This can be used to identify game elements for autonomous actions.
    // Mutates the provided image.
    public static void simplifyImg(Mat src) {
        Mat simple = new Mat(src.size(), CvType.CV_8UC3);
        Imgproc.cvtColor(src, simple, Imgproc.COLOR_BGR2HSV);

        byte[] pixel = new byte[3];
        for (int y = 0; y < src.height(); y++ ){
            for (int x = 0; x < src.width(); x++) {
                simple.get(y, x, pixel);
                simplifyHsv(pixel);
                simple.put(y, x, pixel);
            }
        }

        Imgproc.cvtColor(simple, src, Imgproc.COLOR_HSV2BGR);
    }

    // Simplifes the pixel into red, blue, or black.
    // Mutates the provided HSV pixel.
    public static void simplifyHsv(byte[] pixel) {
        int hue = (int)(pixel[0] & 0xFF);
        int saturation = (int)(pixel[1] & 0xFF);
        int value = (int)(pixel[2] & 0xFF);

        if (value < 70) {
            // Pixel too dark, make black.
            pixel[2] = 0;
        } else if (saturation < 70) {
            // Pixel too light, make black.
            pixel[2] = 0;
        } else if (hue < 15 || hue > 160) {
            // Pixel is red.
            pixel[0] = (byte)0;
            pixel[1] = (byte)255;
            pixel[2] = (byte)255;
        } else if (hue > 90 && hue < 135) {
            // Pixel is blue.
            pixel[0] = (byte)120;
            pixel[1] = (byte)255;
            pixel[2] = (byte)255;
        } else {
            // Not a color of interest, make black.
            pixel[2] = 0;
        }
    }
}
