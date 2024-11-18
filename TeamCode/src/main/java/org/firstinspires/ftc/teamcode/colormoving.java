package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

@TeleOp(name = "colormoving (Blocks to Java)")
public class colormoving extends LinearOpMode {

  private SparkFunOTOS sensor_otos;
  private DcMotor viperSlide;
  private DcMotor leftFront;
  private DcMotor rightFront;
  private DcMotor leftRear;
  private DcMotor rightRear;

  int widthxforlocal;
  int heightyforlocal;
  boolean tf_;
  double speedformot;
  RotatedRect myBoxFit;

  /**
   * Configures the SparkFun OTOS.
   */
  private void configureOTOS() {
    SparkFunOTOS.Pose2D offset;
    SparkFunOTOS.Pose2D currentPosition;

    telemetry.addLine("Configuring OTOS...");
    telemetry.update();
    sensor_otos.setLinearUnit(DistanceUnit.INCH);
    sensor_otos.setAngularUnit(AngleUnit.DEGREES);
    offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    sensor_otos.setOffset(offset);
    sensor_otos.setLinearScalar(1);
    sensor_otos.setAngularScalar(1);
    sensor_otos.calibrateImu();
    sensor_otos.resetTracking();
    currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
    sensor_otos.setPosition(currentPosition);
    telemetry.update();
  }

  /**
   * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions.
   *
   * Unlike a "color sensor" which determines the color of an object in the
   * field of view, this "color locator" will search the Region Of Interest
   * (ROI) in a camera image, and find any "blobs" of color that match the
   * requested color range. These blobs can be further filtered and sorted
   * to find the one most likely to be the item the user is looking for.
   *
   * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
   * The ColorBlobLocatorProcessor process is created first,
   * and then the VisionPortal is built to use this process.
   * The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
   * The matching pixels are then collected into contiguous "blobs" of
   * pixels. The outer boundaries of these blobs are called its "contour".
   * For each blob, the process then creates the smallest possible
   * rectangle "boxFit" that will fully encase the contour.
   * The user can then call getBlobs() to retrieve the list of Blobs,
   * where each Blob contains the contour and the boxFit data.
   * Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are
   *   listed first.
   *
   * To aid the user, a colored boxFit rectangle is drawn on the camera preview to
   * show the location of each Blob. The original Blob contour can also be added to the
   * preview. This is helpful when configuring the ColorBlobLocatorProcessor parameters.
   */
  @Override
  public void runOpMode() {
    ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
    AprilTagProcessor.Builder aprilTagBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    AprilTagProcessor aprilTag;
    VisionPortal myVisionPortal;
    int DESIRED_DISTANCE;
    double SPEED_GAIN;
    double TURN_GAIN;
    double MAX_AUTO_SPEED;
    double MAX_AUTO_TURN;
    boolean USE_WEBCAM;
    int DESIRED_TAG_ID;
    boolean targetFound;
    double drive;
    double turn;
    AprilTagDetection desiredTag;
    List<AprilTagDetection> currentDetections;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    AprilTagDetection detection;
    SparkFunOTOS.Pose2D pos;
    double rangeError;
    double headingError;
    ColorBlobLocatorProcessor.Blob myBlob;

    sensor_otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
    viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    leftRear = hardwareMap.get(DcMotor.class, "leftRear");
    rightRear = hardwareMap.get(DcMotor.class, "rightRear");

    widthxforlocal = 960 / 2;
    heightyforlocal = 540 / 2;
    // Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
    myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
    // - Specify the color range you are looking for.
    myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.BLUE);
    // - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
    //     This can be the entire frame, or a sub-region defined using:
    //     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
    //     Use one form of the ImageRegion class to define the ROI.
    // 50% width/height square centered on screen
    myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1));
    // - Define which contours are included.
    //     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
    //     note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
    myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
    // - Turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
    myColorBlobLocatorProcessorBuilder.setDrawContours(false);
    // - Include any pre-processing of the image or mask before looking for Blobs.
    //     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
    //     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
    //     Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
    //     The higher the number of pixels, the more blurred the image becomes.
    //     Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
    //     Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
    myColorBlobLocatorProcessorBuilder.setBlurSize(5);
    //     Erosion removes floating pixels and thin lines so that only substantive objects remain.
    //     Erosion can grow holes inside regions, and also shrink objects.
    //     "pixels" in the range of 2-4 are suitable for low res images.
    //     Dilation makes objects more visible by filling in small holes, making lines appear thicker,
    //     and making filled shapes appear larger. Dilation is useful for joining broken parts of an
    //     object, such as when removing noise from an image.
    //     "pixels" in the range of 2-4 are suitable for low res images.
    myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
    // Build a vision portal to run the Color Locator process.
    aprilTagBuilder = new AprilTagProcessor.Builder();
    aprilTag = aprilTagBuilder.build();
    aprilTag.setDecimation(2);
    myVisionPortalBuilder = new VisionPortal.Builder();
    //  - Add the ColorBlobLocatorProcessor created above.
    myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
    myVisionPortalBuilder.addProcessor(aprilTag);
    //  - Set the desired video resolution.
    //      Since a high resolution will not improve this process, choose a lower resolution that is
    //      supported by your camera. This will improve overall performance and reduce latency.
    myVisionPortalBuilder.setCameraResolution(new Size(widthxforlocal * 2, heightyforlocal * 2));
    // Set the stream format.
    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
    //  - Choose your video source. This may be for a webcam or for a Phone Camera.
    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    myVisionPortal = myVisionPortalBuilder.build();
    // DESIRED_DISTANCE is how close the camera should get to the target in inches.
    DESIRED_DISTANCE = 12;
    // Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error. (0.50 / 25.0)
    SPEED_GAIN = 0.02;
    // Turn Control "Gain". e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    TURN_GAIN = 0.01;
    // Clip the approach speed to this max value (adjust for your robot)
    MAX_AUTO_SPEED = 0.5;
    // Clip the turn speed to this max value (adjust for your robot)
    MAX_AUTO_TURN = 0.25;
    // Set true to use a webcam, or false for a phone camera.
    USE_WEBCAM = true;
    // Choose the tag you want to approach or set to -1 for ANY tag.
    DESIRED_TAG_ID = -1;
    // Whether an AprilTag target is detected
    targetFound = false;
    // forward power/speed (-1 to +1) positive is forward
    drive = 0;
    // turning power/speed (-1 to +1) positive is CounterClockwise
    turn = 0;
    if (USE_WEBCAM) {
      // Use low exposure time to reduce motion blur.
      setManualExposure(6, 250);
    }
    // Speed up telemetry updates, Just use for debugging.
    telemetry.setMsTransmissionInterval(50);
    telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    configureOTOS();
    viperSlide.setTargetPosition(100);
    viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
    while (opModeIsActive() || opModeInInit()) {
      targetFound = false;
      desiredTag = null;
      // Step through the list of detected tags and look for a matching tag.
      currentDetections = aprilTag.getDetections();
      // Read the current list of blobs.
      myBlobs = myColorBlobLocatorProcessor.getBlobs();
      // The list of Blobs can be filtered to remove unwanted Blobs.
      //   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
      //             conditions will remain in the current list of "blobs".  Multiple filters may be used.
      //
      // Use any of the following filters.
      //
      // A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
      // Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
      ColorBlobLocatorProcessor.Util.filterByArea(3000, 500000, myBlobs);
      // A blob's density is an indication of how "full" the contour is.
      // If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
      // The density is the ratio of Contour-area to Convex Hull-area.
      // A blob's Aspect ratio is the ratio of boxFit long side to short side.
      // A perfect Square has an aspect ratio of 1.  All others are > 1
      // The list of Blobs can be sorted using the same Blob attributes as listed above.
      // No more than one sort call should be made.  Sorting can use ascending or descending order.
      for (AprilTagDetection detection_item : currentDetections) {
        detection = detection_item;
        // Look to see if we have size info on this tag.
        if (detection.metadata != null) {
          // Check to see if we want to track towards this tag.
          if (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID) {
            // Yes, we want to use this tag.
            targetFound = true;
            desiredTag = detection;
            // Don't look any further.
            break;
          } else {
            // This tag is in the library, but we do not want to track it right now.
            telemetry.addData("Skipping", "Tag ID " + detection.id + " is not desired");
          }
        } else {
          // This tag is NOT in the library, so we don't have enough information to track to it.
          telemetry.addData("Unknown", "Tag ID " + detection.id + " is not in TagLibrary");
        }
      }
      telemetry.addLine("");
      if (targetFound) {
        telemetry.addLine("");
        telemetry.addData("Found", "ID " + desiredTag.id + " (" + desiredTag.metadata.name + ")");
        telemetry.addData("Range", JavaUtil.formatNumber(desiredTag.ftcPose.range, 5, 1) + " inches");
        telemetry.addData("Bearing", JavaUtil.formatNumber(desiredTag.ftcPose.bearing, 3, 0) + " degrees");
      } else {
        telemetry.addData(">", "nothin in da view");
        telemetry.addLine("");
      }
      pos = sensor_otos.getPosition();
      // Reset the tracking if the user requests it.
      if (gamepad1.y) {
        sensor_otos.resetTracking();
      }
      // Re-calibrate the IMU if the user requests it.
      if (gamepad1.x) {
        sensor_otos.calibrateImu();
      }
      // Inform user of available controls
      // Get the current vision frame rate.
      telemetry.addLine("" + Math.round(myVisionPortal.getFps()));
      telemetry.addLine("");
      telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
      telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
      telemetry.addLine("");
      telemetry.addData("X coordinate", JavaUtil.formatNumber(pos.x, 2));
      telemetry.addData("Y coordinate", JavaUtil.formatNumber(pos.y, 2));
      telemetry.addData("Heading angle", JavaUtil.formatNumber(pos.h, 2));
      telemetry.addLine("");
      if (gamepad1.right_bumper && targetFound) {
        // Determine heading and range error so we can use them to control the robot automatically.
        rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
        headingError = desiredTag.ftcPose.bearing;
        // Use the speed and turn "gains" to calculate how we want the robot to move. Clip it to the maximum.
        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        telemetry.addData("Auto", "Drive " + JavaUtil.formatNumber(drive, 5, 2) + ", Turn " + JavaUtil.formatNumber(turn, 5, 2));
      } else {
        // Drive using manual POV Joystick mode. Slow things down to make the robot more controllable.
        // Reduce drive rate to 50%.
        drive = -gamepad1.left_stick_y / 2;
        // Reduce turn rate to 25%.
        turn = -gamepad1.right_stick_x / 4;
        telemetry.addData("Manual", "Drive " + JavaUtil.formatNumber(drive, 5, 2) + ", Turn " + JavaUtil.formatNumber(turn, 5, 2));
      }
      viperSlide.setPower(gamepad2.left_stick_y);
      moveRobot((int) drive, (int) turn);
      telemetry.addLine("");
      telemetry.addLine(" Area Density Aspect  Center");
      if (!myBlobs) {
        moveRobot(100, 0);
      }
      // Display the size (area) and center location for each Blob.
      for (ColorBlobLocatorProcessor.Blob myBlob_item : myBlobs) {
        myBlob = myBlob_item;
        // Get a "best-fit" bounding box (called "boxFit", of type RotatedRect) for this blob.
        myBoxFit = myBlob.getBoxFit();
        // Get the aspect ratio of this blob, i.e. the ratio of the
        // longer side of the "boxFit" bounding box to the shorter side.
        telemetry.addLine(JavaUtil.formatNumber(myBlob.getContourArea(), 5, 0) + "  " + JavaUtil.formatNumber(myBlob.getDensity(), 4, 2) + "   " + JavaUtil.formatNumber(myBlob.getAspectRatio(), 5, 2) + "  (" + JavaUtil.formatNumber(myBoxFit.center.x, 3, 0) + "," + JavaUtil.formatNumber(myBoxFit.center.y, 3, 0) + ")");
      }
      if (gamepad1.a) {
        if (!(Math.min(Math.max(Math.abs(myBoxFit.center.x), widthxforlocal - 200), widthxforlocal + 200) == widthxforlocal - 200 || Math.min(Math.max(Math.abs(myBoxFit.center.x), widthxforlocal - 200), widthxforlocal + 200) == widthxforlocal + 200) && Math.abs(myBoxFit.center.y) > heightyforlocal + 150) {
          tf_ = true;
        } else {
          tf_ = false;
        }
        speedformot = 0.2;
        rotatedquads();
        quads();
      }
      if (gamepad1.left_bumper) {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
      }
      telemetry.update();
      sleep(50);
    }
  }

  /**
   * Manually set the camera gain and exposure.
   * This can only be called after calling initAprilTag, and only works for webcams.
   */
  private void setManualExposure(int exposureMs, int gain) {
    VisionPortal visionPortal;
    ExposureControl exposureControl;
    GainControl gainControl;

    // Wait for the camera to be open, then use the controls.
    if (visionPortal == null) {
      return;
    }
    // Make sure camera is streaming before we try to set the exposure controls.
    if (!visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
      telemetry.addData("Camera", "Waiting");
      telemetry.update();
      while (!isStopRequested() && !visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
        sleep(20);
      }
      telemetry.addData("Camera", "Ready");
      telemetry.update();
    }
    // Set camera controls unless we are stopping.
    if (!isStopRequested()) {
      exposureControl = visionPortal.getCameraControl(ExposureControl.class);
      if (!exposureControl.getMode().equals(ExposureControl.Mode.Manual)) {
        exposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(50);
      }
      exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
      sleep(20);
      gainControl = visionPortal.getCameraControl(GainControl.class);
      gainControl.setGain(gain);
      sleep(20);
      telemetry.addData("Camera", "Ready");
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void quads() {
    if (myBoxFit.center.x > widthxforlocal && myBoxFit.center.y > heightyforlocal) {
      print("right and down");
    } else if (myBoxFit.center.x < widthxforlocal && myBoxFit.center.y > heightyforlocal) {
      print("left and down");
    } else if (myBoxFit.center.x > widthxforlocal && myBoxFit.center.y < heightyforlocal) {
      print("right and up");
    } else if (myBoxFit.center.x < widthxforlocal && myBoxFit.center.y < heightyforlocal) {
      print("left and up");
    } else {
      print("center");
    }
  }

  /**
   * Describe this function...
   */
  private void print(String tet) {
    telemetry.addLine(tet);
  }

  /**
   * Describe this function...
   */
  private void rotatedquads() {
    if (myBoxFit.center.x < widthxforlocal - 300 / (1920 / (widthxforlocal * 2))) {
      print("left");
      leftFront.setPower(-speedformot);
      rightFront.setPower(-speedformot);
      leftRear.setPower(-speedformot);
      rightRear.setPower(-speedformot);
    } else if (myBoxFit.center.x > widthxforlocal + 300 / (1920 / (widthxforlocal * 2))) {
      print("right");
      leftFront.setPower(speedformot);
      rightFront.setPower(speedformot);
      leftRear.setPower(speedformot);
      rightRear.setPower(speedformot);
    } else if (myBoxFit.center.y < heightyforlocal + 240 / (1080 / (heightyforlocal * 2))) {
      print("up");
      leftFront.setPower(speedformot);
      rightFront.setPower(-speedformot);
      leftRear.setPower(speedformot);
      rightRear.setPower(-speedformot);
    } else if (myBoxFit.center.y > heightyforlocal + 475 / (1080 / (heightyforlocal * 2))) {
      print("down");
      leftFront.setPower(-speedformot);
      rightFront.setPower(speedformot);
      leftRear.setPower(-speedformot);
      rightRear.setPower(speedformot);
    } else {
      print("center");
      leftFront.setPower(0);
      rightFront.setPower(0);
      leftRear.setPower(0);
      rightRear.setPower(0);
    }
    if (tf_) {
      leftFront.setPower(0);
      rightFront.setPower(0);
      leftRear.setPower(0);
      rightRear.setPower(0);
    }
  }

  /**
   * Move robot according to desired axes motions.
   * Positive x is forward.
   * Positive yaw is counter-clockwise.
   */
  private void moveRobot(int x, int yaw) {
    int leftPower;
    int rightPower;
    double max;

    // Calculate left and right wheel powers.
    leftPower = x - yaw;
    rightPower = x + yaw;
    // Normalize wheel powers to be less than 1.0.
    max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
    if (max > 1) {
      leftPower = (int) (leftPower / max);
      rightPower = (int) (rightPower / max);
    }
    // Send powers to the wheels.
    leftFront.setPower(leftPower);
    rightFront.setPower(-rightPower);
    leftRear.setPower(leftPower);
    rightRear.setPower(-rightPower);
  }
}
