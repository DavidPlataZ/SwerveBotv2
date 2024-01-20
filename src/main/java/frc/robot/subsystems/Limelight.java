package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

//NOTICE Limelight is connected to port 22 on the power distribution

/*
 * Singleton class to handle the Limelight subsystem base
 * <ul>
 *  <li><strong>Use Case</strong>: Manages data sent from the limelight hardware</li>
 * </ul>
 */
public class Limelight extends SubsystemBase {
    /**
     * Table containing information regarding the Network Table
     * <ul>
     *  <li><strong>Network Table</strong>: Distributed data-management system that allows multiple devices to share data over a network</li>
     *  <li><strong>Device Examples</strong>: Robot Controller, Driver Station, and other applications</li>
     * </ul>
     */
    private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();

    /**
     * Network Table specific to the Limelight
     * <ul>
     *  <li><strong>Limelight Network Table</strong>: Contains information regarding the robot and its target(s)</li>
     * </ul>
     */
    private static NetworkTable table = tableInstance.getTable("limelight");

    private static NetworkTableEntry timingTestEntry = table.getEntry("TIMING_TEST_ENTRY");
    private static boolean timingTestEntryValue = false;
    public static final long MAX_UPDATE_TIME = 100_000; // microseconds; 0.1 seconds
    
    public static final boolean POST_TO_SMART_DASHBOARD = true;
    public boolean m_LimelightHasValidTarget = false;
    public double m_LimelightSteerCommand = 0.0;
    public double m_LimelightTargetArea = 0;

    //force test variables
    boolean timingTestEntryValue2 = !timingTestEntryValue;
    // timingTestEntry.forceSetBoolean(timingTestEntryValue);

    static long currentTime = timingTestEntry.getLastChange();
    //temp
    // The pipeline’s latency contribution (ms)
    public static final double IMAGE_CAPTURE_LATENCY = 11;
    private static NetworkTableEntry latencyEntry = table.getEntry("tl");
    //temp
    static long lastUpdate = latencyEntry.getLastChange();
    //connected variables
    static long timeDifference = currentTime - lastUpdate;
    static boolean connected = timeDifference < MAX_UPDATE_TIME;

    /**
     * Determines connectivity state of the Limelight to the Smart Dashboard
     * @return Whether the Limelight is connected to the Smart Dashboard
     */
    public static boolean isConnected() {
        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight Connected", connected);
                //SmartDashboard.putNumberConnection("Limelight Time Difference", timeDifference);
            SmartDashboard.putNumber("Limelight Time Difference", timeDifference);
        }
        return connected;
    }
    public void setLedOn(boolean isConnected) {
        if (isConnected)
        LEDModeEntry.setNumber(LimelightConstants.LED_ON);
    }

    /*
     * DISPLAY VARIABLES
     * not final type if someone would like an edit of aspect ratio etc
     *
     */
    public static double DEFAULT_TARGET_HEIGHT_THRESHOLD = 6;
    public static double DEFAULT_MIN_ASPECT_RATIO = 1.0;
    public static double DEFAULT_MAX_ASPECT_RATIO = 3.0;
    public static double DEFAULT_ANGLE_THRESHOLD = 25;

    /**
     * Check's if an April Tag's height, minimum and maximum ratios of height to width, and angle threshold is valid based on certain criteria
     * <ul>
     *  <li><strong>Criteria:</strong>
     *      <ul>
     *          <li><strong>Target Height Threshold</strong></li>
     *          <li><strong>Minimum Ratio</strong></li>
     *          <li><strong>Maximum Ratio</strong></li>
     *          <li><strong>Angle Threshold</strong></li>
     *      </ul>
     *  </li>
     * </ul>
     * @param targetHeightThreshold The height of an  AprilTag relative to the camera
     * @param minRatio The maximum ratio of an April Tag's height to width relative to the camera
     * @param maxRatio The minimum ratio of an April Tag's height to width relative to the camera
     * @param angleThreshold The angle of an April Tag relative to the camera
     * @return Whether the Limelight has a valid Target based on given requirements
     */

    public static boolean hasValidTarget(double targetHeightThreshold, double minRatio, double maxRatio,double angleThreshold) {
        return hasAnyTarget() 
        && hasValidHeight(targetHeightThreshold) 
        && hasValidBlueAspectRatio(minRatio, maxRatio)
        && hasValidBlueOrientation(angleThreshold);
    }

    /**
     * Determines whether the Limelight has a valid target
     * @return whether the Limelight has a valid target by validating the specific arguments of the target by utilizing the {@link #hasValidTarget(double, double, double, double)} method.
     */
    public static boolean hasValidTarget() {
        return SmartDashboard.getBoolean("CV_FILTER_OVERRIDE", false) && hasValidTarget(
            SmartDashboard.getNumber("HEIGHT_THRESHOLD", DEFAULT_TARGET_HEIGHT_THRESHOLD),
            SmartDashboard.getNumber("MIN_ASPECT_RATIO", DEFAULT_MIN_ASPECT_RATIO),
            SmartDashboard.getNumber("MAX_ASPECT_RATIO", DEFAULT_MAX_ASPECT_RATIO),
            SmartDashboard.getNumber("SKEW_THRESHOLD", DEFAULT_ANGLE_THRESHOLD)
            );
    }

    private static NetworkTableEntry validTargetEntry = table.getEntry("tv");

    /**
     * Determines whether the Limelight has a target or not.
     * <br></br>
     * <strong>NOTE: ONLY checks if the Limelight has a target (NOT if the target is valid)</strong>
     * @return Whether the Limelight has a target
     */
    public static boolean hasAnyTarget() {
        boolean validTarget = validTargetEntry.getDouble(0) > 0.5;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Target", validTarget);
        }

        return validTarget;
    }

    /**
     * Determines if a given height threshold is within a valid range
     * @param targetHeightThreshold The given height of a target
     * @return Whether the the given height threshold is greater than the target's angle relative to Y-axis (in Euclidean coordinates)
     */
    public static boolean hasValidHeight(double targetHeightThreshold) {
        boolean validHeight = getTargetYAngle() < targetHeightThreshold;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Height", validHeight);
        }

        return validHeight;
    }

    /**
     * Determines if a ratio, given minimum and maximum values, is valid
     * @param minRatio The minimum ratio
     * @param maxRatio The maximum ratio
     * @return Whether the aspect ratio of a target is between the minimum and maximum ratio values  (e.g. 0 < x < 127, where "x" is the aspect ratio)
     */
    public static boolean hasValidBlueAspectRatio(double minRatio, double maxRatio) {
        double aspectRatio = getHorizontalSidelength() / getVerticalSidelength();
        boolean validRatio = aspectRatio > minRatio && aspectRatio < maxRatio;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Ratio", validRatio);
            SmartDashboard.putNumber("Aspect Ratio", aspectRatio);
        }

        return validRatio;
    }

    /**
     * Determines whether the orientation of a target, given an angle threshold, is valid.
     * @param angleThreshold The given angle threshold
     * @return Whether the skew of the target is less than or equal to the angle threshold
     */
    public static boolean hasValidBlueOrientation(double angleThreshold) {
        double skew = Math.abs(getTargetSkew());
        boolean validOrientation = Math.min(skew, 90.0 - skew) <= angleThreshold;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Skew", validOrientation);
            SmartDashboard.putNumber("Skew Value", Math.min(skew, 90.0 - skew));
        }

        return validOrientation;
    }

    // horiz. offsets from crosshair to target
    public static final double MIN_X_ANGLE = -27;
    public static final double MAX_X_ANGLE = 27;
    public static final double X_ANGLE_SHIFT = -1.5;
    private static NetworkTableEntry xAngleEntry = table.getEntry("tx");

    // Vert. offset from crosshair to target
    public static final double MIN_Y_ANGLE = -20.5;
    public static final double MAX_Y_ANGLE = 20.5;
    private static NetworkTableEntry yAngleEntry = table.getEntry("ty");
    
    /**
     * Determines the angle of the target, relative to the x-axis
     * @return The target's angle, relative the the x-axis (in Euclidean coordinates)
     */
    public static double getTargetXAngle() {
        double X_SHIFT = SmartDashboard.getNumber("X_SHIFT", 1000);
        if(X_SHIFT > 694) SmartDashboard.putNumber("X_SHIFT", X_ANGLE_SHIFT);
        return xAngleEntry.getDouble(0) + X_SHIFT;
    }

    /**
     * Determines the angle of the target, relative to the y-axis
     * @return The target's angle, relative to the y-axis (in Euclidean coordinates)
     */
    public static double getTargetYAngle() {
        return yAngleEntry.getDouble(0);
    }

    // target area; range 0-100%
    public static final double MIN_TARGET_AREA = 0;
    public static final double MAX_TARGET_AREA = 1;
    private static NetworkTableEntry targetAreaEntry = table.getEntry("ta");

    /**
     * Determines the percentage of the camera screen the target will take up
     * @return The percentage of the camera screen the target will take up as a decimal on a scale of 0-1.
     */
    public static double getTargetArea() {
        return Math.min(targetAreaEntry.getDouble(0) / 100.0, 1);
    }

    // skew from -90-0 degs.
    public static final double MIN_SKEW = -90;
    public static final double MAX_SKEW = 0;
    private static NetworkTableEntry targetSkewEntry = table.getEntry("ts");

    /**
     * Determines the skew of the target
     * @return the skew of the target
     */
    public static double getTargetSkew() {
        return targetSkewEntry.getDouble(0);
    }

    /**
     * Determines the latency of of the camera
     * @return Camera latency in milliseconds
     */
    public static double getLatencyMs() {
        return latencyEntry.getDouble(0) + IMAGE_CAPTURE_LATENCY;
    }

    // information returned from the func.
    public static final double MIN_SIDE_LENGTH = 0;
    public static final double MAX_SIDE_LENGTH = 320;

    private static NetworkTableEntry shortestSideLengthEntry = table.getEntry("tshort");

    /**
     * Determines the shortest side length of the target
     * @return Shortest side length of the target in pixels
     */
    public static double getShortestSidelength() {
        return shortestSideLengthEntry.getDouble(0);
    }

    private static NetworkTableEntry longestSideLengthEntry = table.getEntry("tlong");

    /**
     * Determines the longest side length of the target
     * @return Longest side length of the target in pixels
     */
    public static double getLongestSidelength() {
        return longestSideLengthEntry.getDouble(0);
    }

    private static NetworkTableEntry horizontalSideLengthEntry = table.getEntry("thor");

    /**
     * Determines the horizontal side length of the target
     * @return Horizontal side length of the target in pixels
     */
    public static double getHorizontalSidelength() {
        return horizontalSideLengthEntry.getDouble(0);
    }

    private static NetworkTableEntry verticalSideLengthEntry = table.getEntry("tvert");

    /**
     * Determines the vertical side length of the target
     * @return Vertical side length of the target in pixels
     */
    public static double getVerticalSidelength() {
        return verticalSideLengthEntry.getDouble(0);
    }
    
    /**
     * Camera controller LED modes
     * <ul>
     *  <li>{@link #PIPELINE}</li>
     *  <li>{@link #FORCE_OFF}</li>
     *  <li>{@link #FORCE_BLINK}</li>
     *  <li>{@link #FORCE_ON}</li>
     * </ul>
     */
    public enum LEDMode {
        /**
         * Uses LED mode set in pipeline
         */
        PIPELINE(0),

        /**
         * Forces LEDs to be turned off
         */
        FORCE_OFF(1),

        /**
         * Forces LEDs to blink on and off
         */
        FORCE_BLINK(2),

        /**
         * Forces LEDs to be turned on
         */
        FORCE_ON(3);
    
        /**
         * Initializes enumerations with associated values
         * @param value The value an enumeration is to be associated with
         */
        LEDMode(int value) {
            this.val = value;
        }

        /**
         * Accesses an enumeration's associated value
         * @return Associated value
         */
        public int getCodeValue() {
            return val;
        }

        private int val;
    }    
    
    
    private static NetworkTableEntry LEDModeEntry = table.getEntry("ledMode");
    
    /**
     * Sets LED mode of Limelight
     * @param mode The mode to set the Limelight LED to
     */
    public final void setLEDMode(LEDMode mode) {
        LEDModeEntry.setNumber(mode.getCodeValue());
    }

    /**
     * Camera modes
     * <ul>
     *  <li>{@link #VISION}</li>
     *  <li>{@link #DRIVER}</li>
     * </ul>
     */
    public enum CamMode {
        /**
         * Standard camera vision
         */
        VISION(0),

        /**
         * Use camera for driving purposes
         */
        DRIVER(1);

        /**
         * Initializes enumeration with associated value
         * @param value Value to associate enumeration with
         */
        CamMode(int value) {
            this.val = value;
        }

        /**
         * Accesses associated value
         * @return Associated value
         */
        public int getCodeValue() {
            return val;
        }

        private int val;
    };

    private static NetworkTableEntry camModeEntry = table.getEntry("camMode");

    /**
     * Sets Limelight camera mode
     * @param mode Mode to set Limelight camera to
     */
    public static void setCamMode(CamMode mode) {
        camModeEntry.setNumber(mode.getCodeValue());
    }

    // pipeline stuff
    private static NetworkTableEntry pipelineEntry = table.getEntry("pipeline");

    /**
     * Sets Limelight pipeline
     * @param pipeline Pipeline to set Limelight to
     */
    public static void setPipeline(int pipeline) {
        if (pipeline >= 0 && pipeline <= 9) {
            pipelineEntry.setNumber(pipeline);
        }
    }

    private static NetworkTableEntry getPipelineEntry = table.getEntry("getpipe");

    /**
     * Accesses Limelight pipeline
     * @return Pipeline Limelight is set to
     */
    public static double getPipeline() {
        return getPipelineEntry.getDouble(0);
    }
    
    /**
     * Camera streams
     * <ul>
     *  <li>{@link #STANDARD}</li>
     *  <li>{@link #PIP_MAIN}</li>
     *  <li>{@link #PIP_SECONDARY}</li>
     * </ul>
     */
    public enum CameraStream { 
        /**
         * Limelight with secondary camera
         */
        STANDARD(0),

        /**
         * Secondary camera inside Limelight's main camera
         */
        PIP_MAIN(1),

        /**
         * Display Limelight inside Limelight's camera
         */
        PIP_SECONDARY(2);

        /**
         * Initialize enumeration with associated value
         */
        CameraStream(int value) {
            this.val = value;
        }

        /**
         * Accesses associated value
         * @return Associated value
         */
        public int getCodeValue() {
            return val;
        }

        private int val;
    };
    
    private static NetworkTableEntry CameraStreamEntry = table.getEntry("stream");

    /**
     * Set Limelight's camera stream
     * @param stream The camera stream to set the Limelight to
     */
    public static void setCameraStream(CameraStream stream) {
        CameraStreamEntry.setNumber(stream.getCodeValue());
    }
    
    /**
     * Snapshot Modes
     * <ul>
     *  <li>{@link #STOP}</li>
     *  <li>{@link #TWO_PER_SECOND}</li>
     * </ul>
     */
    public enum SnapshotMode {
        /**
         * Stops taking snapshots
         */
        STOP(0),
//GO CODE GO CODE GOOOOO CODEEEEE!!!
        /**
         * Takes two snapshots per second
         */
        TWO_PER_SECOND(1);

        /**
         * Initializes enumeration with associated value
         * @param value
         */
        SnapshotMode(int value) {
            this.val = value;
        }

        /**
         * Accesses associated value
         * @return Associated value
         */
        public int getCodeValue() {
            return val;
        }

        private int val;
    };
    
    private static NetworkTableEntry SnapshotModeEntry = table.getEntry("snapshot");

    /**
     * Sets Limelight's snapshot mode
     * @param mode Snapshot mode to set Limelight to
     */
    public static void setSnapshotMode(SnapshotMode mode) {
        SnapshotModeEntry.setNumber(mode.getCodeValue());
    }
    public void Update_Limelight_Tracking(double turn)
    {
        
         
          double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
          double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
          //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
          m_LimelightTargetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  
          if (tv < 1.0)
          {
            m_LimelightHasValidTarget = false;
            m_LimelightSteerCommand = 0.0;
            return;
          }
  
          m_LimelightHasValidTarget = true;
  
          // Start with proportional steering
          double steer_cmd = tx * -turn;
          m_LimelightSteerCommand = steer_cmd;
  
          // try to drive forward until the target area reaches our desired area
         /* double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K; */
    }
/* 
    public double targetArea(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }*/
}