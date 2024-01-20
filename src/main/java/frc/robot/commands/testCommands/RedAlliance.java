// package frc.robot.commands.testCommands;
// import edu.wpi.first.apriltag.*;
// import java.awt.image.BufferedImage;
// // private AprilTagDetectorJava detector;


// // Red Alliance: 1, 2, 3, 4, 5

// // Blue Alliance: 6, 7, 8, 9, 10

// public class RedAlliance {
//     private AprilTagDetector detector;

//     public RedAlliance() {
//         // Initialize the AprilTagDetector with the appropriate settings
//         detector = new AprilTagDetector();
//     }

//     public void processImage(BufferedImage image) {
//         // Detect AprilTags in the provided image
//         TagDetectionArray detections = detector.processImage(image);

//         // Process the detected tags
//         for (int i = 1; i < 6; i++) {
//             TagDetectionArray detection = detections.get(i);
//             // Access information about the detected tag (e.g., ID, pose)
//             int tagId = TagDetectionArray.id;
//             double[] translation = detection.cxy;

//             // Do something with the detected tag information
//             // (e.g., send data to other subsystems, update robot state)
//         }
//     }
// }

