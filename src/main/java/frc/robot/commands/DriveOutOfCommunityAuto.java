// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
//import frc.robot.commands.limelightControls.TargetTrackingCommand;
//import frc.robot.commands.limelightControls.driveForwardToTarget;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.Limelight;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOutOfCommunityAuto extends SequentialCommandGroup {
  /** Creates a new DriveOutOfCommunityAuto. */
  public DriveOutOfCommunityAuto(DriveTrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveTimeCommand(AutoConstants.kAutoDriveSpeed,0, drive, 0.5),
      new DriveTimeCommand(-0.4,0, drive, 0.9),
      new DriveTimeCommand(0.4,0, drive, 4.3)
      //TO TEST LATER:
      //, new LimelightTrackingCommand(lime, drive).withTimeout(0.8)
      //, new TargetTrackingCommand(lime, drive).withTimeout(1.5), 
      //new driveForwardToTarget(lime, drive).withTimeout(1.3)
    );
  }
}
