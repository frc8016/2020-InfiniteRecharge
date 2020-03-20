/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoAim extends CommandBase {
  /**
   * Creates a new AutoAim.
   */

  private DriveTrain m_driveTrain;

  public AutoAim(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    addRequirements(m_driveTrain);
  }
  NetworkTableInstance inst;
  NetworkTable visionTable;
  NetworkTableEntry targetPose;
  TrajectoryConfig trajectoryConfig;
  double[] defaultTargetPoseArray = {0.0, 0.0, 0.0};

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //Set Up NetworkTables
    inst = NetworkTableInstance.getDefault();
    visionTable = inst.getTable("chameleon-vision").getSubTable("RobotCamera");

    //Create voltage constraint
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter), AutoConstants.kDriveKinematics, 10);

    trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelMetersPerSecondSquared)
      .setKinematics(AutoConstants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get Target Location from rasPi
    targetPose = visionTable.getEntry("targetPose");

    double[] targetPoseArray = targetPose.getDoubleArray(defaultTargetPoseArray);

    //Create Trajectory such that target is at 0,0,0
    Trajectory robotCentricTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(targetPoseArray[0], targetPoseArray[1], new Rotation2d(targetPoseArray[2])),
      null,
      new Pose2d(0, 0, new Rotation2d(0)),
      trajectoryConfig);

    Trajectory worldCentricTrajectory = robotCentricTrajectory.relativeTo(new Pose2d(targetPoseArray[0], targetPoseArray[1], new Rotation2d(targetPoseArray[2])));
    //Create Robot Origin Pose such that the target is at 0

    RamseteCommand ramseteCommand = new RamseteCommand(
      worldCentricTrajectory, 
      m_driveTrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter), 
      AutoConstants.kDriveKinematics, 
      m_driveTrain::getWheelSpeeds, 
      new PIDController(PIDConstants.kPDriveVel, PIDConstants.kIDriveVel, PIDConstants.kDDriveVel), 
      new PIDController(PIDConstants.kPDriveVel, PIDConstants.kIDriveVel, PIDConstants.kDDriveVel), 
      m_driveTrain::tankDriveVolts, 
      m_driveTrain);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
