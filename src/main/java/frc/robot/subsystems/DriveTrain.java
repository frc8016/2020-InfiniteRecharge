/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SensorConstants;

public class DriveTrain extends SubsystemBase {

  //Create Left Motor Group
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(
      new WPI_VictorSPX(MotorConstants.kLeftMotor1Port), new WPI_VictorSPX(MotorConstants.kLeftMotor2Port));

  //Create Right Motor Group
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new WPI_VictorSPX(MotorConstants.kRightMotor1Port), new WPI_VictorSPX(MotorConstants.kRightMotor2Port));

  //Create encoders
  private final Encoder m_leftEncoder = new Encoder(SensorConstants.kLeftDrivetrainEncoderPort1, SensorConstants.kLeftDrivetrainEncoderPort2);
  private final Encoder m_rightEncoder = new Encoder(SensorConstants.kRightDrivetrainEncoderPort1, SensorConstants.kRightDrivetrainEncoderPort2);

  //Create gyro
  // private final Gyro m_gyro = new ADXRS450_Gyro();

  //Create Robot Drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  //Create Odometry
  private final DifferentialDriveOdometry m_Odometry;



  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    //Set the distance traveled per encoder pulse
    m_leftEncoder.setDistancePerPulse(SensorConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(SensorConstants.kEncoderDistancePerPulse);

    resetEncoders();

    m_Odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getVisionHeading()));
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(Rotation2d.fromDegrees(getVisionHeading()), getVisionPoseFromNT()[0], getVisionPoseFromNT()[1]);
  }

  //Returns current Pose of robot
  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }


  //Allow access to wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  //Create Arcade Drive. Used for manual driving
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  //Set motors to specific voltages. Important for auto as it compensates for voltage drop from battery during match
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  //Resets encoders
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  //Gets average encoder distance
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  //Provide a max output for the drivetrain
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  //Get Gyro Heading
  // public double getGyroHeading() {
  //   return Math.IEEEremainder(m_gyro.getAngle(), 360) * (SensorConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  //Vision
  public double[] getVisionPoseFromNT() {
    double[] targetPoseDefault = {0.0, 0.0, 0.0};
  
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable visionTable = inst.getTable("chameleon-vision").getSubTable("RobotCamera");

    NetworkTableEntry targetPose = visionTable.getEntry("targetPose");

    return targetPose.getDoubleArray(targetPoseDefault);
  }

  public double getVisionHeading() {
    return getVisionPoseFromNT()[2];
  }
}