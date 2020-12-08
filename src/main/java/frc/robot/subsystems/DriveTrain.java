/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.NetworkTableConstants;
import frc.robot.Constants.SensorConstants;

public class DriveTrain extends SubsystemBase {

  //Create Left Motor Group
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(new WPI_VictorSPX(MotorConstants.kLeftMotor1Port), new WPI_VictorSPX(MotorConstants.kLeftMotor2Port));

  //Create Right Motor Group
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new WPI_VictorSPX(MotorConstants.kRightMotor1Port), new WPI_VictorSPX(MotorConstants.kRightMotor2Port));

  //Create encoders
  private final Encoder m_leftEncoder = new Encoder(SensorConstants.kLeftDrivetrainEncoderPort1, SensorConstants.kLeftDrivetrainEncoderPort2);
  private final Encoder m_rightEncoder = new Encoder(SensorConstants.kRightDrivetrainEncoderPort1, SensorConstants.kRightDrivetrainEncoderPort2);

  //Create IMU
  private final PigeonIMU m_IMU = new PigeonIMU(SensorConstants.kPigeonIMUPort);

  //Create Robot Drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  //Create Arcade Drive. Used for manual driving
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  //Gets speed of entire robot
  public ChassisSpeeds getChassisSpeeds() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    ChassisSpeeds chassisSpeeds = AutoConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);
    return chassisSpeeds;
  }

  //Gets robot's forward speed
  public double getLinearVelocity() {
    return getChassisSpeeds().vxMetersPerSecond;
  }

  //Gets robots turning speed
  public double getAngularVelocity() {
    return getChassisSpeeds().omegaRadiansPerSecond;
  }

  //Get IMU status
  public ErrorCode IMUStatus() {
    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
    return m_IMU.getGeneralStatus(genStatus);
  }
  //Gets robot's heading as degrees from IMU
  public double getRobotHeadingDegrees() {
    double[] ypr = new double[3];
    m_IMU.getYawPitchRoll(ypr);
    return -ypr[0];
  }

  //Get robot heading as rotation2d from IMU
  public Rotation2d getRobotHeadingRotation2d() {
    Rotation2d robotRotation = new Rotation2d(Math.toRadians(getRobotHeadingDegrees()));
    return robotRotation;
  }

  //Zero robot yaw
  public void zeroRobotHeading() {
    m_IMU.setYaw(0);
  }

  //Set robot heading to a value
  public void setRobotHeading(double heading) {
    m_IMU.setYaw(heading);
  }


  //Get encoder speeds
  public double[] getEncoderSpeeds() {
    double[] encoderSpeeds = {m_leftEncoder.getRate(), m_rightEncoder.getRate()};
    return encoderSpeeds;
  }

  //Get loading bay pose
  public Pose2d loadingBayPose() {
    NetworkTableEntry pipeline = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("loadingBayCamera").getEntry("pipeline");

    pipeline.setDouble(NetworkTableConstants.loadingBayPipelineIndex);

    Double[] poseArray = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("loadingBayCamera").getEntry("targetPose").getDoubleArray(new Double[]{0.0,0.0,0.0});
    Rotation2d robotRotation = new Rotation2d(Math.toRadians(poseArray[2]));

    return new Pose2d(poseArray[0], poseArray[1], robotRotation);
  }

    //Get loading bay pose
    public Pose2d PowerPortPose() {
      NetworkTableEntry pipeline = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("loadingBayCamera").getEntry("pipeline");
  
      pipeline.setDouble(NetworkTableConstants.powerPortPipelineIndex);
  
      Double[] poseArray = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("loadingBayCamera").getEntry("targetPose").getDoubleArray(new Double[]{0.0,0.0,0.0});
      Rotation2d robotRotation = new Rotation2d(Math.toRadians(poseArray[2]));
  
      return new Pose2d(poseArray[0], poseArray[1], robotRotation);
    }
  

  /*Robot-centric to Field-centric coordinate converter. Requires a object that is known in both coordinate systems,
  such as the loading bay or goal. Used for converting data sent from vision camera to field coordinates, allowing
  the robot to know it's position on the field.    Rc = Robot centric, Fc = Field centric*/
  public Pose2d convertRobotCentricToFieldCentricCoordinates(Double[] knownItemInRobotCoordinates, Double[] knownItemInFieldCoordinates) {
    int RcQuadrant;
   
    double RcX = knownItemInRobotCoordinates[0];
    double RcY = knownItemInRobotCoordinates[1];
    double RcTheta = knownItemInRobotCoordinates[2];

    double FcX = knownItemInFieldCoordinates[0];
    double FcY = knownItemInFieldCoordinates[1];

    //Determine quadrant of object in robot centric coordinates. Allows reference angle to be converted to actual angle.
    if (RcX >= 0) {
      if (RcY >= 0) {
        RcQuadrant = 1;
      } else {
        RcQuadrant = 4;
      }
    } else {
      if (RcY >= 0) {
        RcQuadrant = 2;
      } else {
        RcQuadrant = 3;
      }
    }


    //Solve for values used in both conversion equations (To avoid doing it twice, and hopefully optimize the performance of the program)
    double r = Math.sqrt(RcX*RcX + RcY*RcY);
    double theta = Utilities.convertReferenceAngleToActualAngle(Math.atan(Math.abs(RcY/RcX)), RcQuadrant) + RcTheta;

    double fieldCoordX = FcX - r * Math.cos(theta);
    double fieldCoordY = FcY - r * Math.sin(theta);
    Rotation2d fieldCoordTheta = new Rotation2d(RcTheta);

    Pose2d robotCoordinates = new Pose2d(fieldCoordX, fieldCoordY, fieldCoordTheta);
    return robotCoordinates;
  }


  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    //Set the distance traveled per encoder pulse
    m_leftEncoder.setDistancePerPulse(SensorConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(SensorConstants.kEncoderDistancePerPulse);
  }

  @Override
  public void periodic() {

    //Publish velocities to Smart Dashboard
    SmartDashboard.putNumber("Linear Velocity", getLinearVelocity());
    SmartDashboard.putNumber("Angular Velocity", getAngularVelocity());
  }

}

//This is a change