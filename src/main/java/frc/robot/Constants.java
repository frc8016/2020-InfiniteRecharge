/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public final static class AutoConstants {
		public static final double ksVolts = 0.22; // Tune to actual robot!
		public static final double kvVoltSecondsPerMeter = 1.98; // Tune to actual robot!
		public static final double kaVoltSecondsSquaredPerMeter = 0.2; // Tune to actual robot!

		public static final double kPDriveVel = 8.5; // Tune to actual robot!

		public static final double kTrackWidthMeters = 0.69; // Tune to actual robot!

		public static final double kMaxSpeedMetersPerSecond = 3; // Tune to actual robot!
		public static final double kMaxAccelMetersPerSecondSquared = 3; // Tune to actual robot!

		// Baseline vaules for RAMSETE follwer in units of meters and seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;

		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackWidthMeters);
	}

	public final static class PIDConstants {

		public static final double kPDriveVel = 1; //Tune to actual robot!
		public static final double kIDriveVel = 0; //Tune to actual robot!
		public static final double kDDriveVel = 0; //Tune to actual robot!

	}
	public final static class ValueConstants{
		public static final double kIntakeMotorSpeedScalar = .5;
		public static final double kCageSpeedScalar = -.5;
		public static final double kClimbSpeedScalar = .5;
		public static final double kWheelDiameterMeters = 0.1524; 
	}
	public final static class MotorConstants{

		public static final int kLeftMotor1Port = 1; //VictorSPX over CAN
		public static final int kLeftMotor2Port = 2; //VictorSPX over CAN
		public static final int kRightMotor1Port = 3; //VictorSPX over CAN
		public static final int kRightMotor2Port = 4; //VictorSPX over CAN
		public static final int kIntakeMotorPort = 0; //Spark over PWM
		public static final int kCageUpperMotorPort = 2;//Spark over PWM
		public static final int kCageLowerMotorPort = 1;//Spark over PWM
		public static final int kClimbMotor1Port = 3; //Spark over PWM
		public static final int kClimbMotor2Port = 4; //Spark over PWM
        
	}

	public final static class SensorConstants{
		public static final int kLeftDrivetrainEncoderPort1 = 0;
		public static final int kLeftDrivetrainEncoderPort2 = 1;
		public static final int kRightDrivetrainEncoderPort1 = 2;
		public static final int kRightDrivetrainEncoderPort2 = 3;
		public static final boolean kLeftEncodersReversed = false;
		public static final boolean kRightEncodersReversed = false;
		public static final int kEncoderCPR = 1024;

		public static final double kEncoderDistancePerPulse = 
			(ValueConstants.kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR; //meters, tune to actual robot value

		public static final boolean kGyroReversed = false;
		public static final int kPigeonIMUPort = 5;

	}
	
	public final static class OiConstants{
		public static final int kDriverJoystickPort = 0;
		public static final int kOperatorControllerPort = 1;

	}

	public final class PneumaticsConstants{
		public static final int kIntakeSolenoidPort1 = 0;
		public static final int kIntakeSolenoidPort2 = 1;
		public static final int kFlapSolenoidPort1 = 2;
		public static final int kFlapSolenoidPort2 = 3;
		public static final int kClimbSolenoidPort1 = 4;
		public static final int kClimbSolenoidPort2 = 5;
	}

	public final class XboxConstants{
		public static final int kLeftAxisX = 0;
		public static final int kLeftAxisY = 1;
		public static final int kRightAxisX = 4;
		public static final int kRightAxisY = 5;

		public static final int kTriggerLeft = 2;
		public static final int kTriggerRight = 3;

		public static final int kButtonLeft = 5;
		public static final int kButtonRight = 6;

		public static final int kButtonA = 1;
		public static final int kButtonB = 2;
		public static final int kButtonX = 3;
		public static final int kButtonY = 4;

		public static final int kButtonBack = 7;
		public static final int kButtonMenu = 8;

		public static final int kButtonStickLeft = 9;
		public static final int kButtonStickRight = 10;
	}
}

