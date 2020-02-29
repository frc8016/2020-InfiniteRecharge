/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public final class ValueConstants{
		public static final double kIntakeMotorSpeedScalar = .5;
		public static final double kCageSpeedScalar = .5;
	}
	public final class MotorConstants{

		public static final int kLeftMotor1Port = 0;
		public static final int kLeftMotor2Port = 1;
		public static final int kRightMotor1Port = 2;
		public static final int kRightMotor2Port = 3;
		public static final int kIntakeMotorPort = 0;
		public static final int kCageUpperMotor1Port = 1;
		public static final int kCageUpperMotor2Port = 2;
		public static final int kCageLowerMotor1Port = 3;
		public static final int kCageLowerMotor2Port = 4;
        
	}
	
	public final class OiConstants{
		public static final int kDriverJoystickPort = 0;
		public static final int kOperatorControllerPort = 1;

	}

	public final class PneumaticsConstants{
		public static final int kIntakeSolenoidPort1 = 0;
		public static final int kIntakeSolenoidPort2 = 1;
	}

	public final class XboxConstants{
		public static final int kLeftAxisX = 0;
		public static final int kLeftAxisY = 1;
		public static final int kRightAxisX = 4;
		public static final int kRightAxisY = 5;

		public static final int kTriggerLeft = 2;
		public static final int kTriggerRight = 3;

		public static final int kButtonLeft = 4;
		public static final int kButtonRight = 5;

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

