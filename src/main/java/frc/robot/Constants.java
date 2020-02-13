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
	}
	public final class MotorConstants{

		public static final int kLeftMotor1Port = 0;
		public static final int kLeftMotor2Port = 1;
		public static final int kRightMotor1Port = 2;
		public static final int kRightMotor2Port = 3;
		public static final int kIntakeMotorPort = 0;
        
	}
	
	public final class OiConstants{
		public static final int kDriverJoystickPort = 0;

	}

	public final class PneumaticsConstants{
		public static final int kIntakeSolenoidPort1 = 0;
		public static final int kIntakeSolenoidPort2 = 1;
	}
}

