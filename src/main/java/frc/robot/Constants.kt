package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.math.util.Units as OldUnits
import lib.pid.PIDConfig
import lib.swerve.*

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants
{
    object OperatorConstants
    {
        const val DRIVER_CONTROLLER_PORT = 0
    }

    object RatioConstants {
        val ANGLE_MOTOR_RATIO = calculateAngleRatio(150/7.0, DesiredMeasurements.RADIANS)
        val DRIVE_MOTOR_RATIO = calculateDriveRatio(8.14, Units.Inch.of(4.0))

        val ANGLE_VELOCITY_RAD_PER_SEC = calculateAngleVelocityRatioRadPerSec(150/7.0)
        val DRIVE_VELOCITY_METERS_PER_SEC = calculateDriveVelocityMetersPerSec(8.14, Units.Inch.of(4.0))
    }

    object PIDConstants {
        val DRIVE_PID = PIDConfig(0.01, 0.0, 0.0, 0.0, 0.0)
        val ANGLE_PID = PIDConfig(0.5, 0.0001, 0.001, 0.0, 0.0)
    }

}



