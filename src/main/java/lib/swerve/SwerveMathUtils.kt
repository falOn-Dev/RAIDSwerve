package lib.swerve

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

enum class DesiredMeasurements {
    ROTATIONS,
    DEGREES,
    RADIANS
}

fun calculateAngleRatio(gearRatio: Double, desiredMeasurements: DesiredMeasurements): Double {
    when(desiredMeasurements){
        DesiredMeasurements.ROTATIONS -> return 1 / gearRatio
        DesiredMeasurements.DEGREES -> return 360 / gearRatio
        DesiredMeasurements.RADIANS -> return (2 * Math.PI) / gearRatio
    }
}

fun calculateDriveRatio(gearRatio: Double, wheelDiameter: Measure<Distance>): Double {
    return (wheelDiameter.`in`(Units.Meters) * Math.PI) / gearRatio
}

fun calculateAngleVelocityRatioRadPerSec(gearRatio: Double): Double {
    return ((2 * Math.PI) * (gearRatio)) / 60
}

fun calculateDriveVelocityMetersPerSec(gearRatio: Double, wheelDiameter: Measure<Distance>): Double {
    return (wheelDiameter.`in`(Units.Meters) * Math.PI * gearRatio) / 60
}