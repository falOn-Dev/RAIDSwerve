package frc.robot.subsystems

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants
import lib.swerve.SwerveModule

object SwerveSubsystem : SubsystemBase() {

    object Modules {
        val MODULE_OFFSET = Units.inchesToMeters(10.3)

        val frontLeftModule: SwerveModule = SwerveModule(
            1, 2, 3,
            Constants.PIDConstants.DRIVE_PID, Constants.PIDConstants.ANGLE_PID,
            Translation2d(MODULE_OFFSET, MODULE_OFFSET),
            204.53,
            true,
            true,
            0
        )

        val frontRightModule: SwerveModule = SwerveModule(
            4, 5, 6,
            Constants.PIDConstants.DRIVE_PID, Constants.PIDConstants.ANGLE_PID,
            Translation2d(MODULE_OFFSET, -MODULE_OFFSET),
            287.13,
            true,
            true,
            1
        )

        val backLeftModule: SwerveModule = SwerveModule(
            7, 8, 9,
            Constants.PIDConstants.DRIVE_PID, Constants.PIDConstants.ANGLE_PID,
            Translation2d(-MODULE_OFFSET, MODULE_OFFSET),
            129.64,
            true,
            true,
            2
        )

        val backRightModule: SwerveModule = SwerveModule(
            10, 11, 12,
            Constants.PIDConstants.DRIVE_PID, Constants.PIDConstants.ANGLE_PID,
            Translation2d(-MODULE_OFFSET, -MODULE_OFFSET),
            99.23,
            true,
            true,
            3
        )
    }

    val modules: Array<SwerveModule> = arrayOf(
        Modules.frontLeftModule,
        Modules.frontRightModule,
        Modules.backLeftModule,
        Modules.backRightModule
    )

    val kinematics = SwerveDriveKinematics(
        Modules.frontLeftModule.position,
        Modules.frontRightModule.position,
        Modules.backLeftModule.position,
        Modules.backRightModule.position
    )

    val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d(),
        getModulePositions(),
        Pose2d()
    )

    val pigeon: Pigeon2 = Pigeon2(13)

    init {
        zeroModules()
        resetIMU()
        zeroHeading()
    }

    fun zeroModules() {
        modules.forEach { it.zero() }
    }

    fun resetIMU() {
        val configs: Pigeon2Configuration = Pigeon2Configuration()
        configs.MountPose.MountPoseYaw = 0.0
        configs.MountPose.MountPoseRoll = 0.0
        configs.MountPose.MountPosePitch = 0.0

        pigeon.configurator.apply(configs)
    }

    fun zeroHeading() {
        pigeon.setYaw(0.0)
    }

    fun getModulePositions(): Array<SwerveModulePosition> {
        return arrayOf(
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        )
    }

    fun setChassisSpeeds(speeds: ChassisSpeeds) {
        var states: Array<SwerveModuleState> = kinematics.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0)
        modules.forEachIndexed() { index, module ->
            module.setDesiredState(states[index])
            println("Setting Module[$index] to ${states[index].speedMetersPerSecond}")

        }
    }



}