package lib.swerve

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.Constants
import lib.math.wrapDegrees
import lib.motors.setLoopRampRate
import lib.pid.PIDConfig
import lib.pid.setConfig

class SwerveModule {
    val driveMotor: CANSparkMax
    val angleMotor: CANSparkMax
    val absoluteEncoder: CANcoder

    val position: Translation2d
    val angleOffset: Double
    val angleMotorInverted: Boolean
    val driveMotorInverted: Boolean

    val table: NetworkTable
    val index: Int

    val drivePID: SparkPIDController
    val anglePID: SparkPIDController
    val drivePIDConfig: PIDConfig
    val anglePIDConfig: PIDConfig

    constructor(
        driveMotorID: Int,
        angleMotorID: Int,
        absoluteEncoderID: Int,
        drivePIDConfig: PIDConfig,
        anglePIDConfig: PIDConfig,
        position: Translation2d,
        angleOffset: Double,
        angleMotorInverted: Boolean,
        driveMotorInverted: Boolean,
        index: Int
    ) {
        this.index = index
        table = NetworkTableInstance.getDefault().getTable("Module[$index]")

        driveMotor = CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless)
        angleMotor = CANSparkMax(angleMotorID, CANSparkLowLevel.MotorType.kBrushless)
        absoluteEncoder = CANcoder(absoluteEncoderID)

        this.drivePIDConfig = drivePIDConfig
        this.anglePIDConfig = anglePIDConfig

        this.position = position
        this.angleOffset = angleOffset
        this.angleMotorInverted = angleMotorInverted
        this.driveMotorInverted = driveMotorInverted

        // Configure the things
        driveMotor.inverted = driveMotorInverted
        angleMotor.inverted = angleMotorInverted

        // Set the motors to brake mode
        // TODO: Make these brake
        driveMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        angleMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)

        driveMotor.enableVoltageCompensation(12.0)
        angleMotor.enableVoltageCompensation(12.0)

        driveMotor.setSmartCurrentLimit(40)
        angleMotor.setSmartCurrentLimit(20)

        driveMotor.setLoopRampRate(0.25)
        angleMotor.setLoopRampRate(0.25)

        driveMotor.encoder.velocityConversionFactor = Constants.RatioConstants.DRIVE_VELOCITY_METERS_PER_SEC
        driveMotor.encoder.positionConversionFactor = Constants.RatioConstants.DRIVE_MOTOR_RATIO

        angleMotor.encoder.velocityConversionFactor = Constants.RatioConstants.ANGLE_VELOCITY_RAD_PER_SEC
        angleMotor.encoder.positionConversionFactor = Constants.RatioConstants.ANGLE_MOTOR_RATIO

        this.drivePID = driveMotor.pidController
        this.anglePID = angleMotor.pidController

        drivePID.setConfig(drivePIDConfig)
        anglePID.setConfig(anglePIDConfig)

        anglePID.setPositionPIDWrappingEnabled(true)
        anglePID.setPositionPIDWrappingMinInput(0.0)
        anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI)

        anglePID.setOutputRange(-1.0, 1.0)
        drivePID.setOutputRange(-1.0, 1.0)

        zero()

        driveMotor.burnFlash()
        angleMotor.burnFlash()


    }

    fun getAbsoluteAngle(): Rotation2d {
        val rot = Units.rotationsToDegrees(absoluteEncoder.position.valueAsDouble)
        return Rotation2d.fromDegrees(rot - angleOffset)
    }

    fun getRelativeAngle(): Rotation2d {
        return Rotation2d.fromDegrees(angleMotor.encoder.position)
    }

    fun zero() {
        angleMotor.encoder.setPosition(getAbsoluteAngle().radians)
        anglePID.setReference(0.0, CANSparkBase.ControlType.kPosition)
        driveMotor.encoder.setPosition(0.0)
    }

    fun getState(): SwerveModuleState {
        return SwerveModuleState(
            driveMotor.encoder.velocity,
            getRelativeAngle()
        )
    }

    fun getPosition(): SwerveModulePosition {
        return SwerveModulePosition(
            driveMotor.encoder.position,
            getRelativeAngle()
        )
    }

    fun setDesiredState(state: SwerveModuleState) {
        val correctedState = SwerveModuleState.optimize(
            state,
            getRelativeAngle()
        )

        drivePID.setReference(
            correctedState.speedMetersPerSecond,
            CANSparkBase.ControlType.kVelocity
        )

        anglePID.setReference(
            correctedState.angle.radians,
            CANSparkBase.ControlType.kPosition
        )
    }

    fun resetEncoders() {
        driveMotor.encoder.setPosition(0.0)
        angleMotor.encoder.setPosition(getAbsoluteAngle().radians)
    }


}