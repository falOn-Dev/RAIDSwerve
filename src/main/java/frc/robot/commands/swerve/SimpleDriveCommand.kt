package frc.robot.commands.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import java.util.function.DoubleSupplier

class SimpleDriveCommand(
    forwards: DoubleSupplier,
    strafe: DoubleSupplier,
    rotation: DoubleSupplier
    ) : Command() {
    private val swerveSubsystem = SwerveSubsystem
    private val forwards: DoubleSupplier
    private val strafe: DoubleSupplier
    private val rotation: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
        this.forwards = forwards
        this.strafe = strafe
        this.rotation = rotation
    }

    override fun initialize() {}

    override fun execute() {
        val speeds: ChassisSpeeds = ChassisSpeeds(forwards.asDouble, strafe.asDouble, rotation.asDouble)
        swerveSubsystem.setChassisSpeeds(speeds)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
