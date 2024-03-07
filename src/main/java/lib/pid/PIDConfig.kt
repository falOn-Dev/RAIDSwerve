package lib.pid

data class PIDConfig(
    val kP: Double,
    val kI: Double,
    val kD: Double,
    val kF: Double,
    val kIz: Double
)
