package lib.motors

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkMax

fun CANSparkBase.setLoopRampRate(rate: Double) {
    this.setOpenLoopRampRate(rate)
    this.setClosedLoopRampRate(rate)
}