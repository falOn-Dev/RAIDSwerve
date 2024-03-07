package lib.pid

import com.revrobotics.SparkPIDController

fun SparkPIDController.setConfig(config: PIDConfig) {
    this.p = config.kP
    this.i = config.kI
    this.d = config.kD
    this.ff = config.kF
    this.iZone = config.kIz
}