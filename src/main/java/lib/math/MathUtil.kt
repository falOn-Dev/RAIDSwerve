package lib.math

fun wrapDegrees(degrees: Double): Double {
    var result = degrees % 360.0

    if (result < 0) {
        result += 360.0
    }

    return result
}