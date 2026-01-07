package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.GoalFinder.gfGoalDistance
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import kotlin.math.pow

val goal = Pose(0.0, 141.0)
val shooterToGoalZSqrd = (46.0 - 13.5).pow(2.0)

object ShooterController {
    data class ShotParameters(val distance: Double, val velocity: Double, val angle: Double)

    private val shooterLookupTable: Map<Double, ShotParameters> = mapOf(
//Distance      to ShotParameters(Dist,            Velocity,        Angle)                   // Origin (X, Y)
        45.53   to ShotParameters(45.53, 985.0, 0.680),           // Pair(24, 120) //checked
        52.85   to ShotParameters(52.85, 1020.0,0.620),           // Pair(36, 120) //checked v2
        61.65   to ShotParameters(61.65, 1000.0, 0.560),          // Pair(48, 120) //checked
        66.70   to ShotParameters(66.70, 1020.0, 0.550),          // Pair(48, 108) //checked v2
        73.38   to ShotParameters(73.38, 1100.0, 0.580),          // Pair(48, 96) //checked
        81.74   to ShotParameters(81.74, 1180.0, 0.540),          // Pair(72, 120) // checked
        85.61   to ShotParameters(85.61, 1150.0, 0.530),          // Pair(72, 108) //checked v2
        90.91   to ShotParameters(90.91, 1230.0, 0.530),          // Pair(72, 96) //checked
        103.50  to ShotParameters(103.50, 1260.0, 0.500),         // Pair(96, 120) //checked
        104.89  to ShotParameters(104.89, 1255.0, 0.520),         // Pair(72, 72) //checked
        110.89  to ShotParameters(110.89, 1290.0, 0.500),         // Pair(96, 96) //checked
        141.17  to ShotParameters(141.17, 1480.0, 0.500),         // Pair(72, 24) //checked
        141.43  to ShotParameters(141.43, 1480.0, 0.500),         // Pair(48, 12) //checked
        151.27  to ShotParameters(151.27, 1560.0, 0.520),         // Pair(72, 12) //checked
        164.05  to ShotParameters(164.05, 1600.0, 0.520),         // Pair(96, 12) //checked
    ).toSortedMap()

    private fun lerp(x: Double, x0: Double, x1: Double, y0: Double, y1: Double): Double {
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    }


    /**
     * Finds the shot parameters for the distance closest to the target.
     * If the distance is within a certain threshold it returns the entry.
     */

    fun getShot(distance: Double): ShotParameters? {
        val keys = shooterLookupTable.keys.sorted()
        val values = shooterLookupTable.values

        // Clamp to bounds
        if (distance !in keys.first()..keys.last()) {
            return null
        }

        var low = 0
        var high = keys.size - 1

        while (low <= high) {
            val mid = (low + high) ushr 1
            val midValue = keys[mid]

            when {
                distance < midValue -> high = mid - 1
                distance > midValue -> low = mid + 1
                else -> return shooterLookupTable[midValue]!!
            }
        }

        // At this point:
        // high < low
        // high is index of lower bound
        // low is index of upper bound

        val lowerKey = keys[high]
        val upperKey = keys[low]

        val lower = shooterLookupTable[lowerKey]!!
        val upper = shooterLookupTable[upperKey]!!

        return ShotParameters(
            distance = distance,
            velocity = lerp(distance, lowerKey, upperKey, lower.velocity, upper.velocity),
            angle = lerp(distance, lowerKey, upperKey, lower.angle, upper.angle)
        )
    }

    fun applyShot(params: ShotParameters) {
        // Set the hood angle
        ShooterAngle.targetPosition = params.angle

        // Schedule the movement and the flywheel spin
        CommandManager.scheduleCommand(
            ShooterAngle.update()
        )

        CommandManager.scheduleCommand(
            Shooter.spinAtSpeed(params.velocity)
        )
    }
}
