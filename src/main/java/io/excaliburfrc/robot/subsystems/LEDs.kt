package io.excaliburfrc.robot.subsystems

import edu.wpi.first.wpilibj.PWM
import io.excaliburfrc.robot.Constants.LED_PORT
import kotlin.properties.Delegates

object LEDs {
    private val leds = PWM(LED_PORT)

    enum class LedMode(val value: Double) {
        BLUE(0.87), RED(0.61), GREEN(0.73), YELLOW(0.67), RAINBOW(-0.97), OFF(0.99)


    }

    var mode: LedMode by Delegates.observable(LedMode.BLUE){ _, _, newValue ->
            leds.speed = newValue.value
        }
}
