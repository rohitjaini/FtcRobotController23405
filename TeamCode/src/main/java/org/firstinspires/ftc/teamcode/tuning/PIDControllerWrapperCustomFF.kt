package org.firstinspires.ftc.teamcode.tuning

import com.arcrobotics.ftclib.controller.PIDController
import com.rowanmcalpin.nextftc.core.control.controllers.Controller
import org.firstinspires.ftc.teamcode.tuning.Globals.*
import kotlin.math.cos

class PIDControllerWrapperCustomFF(
    val ftcLibController: PIDController,
    var setArmF: Double,
    override var setPointTolerance: Double
) : Controller {
    override var target: Double
        get() = ftcLibController.setPoint
        set(value) {
            ftcLibController.setPoint = value
        }

    override fun calculate(reference: Double): Double {

        val ff: Double = cos(Math.toRadians(target / armTicksInDegrees) - Math.toRadians(ARM_INITIAL_ANGLE) + Math.toRadians(90.0)) * setArmF

        return ftcLibController.calculate(reference) + ff
    }

    override fun reset() {
        ftcLibController.reset()
    }
}