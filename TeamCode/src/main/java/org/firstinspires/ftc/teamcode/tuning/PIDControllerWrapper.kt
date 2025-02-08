package org.firstinspires.ftc.teamcode.tuning

import com.arcrobotics.ftclib.controller.PIDController
import com.rowanmcalpin.nextftc.core.control.controllers.Controller

class PIDControllerWrapper(
    val ftcLibController: PIDController,
    val ff: Double,
    override var setPointTolerance: Double
) : Controller {
    override var target: Double
        get() = ftcLibController.setPoint
        set(value) {
            ftcLibController.setPoint = value
        }

    override fun calculate(reference: Double): Double {
        return ftcLibController.calculate(reference) + ff
    }

    override fun reset() {
        ftcLibController.reset()
    }
}