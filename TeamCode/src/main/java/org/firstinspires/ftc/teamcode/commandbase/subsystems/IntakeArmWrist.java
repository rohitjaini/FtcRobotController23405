package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.tuning.PIDControllerWrapper;

public class IntakeArmWrist extends Subsystem {

    public static final IntakeArmWrist INSTANCE = new IntakeArmWrist();
    private IntakeArmWrist() { }

    public MotorEx intakeArmMotor;

    public Servo rightWristServo;

    public String intakeArmMotorName = "intakeArmMotor";
    public String rightWristServoName = "rightWristServo";

    public PIDControllerWrapper intakeArmController = new PIDControllerWrapper(new PIDController(0,0,0), 0,10);




    @Override
    public void initialize() {
        rightWristServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, rightWristServoName);
        intakeArmMotor = new MotorEx(intakeArmMotorName);
    }
}
