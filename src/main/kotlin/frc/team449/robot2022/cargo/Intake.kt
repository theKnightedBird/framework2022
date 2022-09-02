package frc.team449.robot2022.cargo

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class Intake (val intakeMotor : WrappedMotor,
              val shooterMotor : WrappedMotor,
              var shooterMotorVoltage : Double)
  : SubsystemBase(){

    fun runIntake(){
      intakeMotor.set(CargoConstants.FEEDER_OUTPUT)
      shooterMotor.set(-shooterMotorVoltage)
    }

    fun runIntakeReverse(){
      intakeMotor.set(-CargoConstants.FEEDER_OUTPUT)
      shooterMotor.set(-shooterMotorVoltage)
    }

}
