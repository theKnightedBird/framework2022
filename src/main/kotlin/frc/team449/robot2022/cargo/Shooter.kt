package frc.team449.robot2022.cargo

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class Shooter (val shooterMotor : WrappedMotor,
               var desiredVelocity : Double,
               val maxVelocity : Double,
               val motor : SimpleMotorFeedforward,
               val shooterPIDController : PIDController)
  : SubsystemBase(){
  private var flywheelOn : Boolean = false

  val trueDesiredVelocity : Double = desiredVelocity

  override fun periodic() {
    if (flywheelOn){
      shooterMotor.setVoltage(motor.calculate(desiredVelocity) + shooterPIDController.calculate(shooterMotor.velocity, desiredVelocity))
    }
    else shooterMotor.setVoltage(motor.calculate(0.0) + shooterPIDController.calculate(shooterMotor.velocity, 0.0))
  }

//  fun flywheelStatusSet(flywheelBool : Boolean) {
//    flywheelOn = flywheelBool
//    if(!flywheelOn){
//      desiredVelocity = 0.0
//    }
//    if(flywheelOn){
//      desiredVelocity = trueDesiredVelocity
//    }
//
//  }

  fun shoot(){
    flywheelOn = true
    //flywheelStatusSet(true)
  }

  fun stopShoot(){
    flywheelOn = false
    //flywheelStatusSet(false)


  }

}