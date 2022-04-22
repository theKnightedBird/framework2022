package frc.team449.system.motor

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.REVLibError
import com.revrobotics.SparkMaxLimitSwitch
import edu.wpi.first.wpilibj.RobotController
import frc.team449.system.encoder.Encoder
import org.jetbrains.annotations.NotNull

/**
 * Create a Spark Max with the given configurations
 * 
 * @param name The motor's name
 * @param id The motor's CAN ID
 */
fun createSparkMax(
  name: String,
  id: Int,
  encCreator: (CANSparkMax, String) -> Encoder,
  enableBrakeMode: Boolean = true,
  inverted: Boolean = false,
  currentLimit: Int = 0,
  enableVoltageComp: Boolean = true,
  slaveSparks: Map<Int, Boolean> = mapOf(),
  controlFrameRateMillis: Int = -1,
  statusFrameRatesMillis: Map<CANSparkMaxLowLevel.PeriodicFrame, Int> = mapOf()
): WrappedMotor {
  val motor = CANSparkMax(
    id,
    CANSparkMaxLowLevel.MotorType.kBrushless
  )
  if (motor.getLastError() != REVLibError.kOk) {
    System.out.println(
      "Motor could not be constructed on port " +
        id +
        " due to error " +
        motor.getLastError()
    )
  }

  motor.restoreFactoryDefaults()

  val enc = encCreator(motor, name + "Enc")

  val brakeMode =
    if (enableBrakeMode) CANSparkMax.IdleMode.kBrake
    else CANSparkMax.IdleMode.kCoast

  motor.setInverted(inverted)
  // Set brake mode
  motor.setIdleMode(brakeMode)

  // Set frame rates
  if (controlFrameRateMillis >= 1) {
    // Must be between 1 and 100 ms.
    motor.setControlFramePeriodMs(controlFrameRateMillis)
  }

  for ((statusFrame, period) in statusFrameRatesMillis) {
    motor.setPeriodicFramePeriod(statusFrame, period)
  }

  // Set the current limit if it was given
  if (currentLimit > 0) {
    motor.setSmartCurrentLimit(currentLimit)
  }

  if (enableVoltageComp) {
    motor.enableVoltageCompensation(RobotController.getBatteryVoltage())
  } else {
    motor.disableVoltageCompensation()
  }

  for ((slavePort, slaveInverted) in slaveSparks) {
    val slave = createFollowerSpark(slavePort)
    slave.restoreFactoryDefaults()
    slave.follow(motor, slaveInverted)
    slave.setIdleMode(brakeMode)
    if (currentLimit > 0) {
      slave.setSmartCurrentLimit(currentLimit)
    }
    slave.burnFlash()
  }

  motor.burnFlash()

  return WrappedMotor(name, motor, enc)
}

/**
 * Create a Spark that will follow another Spark
 *
 * @param port The follower's CAN ID
 */
@NotNull
private fun createFollowerSpark(port: Int): CANSparkMax {
  val follower = CANSparkMax(
    port,
    CANSparkMaxLowLevel.MotorType.kBrushless
  )

  follower
    .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
    .enableLimitSwitch(false)
  follower
    .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
    .enableLimitSwitch(false)

  follower.setPeriodicFramePeriod(
    CANSparkMaxLowLevel.PeriodicFrame.kStatus0,
    100
  )
  follower.setPeriodicFramePeriod(
    CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
    100
  )
  follower.setPeriodicFramePeriod(
    CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
    100
  )

  return follower
}