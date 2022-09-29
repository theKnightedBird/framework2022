package frc.team449.control.holonomic

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.annotations.Log

/**
 * @param modules the list of swerve modules on this drivetrain
 * @param ahrs the gyro that is mounted on the chassis
 * @param maxLinearSpeed the maximum translation speed of the chassis.
 * @param maxRotSpeed the maximum rotation speed of the chassis
 */
open class SwerveDrive(
  private val modules: List<SwerveModule>,
  val ahrs: AHRS,
  override val maxLinearSpeed: Double,
  override val maxRotSpeed: Double
) : SubsystemBase(), HolonomicDrive {
  init {
    // Zero out the gyro
    ahrs.calibrate()
    ahrs.reset()
  }
  private val kinematics = SwerveDriveKinematics(
    *this.modules
      .map { it.location }.toTypedArray()
  )

  private val odometry = SwerveDriveOdometry(this.kinematics, -ahrs.heading)

  @Log.ToString
  var desiredSpeeds = ChassisSpeeds()
  @Log.Graph
  var desiredSpeedsX = 0.0
  @Log.Graph
  var desiredSpeedsY = 0.0
  @Log.Graph
  var desiredSpeedsOmega = 0.0

  @Log.ToString
  var actualSpeeds = ChassisSpeeds()
  @Log.Graph
  var actualSpeedsX = 0.0
  @Log.Graph
  var actualSpeedsY = 0.0
  @Log.Graph
  var actualSpeedsOmega = 0.0

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
  }

  override val heading: Rotation2d
    @Log.ToString
    get() {
      return ahrs.heading
    }

  override var pose: Pose2d
    @Log.ToString
    get() {
      return this.odometry.poseMeters
    }
    set(value) {
      ahrs.reset()
      ahrs.heading = value.rotation
      this.odometry.resetPosition(value, value.rotation)
    }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {
    desiredSpeedsX = desiredSpeeds.vxMetersPerSecond
    desiredSpeedsY = desiredSpeeds.vyMetersPerSecond
    desiredSpeedsOmega = desiredSpeeds.omegaRadiansPerSecond
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    /** If any module is going faster than the max speed,
     *  apply scaling down */
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      DriveConstants.MAX_ATTAINABLE_MK4I_SPEED
    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    this.odometry.update(
      heading,
      *this.modules
        .map { it.state }.toTypedArray()
    )

    for (module in modules)
      module.update()

    actualSpeeds = kinematics.toChassisSpeeds(
      *this.modules
        .map { it.state }.toTypedArray()
    )
    actualSpeedsX = actualSpeeds.vxMetersPerSecond
    actualSpeedsY = actualSpeeds.vyMetersPerSecond
    actualSpeedsOmega = actualSpeeds.omegaRadiansPerSecond
  }

  companion object {
    /**
     * Create a swerve drivetrain
     *
     * @param ahrs Gyro used for robot heading
     * @param maxLinearSpeed Max speed (m/s) at which the robot can translate
     * @param maxRotSpeed Max speed (rad/s) at which the robot can turn in place
     * @param frontLeftDriveMotor
     * @param frontRightDriveMotor
     * @param backLeftDriveMotor
     * @param backRightDriveMotor
     * @param frontLeftTurnMotor
     * @param frontRightTurnMotor
     * @param backLeftTurnMotor
     * @param backRightTurnMotor
     * @param wheelbase the length of the sides of the robot measured from the center of the wheels
     * @param trackwidth the length of the front and back of the robot measured from center of wheels
     * @param driveMotorController Supplier to make copies of the same driving PID controllers for all the modules
     * @param driveFeedforward Driving feedforward for all the modules
     */
    fun swerveDrive(
      ahrs: AHRS,
      maxLinearSpeed: Double,
      maxRotSpeed: Double,
      frontLeftDriveMotor: WrappedMotor,
      frontRightDriveMotor: WrappedMotor,
      backLeftDriveMotor: WrappedMotor,
      backRightDriveMotor: WrappedMotor,
      frontLeftTurnMotor: CANSparkMax,
      frontLeftEncoder: AbsoluteEncoder,
      frontRightTurnMotor: CANSparkMax,
      frontRightEncoder: AbsoluteEncoder,
      backLeftTurnMotor: CANSparkMax,
      backLeftEncoder: AbsoluteEncoder,
      backRightTurnMotor: CANSparkMax,
      backRightEncoder: AbsoluteEncoder,
      wheelbase: Double,
      trackwidth: Double,
      driveMotorController: () -> PIDController,
      driveFeedforward: SimpleMotorFeedforward
    ): SwerveDrive {
      val modules = listOf(
        SwerveModule.create(
          "FLModule",
          frontLeftDriveMotor,
          frontLeftTurnMotor,
          frontLeftEncoder,
          driveMotorController(),
          driveFeedforward,
          Translation2d(wheelbase / 2, trackwidth / 2)
        ),
        SwerveModule.create(
          "FRModule",
          frontRightDriveMotor,
          frontRightTurnMotor,
          frontRightEncoder,
          driveMotorController(),
          driveFeedforward,
          Translation2d(wheelbase / 2, - trackwidth / 2)
        ),
        SwerveModule.create(
          "BLModule",
          backLeftDriveMotor,
          backLeftTurnMotor,
          backLeftEncoder,
          driveMotorController(),
          driveFeedforward,
          Translation2d(- wheelbase / 2, trackwidth / 2)
        ),
        SwerveModule.create(
          "BRModule",
          backRightDriveMotor,
          backRightTurnMotor,
          backRightEncoder,
          driveMotorController(),
          driveFeedforward,
          Translation2d(- wheelbase / 2, - trackwidth / 2)
        )
      )
      return SwerveDrive(
        modules,
        ahrs,
        maxLinearSpeed,
        maxRotSpeed
      )
    }

    /**
     * @return the sim version of this drive
     */
    fun simOf(swerve: SwerveDrive): SwerveSim {
      return SwerveSim(swerve.modules, swerve.ahrs, swerve.maxLinearSpeed, swerve.maxRotSpeed)
    }
  }
}
