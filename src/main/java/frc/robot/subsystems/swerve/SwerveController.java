// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Velocity;

/**
 * Encapsula la configuración y el control de un módulo swerve:
 * <ul>
 *   <li>Motor de tracción operando con su control de velocidad</li>
 *   <li>Motor de giro operando en modo MaxMotion con su posición suavizada</li>
 * </ul>
 *
 * Esta clase:
 * <ul>
 *   <li>Aplica las ganancias PID y factores de conversión desde {@link SwerveConstants}</li>
 *   <li>Provee métodos seguros para establecer velocidad lineal y ángulo objetivo</li>
 * </ul>
 */
public class SwerveController {

    // --- Motores físicos ---

    /** Motor de tracción del módulo */
    private final SparkMax driveMotor;

    /** Motor de giro del módulo */
    private final SparkMax turningMotor;


    // --- Controladores PID ---

    /** Controlador del motor de tracción */
    private final SparkClosedLoopController drivePID;

    /** Controlador del motor de giro */
    private final SparkClosedLoopController turningPID;

    /** 
    * Inicializa los controladores que estan vinculados al hardware físico de los módulos definifos en SwerveIO
    * @param io El contenedor de hardware (motores y sensores) del móduo
    */
    public SwerveController(SwerveIO io){
        //Se extraen los motores del IO
        this.driveMotor = io.getDriveMotor();
        this.turningMotor = io.getTurningMotor();

        //Se extraen los controladores PID internos de la memoria del SparkMax
        this.drivePID = driveMotor.getClosedLoopController();
        this.turningPID = turningMotor.getClosedLoopController();
        
        // Aplicamos las configuraciones a la memoria 
        configureDriveMotors();
        configureTurningMotors();
        resetEncoders();
    }

    /**
     * Configura los parámetros del motor de tracción
     * Establece los límites de corriente, factores de conversión a metros y el PID de velocidad
     */
    private void configureDriveMotors() {
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        /**  Comportamiento físico y protección eléctrica */
        driveConfig.smartCurrentLimit(30);  // Establece el limite de corriente

        /** Conversión de unidades: 
         *  De: Unidades internas del motor
         *  A: Metros 
        */

        driveConfig.encoder.positionConversionFactor(SwerveConstants.ROT_2_M);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.ROT_2_M / 60);

        /** Guarda la configuración en la memoria del Spark y se protege reinicios bruscos */
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    


    /**
     * Configura los parámetros del motor de giro
     * Establece el uso de Wrapping y el perfil de movimiento MAXMotion
     */
    private void configureTurningMotors() {
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        
        /**  Comportamiento físico y protección eléctrica */
        turningConfig.idleMode(IdleMode.kBrake);
        turningConfig.smartCurrentLimit(30);    // Establece el limite de corriente

        // Configuración del encoder
                
        turningConfig.encoder.positionConversionFactor(SwerveConstants.ROT_2_RAD);  // Salida en radianes
        turningConfig.encoder.velocityConversionFactor(SwerveConstants.ROT_2_RAD / 60);


        turningConfig.closedLoop.pid(SwerveConstants.POS_KP, SwerveConstants.POS_KI, SwerveConstants.POS_KD);
        

        /** Le indica al PID leer los datos del encoder */
        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);   

        /** Position Wrapping: Optimiza y hace al módulo swerve ir por el camino más corto en un círculo (ej. de 359° a 1°) */
        turningConfig.closedLoop.positionWrappingEnabled(true);
        turningConfig.closedLoop.positionWrappingInputRange(-Math.PI,  Math.PI);

        /** Guarda la configuración en la memoria del Spark y se protege reinicios bruscos */
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Ordena el motor de tracción moverse a una velocidad específica
     * @param velocityMps Velocidad deseada en Metros por Segundo
     */
    public void setVelocity (double velocityMps) {
        //  Deadband de velocidad: A partir de cierto umbral ignora comandos
        if (Math.abs(velocityMps) > 0.1) {

            drivePID.setSetpoint((velocityMps / SwerveConstants.MAX_SPEED_MPS), ControlType.kDutyCycle ,ClosedLoopSlot.kSlot0, 0.0);
            
        } else {

            driveMotor.stopMotor();

        }
    }

    /**
     * Ordena al motor de rotación establecerse en cierto ángulo ecpecífico
     * @param angleRad Ángulo deseado en Radianes (Rango de 0 a 2π)
     */
    public void setAngle ( double angleRad) {
        
        // Lee posición actual del encoder 
        double currentAngleRad = turningMotor.getEncoder().getPosition();  


            //  Usa MAXMotion para un movimiento fluido y rápido
            turningPID.setSetpoint(angleRad, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);    

        
    }


    public void resetEncoders(){
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(0);
    }
}
