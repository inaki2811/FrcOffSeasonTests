// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** 
 * Gestiona el Hardware de un solo módulo swerve:
 * <ul>
 *  <li> Motor de tracción (drive)  </li>
 *  <li> Motor de giro (turning)  </li>
 *  <li> Encoder relativo de tracción (driveEncoder)  </li>
 *  <li> Encoder absoluto de giro (turningEncoder)  </li>
 * </ul>
 * 
 * Esta clase se encarga de:
 * <ul>
 *  <li> Inicializar los dispostivos(CAN)  </li>
 *  <li> Resetear el encoder de tracción   </li>
 *  <li> Proveer los getters de motores  </li>
 *  <li> Proveer los getters de encoders  </li>
 *  <li> Proveer el getters del OffSet del encoder absoluto  </li>
 * </ul>
 *  
 * */
public class SwerveIO {

    /** Motor encargado de la tracción del Swerve */
    private final SparkMax driveMotor;

    /** Motor encargado del giro del Swerve */
    private final SparkMax turningMotor;


    /** Encoder relativo del motor de tracción */
    private final RelativeEncoder driveEncoder;

    /** Encoder absoluto del motor de giro */
    private final AbsoluteEncoder turningAbsoluteEncoder;

    /** OffSet del encoder absoluto en Radianes
     *  <p>
     *  Con esto alineas mecánicamente el 0 del módulo con el 0 del software
     *  Si no se tiene medido aún, puedes dejarlo en 0 hasta que se mida
     */
    private final double absoluteEncoderOffSetRAD;

    /**
     * 
     * Crea un nuevo SwerveIo para cada módulo swerve
     * 
     * @param driveSparkID
     * @param turningSparkID
     * @param absoluteEncoderOffSetRAD
     */
    public SwerveIO(
            int driveSparkID,
            int turningSparkID,
            double absoluteEncoderOffSetRAD
    ) {
        this.driveMotor = new SparkMax(driveSparkID, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();
        this.turningMotor = new SparkMax(turningSparkID, MotorType.kBrushless);
        this.turningAbsoluteEncoder = turningMotor.getAbsoluteEncoder();
        this.absoluteEncoderOffSetRAD = absoluteEncoderOffSetRAD;

        // Resetea motor de tracción
        resetDriveEncoder();
    }

    //** Reinicia la posición del motor de tracción a 0 metros*/
    public void resetDriveEncoder(){
        driveEncoder.setPosition(0.0);
    }
    
    // --- Getters de Encoders para Odometría ---



    /** @return La posición recorrida por la rueda en metros */
    public double getDrivePositionMeters(){
        return driveEncoder.getPosition();
    }

    /** @return La velocidad actual de la rueda en metros por segundo */
    public double getDriveVelocityPerSecond() {
        return driveEncoder.getVelocity();
    }

    /** @return El ángulo de la rueda en radianes */
    public double getTurningAbsoluteEncoderRadians(){

        return turningAbsoluteEncoder.getPosition();

    }

    // --- Getters de Hardware ---

    public SparkMax getDriveMotor() {
        return driveMotor;
    }

    public SparkMax getTurningMotor() {
        return turningMotor;
    }

    /** @return El offset físico del módulo */
    public double getAbsoluteEncoderOffSetRad(){
        return absoluteEncoderOffSetRAD;
    }
}


