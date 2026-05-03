// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

/** Add your docs here. */
public class SwerveConstants {


//-------------------------------------------------------------
// GEOMETRÍA Y CONVERSIONES
// -------------------------------------------------------------

    /** Diámetro de la rueda en metros. */
    public static final double WHEEL_DIAMETER = 0.108;

    /**
     * Relación de transmisión del motor de tracción (drive).
     * <p>
     * Vueltas de motor por cada vuelta de rueda.
     */
    public static final double PWR_RATIO = 6.03;

    /**
     * Relación de transmisión del motor de giro (steer).
     * <p>
     * Vueltas de motor por cada vuelta completa del módulo.
     */
    public static final double STR_RATIO = 26.09;

    /**
     * Factor de conversión de rotaciones de motor de tracción a metros
     * recorridos por el módulo.
     *
     * rotaciones_motor * ROT_2_M = metros
     */
    public static final double ROT_2_M =
            (Math.PI * WHEEL_DIAMETER) / PWR_RATIO;

    /**
     * Factor de conversión de rotaciones del motor de giro a radianes de ángulo
     * del módulo.
     *
     * rotaciones_motor * ROT_2_RAD = radianes
     */
    public static final double ROT_2_RAD =
            (2.0 * Math.PI) / STR_RATIO;


    // --------------------------------------------------------------------
    // GANANCIAS DE CONTROL - VELOCIDAD (DRIVE)
    // --------------------------------------------------------------------

    /** kS: salida para vencer fricción estática en el drive. */
    public static final double VEL_KS = 0.16;

    /** kV: salida por unidad de velocidad objetivo (output / rps). */
    public static final double VEL_KV = 0.12;

    /** kA: salida por unidad de aceleración objetivo (output / (rps/s)). */
    public static final double VEL_KA = 0.003;

    /** kP: salida por unidad de error de velocidad (output / rps). */
    public static final double VEL_KP = 0.1;

    /** kI: salida por unidad de error integrado de velocidad. */
    public static final double VEL_KI = 0;

    /** kD: salida por unidad de derivada del error de velocidad. */
    public static final double VEL_KD = 0.025;

    // --------------------------------------------------------------------
    // GANANCIAS DE CONTROL - POSICIÓN (STEER)
    // --------------------------------------------------------------------

    /**
     * kG: salida para compensar gravedad (en este caso, torque/rozamiento
     * del módulo).
     */
    public static final double POS_KG = 0;

    /** kS: salida para vencer fricción estática (offset inicial). */
    public static final double POS_KS = 0.25;

    /** kV: salida por unidad de velocidad objetivo (output / rps). */
    public static final double POS_KV = 0.12;

    /** kA: salida por unidad de aceleración objetivo (output / (rps/s)). */
    public static final double POS_KA = 0.01;

    /** kP: salida por unidad de error de posición (output / rotación). */
    public static final double POS_KP = 4.8;

    /** kI: salida por unidad de error integrado de posición. */
    public static final double POS_KI = 0.0;

    /** kD: salida por unidad de error de velocidad (derivada). */
    public static final double POS_KD = 0.2;

    
    // --------------------------------------------------------------------
    // LIMITES DE ACELERACIÓN / ESTABILIDAD
    // --------------------------------------------------------------------

    /** Aceleración máxima hacia adelante (m/s²) usada en el limitador. */
    public static final double MAX_FORDWARD_ACCEL = 11.9;

    /** Aceleración máxima frontal (m/s²) en el modelo de estabilidad. */
    public static final double MAX_FRONT_ACCEL = 11.9;

    /** Aceleración máxima lateral (m/s²) en el modelo de estabilidad. */
    public static final double MAX_SIDE_ACCEL = 11.9;

    /**
     * Coeficiente de fricción efectivo rueda-suelo.
     * <p>
     * Se usa solo para derivar la aceleración lateral máxima por skid.
     */
    private static final double FRICTION_COF = 2.255;

    /**
     * Aceleración máxima antes de patinar (skid) en m/s².
     * <p>
     * Aproximada como μ * g.
     */
    public static final double MAX_SKID_ACCEL = FRICTION_COF * 9.81;

    /**
     * Zona muerta de velocidad del módulo (m/s).
     * <p>
     * Si la velocidad deseada está dentro de este rango alrededor de 0,
     * se fuerza a 0 para evitar vibraciones.
     */
    public static final double VELOCITY_DEADZONE = 0.02;

    /** Factor de asistencia al strafe en el modo asistido. */
    public static final double ASSIST_STRAFE_FACTOR = 0.4;

    
    // --------------------------------------------------------------------
    // MOTION MAGIC EXPO - STEER (POSICIÓN)
    // --------------------------------------------------------------------

    /** Velocidad de crucero de Motion Magic para el steer (rot/s). */
    public static final double MAGIC_MOTION_VELOCITY_STR = 100;

    /** Aceleración de Motion Magic para el steer (rot/s²). */
    public static final double MAGIC_MOTION_ACCELERATION_STR = 1000;

    /** Jerk de Motion Magic para el steer (rot/s³). */
    public static final double MAGIC_MOTION_JERK_STR = 0;

    /**
     * Ganancia kV del modo Motion Magic Expo para el steer.
     * <p>
     * Escala la contribución de la velocidad en el perfil de movimiento.
     */
    public static final double MAGIC_MOTION_EXPO_KV_STR = 0.12;

    /**
     * Ganancia kA del modo Motion Magic Expo para el steer.
     * <p>
     * Escala la contribución de la aceleración en el perfil de movimiento.
     */
    public static final double MAGIC_MOTION_EXPO_KA_STR = 0.10;
}
