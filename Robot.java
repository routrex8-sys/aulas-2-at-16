
package frc.robot;

import java.io.File;
import java.io.IOException;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Robot extends TimedRobot {

    // Controle
    private final XboxController controle =
        new XboxController(0);

    // Hardware
    private final DigitalOutput led =
        new DigitalOutput(6);

    private final Mecanismos mecanismos =
        new Mecanismos();

    private final MotorKraken kraken =
        new MotorKraken(6);

    private final Sensores sensores =
        new Sensores();

    private final Simulacao simulacao =
        new Simulacao();

    private final Logs logs =
        new Logs();

    // Inversao do controle
    private boolean invertido = false;

    // Controle do modo
    private String modoAtual = "";

    // Swerve
    private static final double VEL_MAX =
        Units.feetToMeters(14.5);

    private SwerveDrive swerve;

    private final StructPublisher<Pose2d> posePub =
        NetworkTableInstance.getDefault()
            .getStructTopic("Swerve/Pose", Pose2d.struct)
            .publish();

    @Override
    public void robotInit() {

        Dashboard.init();

        led.setPWMRate(500);
        led.enablePWM(0.0);

        SmartDashboard.putNumber("Kraken/Setpoint", 0);
        SmartDashboard.putNumber("Kraken/Modo", 0);

        SmartDashboard.putNumber("Controle/Velocidade", 0);

        // Inicializa YAGSL
        SwerveDriveTelemetry.verbosity =
            TelemetryVerbosity.HIGH;

        try {
            swerve = new SwerveParser(
                new File(
                    Filesystem.getDeployDirectory(),
                    "swerve"
                )
            ).createSwerveDrive(VEL_MAX);

        } catch (IOException e) {
            throw new RuntimeException(
                "Erro ao carregar configuracao YAGSL", e);
        }

        SmartDashboard.putBoolean(
            "Sistema/Inicializado", true);
    }

    @Override
    public void robotPeriodic() {

        String modo = Dashboard.modo.getSelected();

        if (modo == null) {
            modo = "CONTROLE";
        }

        // Monitoramento dos motores
        mecanismos.periodic();

        // Sensores do modo selecionado
        sensores.periodic(modo);

        // Simulacao virtual
        simulacao.periodic();

        // Dashboard LED
        boolean ligado = Dashboard.botao("LED/Ligar");

        double brilho = ligado
            ? Dashboard.clamp(
                Dashboard.numero("LED/Brilho", 0.5), 0, 1)
            : 0;

        Dashboard.status(
            ligado, brilho,
            SmartDashboard.getString("Mensagem", ""));

        // Swerve
        if (swerve != null) {
            swerve.updateOdometry();
            posePub.set(swerve.getPose());
        }

        // Informacoes do sistema
        SmartDashboard.putString(
            "Sistema/Modo atual", modo);

        SmartDashboard.putBoolean(
            "Sistema/Invertido", invertido);
    }

    private void pararTudo() {

        mecanismos.pararTudo();
        kraken.parar();

        controle.setRumble(RumbleType.kBothRumble, 0);

        if (swerve != null) {
            swerve.lockPose();
        }
    }

    @Override
    public void teleopInit() {

        modoAtual = "";
    }

    @Override
    public void teleopPeriodic() {

        String modo = Dashboard.modo.getSelected();

        if (modo == null) {
            modo = "CONTROLE";
        }

        // Ao trocar de modo, interrompe os comandos anteriores
        if (!modo.equals(modoAtual)) {
            pararTudo();
            modoAtual = modo;
        }

        switch (modo) {

            case "DIGITAL" -> {

                boolean pressionado =
                    sensores.botaoPressionado();

                led.updateDutyCycle(
                    pressionado ? 1.0 : 0.0);
            }

            case "SPARK_CIM" -> {

                led.updateDutyCycle(0);

                mecanismos.controlarCIM();
            }

            case "SPARK_NEO" -> {

                mecanismos.controlarNEO();
            }

            case "MAXMOTION" -> {

                mecanismos.controlarMAXMotion();
            }

            case "RPM" -> {

                mecanismos.controlarRPM();
            }

            case "KRAKEN" -> {

                double sp =
                    Dashboard.numero("Kraken/Setpoint", 0);

                int tipo = (int) Dashboard.numero(
                    "Kraken/Modo", 0);

                switch (tipo) {
                    case 0 -> kraken.setVelocidade(sp);
                    case 1 -> kraken.setPosicao(sp);
                    case 2 -> kraken.setMotionMagic(sp);
                    case 3 -> kraken.setRPM(sp);
                    default -> kraken.parar();
                }
            }

            case "SENSORES" -> {

                // Leituras realizadas em robotPeriodic()
            }

            case "ENCODERS" -> {

                // Leituras realizadas em robotPeriodic()
            }

            case "PIGEON" -> {

                // Leituras realizadas em robotPeriodic()
            }

            case "CONTROLE" -> {

                controlarXbox();
            }

            case "SIMULACAO" -> {

                simulacao.controlar();
            }

            case "SWERVE" -> {

                controlarSwerve();
            }

            default -> pararTudo();
        }

        // Telemetria Kraken
        SmartDashboard.putNumber(
            "Kraken/Posicao", kraken.getPosicao());

        SmartDashboard.putNumber(
            "Kraken/RPM", kraken.getRPM());

        SmartDashboard.putNumber(
            "Kraken/Corrente", kraken.getCorrente());
    }

    private void controlarXbox() {

        double eixo = -MathUtil.applyDeadband(
            controle.getLeftY(), 0.1);

        double saida;

        if (controle.getAButton()) {
            saida = 0.5;
        } else if (controle.getBButton()) {
            saida = -0.5;
        } else {
            saida = eixo *
                (controle.getRightBumperButton() ? 1.0 : 0.5);
        }

        saida = MathUtil.clamp(
            saida + controle.getRightTriggerAxis() * 0.5,
            -1, 1);

        if (controle.getXButtonPressed()) {
            invertido = !invertido;
        }

        if (invertido) {
            saida = -saida;
        }

        mecanismos.controlarXbox(saida);

        double corrente =
            mecanismos.getCorrenteControle();

        controle.setRumble(
            RumbleType.kBothRumble,
            corrente > 30 ? 0.5 : 0);

        logs.registrar(
            eixo,
            saida,
            mecanismos.getRPMControle(),
            corrente,
            invertido
        );

        SmartDashboard.putNumber(
            "Controle/Eixo Y", eixo);

        SmartDashboard.putNumber(
            "Controle/Saida", saida);
    }

    private void controlarSwerve() {

        if (swerve == null) {
            return;
        }

        double x = -MathUtil.applyDeadband(
            controle.getLeftY(), 0.1) * VEL_MAX;

        double y = -MathUtil.applyDeadband(
            controle.getLeftX(), 0.1) * VEL_MAX;

        double rot = -MathUtil.applyDeadband(
            controle.getRightX(), 0.1)
            * swerve.getMaximumChassisAngularVelocity();

        swerve.drive(
            new Translation2d(x, y),
            rot,
            true,
            false
        );

        if (controle.getStartButtonPressed()) {
            swerve.zeroGyro();
        }
    }

    @Override
    public void disabledInit() {

        pararTudo();

        led.updateDutyCycle(0);
    }

    @Override
    public void disabledPeriodic() {

        controle.setRumble(RumbleType.kBothRumble, 0);
    }
}
