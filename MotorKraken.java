
package frc.robot;

public class MotorKraken {

    private final TalonFX motor;

    private final DutyCycleOut velocidade = new DutyCycleOut(0);
    private final PositionVoltage posicao = new PositionVoltage(0).withSlot(0);
    private final MotionMagicVoltage motionMagic =
        new MotionMagicVoltage(0).withSlot(0);
    private final VelocityVoltage rpm =
        new VelocityVoltage(0).withSlot(1);

    public MotorKraken(int id) {
        motor = new TalonFX(id);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted =
            InvertedValue.CounterClockwise_Positive;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimit = 60;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.Slot0.kP = 2.4;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0.1;

        cfg.Slot1.kS = 0.1;
        cfg.Slot1.kV = 0.12;
        cfg.Slot1.kP = 0.11;

        cfg.MotionMagic.MotionMagicCruiseVelocity = 40;
        cfg.MotionMagic.MotionMagicAcceleration = 80;
        cfg.MotionMagic.MotionMagicJerk = 800;

        motor.getConfigurator().apply(cfg);
        motor.setPosition(0);
    }

    public void setVelocidade(double valor) {
        motor.setControl(velocidade.withOutput(
            Math.max(-1, Math.min(1, valor))));
    }

    public void setPosicao(double rotacoes) {
        motor.setControl(posicao.withPosition(rotacoes));
    }

    public void setMotionMagic(double rotacoes) {
        motor.setControl(motionMagic.withPosition(rotacoes));
    }

    public void setRPM(double valorRPM) {
        motor.setControl(rpm.withVelocity(valorRPM / 60.0));
    }

    public double getPosicao() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getRPM() {
        return motor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getCorrente() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void parar() {
        motor.stopMotor();
    }
}
