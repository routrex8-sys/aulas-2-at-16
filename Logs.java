
package frc.robot;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Logs {

    private final DoubleLogEntry eixo;
    private final DoubleLogEntry saida;
    private final DoubleLogEntry rpm;
    private final DoubleLogEntry corrente;
    private final BooleanLogEntry invertido;

    public Logs() {

        DataLogManager.start();

        DriverStation.startDataLog(DataLogManager.getLog());

        DataLog log = DataLogManager.getLog();

        eixo = new DoubleLogEntry(log, "/Controle/EixoY");
        saida = new DoubleLogEntry(log, "/Motor/Saida");
        rpm = new DoubleLogEntry(log, "/Motor/RPM");
        corrente = new DoubleLogEntry(log, "/Motor/Corrente");

        invertido = new BooleanLogEntry(
            log, "/Motor/Invertido");
    }

    public void registrar(double e, double s,
                          double r, double c, boolean i) {

        eixo.append(e);
        saida.append(s);
        rpm.append(r);
        corrente.append(c);
        invertido.append(i);
    }
}
