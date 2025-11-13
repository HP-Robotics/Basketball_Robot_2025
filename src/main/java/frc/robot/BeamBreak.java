package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
    private DigitalInput m_beam;

    public BeamBreak(int channelId) {
        m_beam = new DigitalInput(channelId);
    }

    public Boolean beamBroken() {
        return (!m_beam.get());
    }
}