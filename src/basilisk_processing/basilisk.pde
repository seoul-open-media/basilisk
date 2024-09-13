class Basilisk {
  // (pRA - pLA) / b == e^{i * theta}
  //                 == e^{i * (sigL + phiL)}
  //                 == e^{i * (sigR + phiR)}
  // (pLT - pLA) / f == e^{i * sigL}
  // (pRT - pRA) / f == e^{i * sigR}
  private PVector pLA, pLT, pRA, pRT;  // Positions of left/right ankle/toe.
  public static final float bar = 100, foot = 40;    // Length of the bar and foot.
  private float sigL, phiL, theta, sigR, phiR;  // Angles in revolutions.
  private boolean magLA, magLT, magRA, magRT;   // Electromagnet on/off.

  public Basilisk(PVector _pLA, float _sigL, float _theta, float _sigR) {
    pLA = _pLA;
    sigL = _sigL;
    theta = _theta;
    sigR = _sigR;

    pLT = PVector.add(pLA, PVector.fromAngle(revToRad(sigL)).mult(foot));
    pRA = PVector.add(pLA, PVector.fromAngle(revToRad(theta)).mult(bar));
    pRT = PVector.add(pRA, PVector.fromAngle(revToRad(sigR)).mult(foot));

    phiL = theta - sigL;
    phiR = theta - sigR;
  }

  public void setPhiL(float val) {
    phiL = val;

    if (!magLA && magRA) {  // Left foot fixed, right foot free.
      sigR = sigL + phiL - phiR;
      theta = sigL + phiL;
      pLT = PVector.add(pLA, PVector.fromAngle(revToRad(sigL)).mult(foot));
      pRA = PVector.add(pLA, PVector.fromAngle(revToRad(theta)).mult(bar));
      pRT = PVector.add(pRA, PVector.fromAngle(revToRad(sigR)).mult(foot));
    }
  }

  public void setElectromagnet(int idx, boolean on) {
    switch (idx) {
    case 1:
      magLA = on;
      break;
    case 2:
      magLT = on;
      break;
    case 3:
      magRA = on;
      break;
    case 4:
      magRT = on;
      break;
    }
  }

  void run() {
  }

  void display() {
    noStroke();
    fill(magLA ? redBright : redDark);
    ellipse(pLA, 10);
    fill(magLT ? redBright : redDark);
    ellipse(pLT, 6);
    fill(50, 220, 50);
    fill(magRA ? greenBright : greenDark);
    ellipse(pRA, 10);
    fill(magRT ? greenBright : greenDark);
    ellipse(pRT, 6);

    arrow(pLA, pLT, 0, "sigL=" + sigL, 2, 15);
    arrow(pRA, pRT, 0, "sigR=" + sigR, 2, 15);
    arrow(pLA, pRA, 30, "theta=" + theta, 4, 15);

    spiral(pLA, sigL, theta, "phiL=" + phiL, 2, 15);
    spiral(pRA, sigR, theta, "phiR=" + phiR, 2, 15);
  }
}

Basilisk basilisk;
