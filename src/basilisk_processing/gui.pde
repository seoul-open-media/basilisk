LazyGui gui;

void runGui() {
  for (int i = 1; i <= 4; i++) {
    basilisk.setElectromagnet(i, gui.toggle("electromagnets/em" + i) );
  }
  
  basilisk.setPhiL(gui.slider("phiL", -0.25, -1.25, 0.75));
}
