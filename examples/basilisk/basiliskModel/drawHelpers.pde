void ellipse(PVector center, float radius) {
  ellipse(center.x, center.y, 2 * radius, 2 * radius);
}

void line(PVector p1, PVector p2) {
  line(p1.x, p1.y, p2.x, p2.y);
}

void arrow(PVector from, PVector to, float more, String txt, color arrowStrokeWeight, float textSize) {
  PVector dir = PVector.sub(to, from).normalize();
  PVector moreVec = PVector.mult(dir, more);
  PVector dest = PVector.add(to, moreVec);
  stroke(0);
  strokeWeight(arrowStrokeWeight);
  line(from, dest);
  dir.setMag(10).rotate(0.875 * PI);  // Left arrowhead.
  line(dest, PVector.add(dest, dir));
  dir.rotate(0.25 * PI);  // Right arrowhead.
  line(dest, PVector.add(dest, dir));
  dir.rotate(0.875 * PI);  // Restore.

  pushMatrix();
  translate(dest.x, dest.y);
  float heading = dir.heading();
  float margin = 5;
  if (heading < HALF_PI || heading > 1.5 * PI) {
    rotate(heading);
    textAlign(LEFT);
  } else {
    rotate(heading + PI);
    textAlign(RIGHT);
    margin *= -1;
  }
  scale(1, -1);
  fill(0);
  textSize(textSize);
  text(txt, margin, 0);
  popMatrix();
}

void spiral(PVector center, float from, float to, String txt, float strokeWeight, float textSize) {
  strokeWeight(strokeWeight);
  colorMode(HSB);

  float angleStep = 0.01 * (from <= to ? 1 : -1);
  float radiusStep = 0.005 * (from <= to ? 1 : -1);
  float x = 0, y = 0;

  for (float angle = from, radius = 20; from <= to ? angle <= to : angle >= to; angle += angleStep, radius += radiusStep) {
    x = center.x + cos(revToRad(angle)) * radius;
    y = center.y + sin(revToRad(angle)) * radius;
    stroke(map(angle, from, to, 0, 255), 255, 255);
    point(x, y);
  }

  PVector leftArrowhead = PVector.fromAngle(revToRad(to) + 0.25 * PI).setMag(6);
  line(x, y, x + leftArrowhead.x, y + leftArrowhead.y);
  PVector rightArrowhead = PVector.fromAngle(revToRad(to) + 0.75 * PI).setMag(6);
  line(x, y, x + rightArrowhead.x, y + rightArrowhead.y);

  float txtHeading = ((from + to) * 0.5) % 1.0;
  x = center.x + cos(revToRad(txtHeading)) * 20;
  y = center.y + sin(revToRad(txtHeading)) * 20;
  float margin = 5;

  pushMatrix();
  translate(x, y);
  if (txtHeading < 0.25 || txtHeading > 0.75) {
    rotate(revToRad(txtHeading));
    textAlign(LEFT);
  } else {
    rotate(revToRad(txtHeading) + PI);
    textAlign(RIGHT);
    margin *= -1;
  }
  scale(1, -1);
  fill(0);
  textSize(textSize);
  text(txt, margin, 0);
  popMatrix();
}
