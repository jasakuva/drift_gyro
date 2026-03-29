class FastCurve10 {
public:
  static const uint8_t NUM_POINTS = 10;

  FastCurve10(float xMin = 0.0f, float xMax = 1.0f)
    : xMin(xMin), xMax(xMax) {
    step = (xMax - xMin) / (NUM_POINTS - 1);
    for (uint8_t i = 0; i < NUM_POINTS; i++) {
      yPoints[i] = 0.0f;
    }
  }

  void setY(uint8_t index, float y) {
    if (index >= NUM_POINTS) return;
    yPoints[index] = y;
  }

  float getY(float x) const {
    if (x <= xMin) return yPoints[0];
    if (x >= xMax) return yPoints[NUM_POINTS - 1];

    float pos = (x - xMin) / step;
    uint8_t i = (uint8_t)pos;

    if (i >= NUM_POINTS - 1) return yPoints[NUM_POINTS - 1];

    float t = pos - i;
    return yPoints[i] + t * (yPoints[i + 1] - yPoints[i]);
  }

private:
  float yPoints[NUM_POINTS];
  float xMin;
  float xMax;
  float step;
};