class DerivativeN {
public:
  explicit DerivativeN(uint8_t n) {
    setWindow(n);
    reset();
  }

  void setWindow(uint8_t n) {
    if (n < 2) n = 2;
    if (n > MAX_N) n = MAX_N;
    N = n;
    reset();
  }

  uint8_t window() const { return N; }

  // Add one new sample (call at constant rate)
  void add(float value) {
    values[idx] = value;
    idx = (idx + 1) % N;

    if (!filled) {
      count++;
      if (count >= N) filled = true;
    }
  }

  // Derivative in units per sample
  float get() const {
    if (!filled) return 0.0f;

    // x-axis = sample index: 0,1,2,...,N-1
    // Oldest sample is at idx
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;

    for (uint8_t k = 0; k < N; k++) {
      uint8_t i = (idx + k) % N;  // chronological order
      float x = (float)k;
      float y = values[i];

      sum_x  += x;
      sum_y  += y;
      sum_xx += x * x;
      sum_xy += x * y;
    }

    float denom = (float)N * sum_xx - sum_x * sum_x;
    if (denom == 0.0f) return 0.0f;

    return ((float)N * sum_xy - sum_x * sum_y) / denom;
  }

  bool ready() const { return filled; }

  void reset() {
    idx = 0;
    count = 0;
    filled = false;
    for (uint8_t i = 0; i < MAX_N; i++) {
      values[i] = 0.0f;
    }
  }

private:
  static constexpr uint8_t MAX_N = 20; // adjust as needed
  float values[MAX_N];

  uint8_t N = 5;        // active window length
  uint8_t idx = 0;      // next write index
  uint8_t count = 0;
  bool filled = false;
};