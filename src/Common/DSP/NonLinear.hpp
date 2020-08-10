#pragma once

inline float tanhDriveSignal(float x, float drive) {
    x *= drive;

    if(x < -1.3f) {
        return -1.f;
    }
    else if(x < -0.75f) {
        return (x * x + 2.6f * x + 1.69f) * 0.833333f - 1.f;
    }
    else if(x > 1.3f) {
        return 1.f;
    }
    else if(x > 0.75f) {
        return 1.f - (x * x - 2.6f * x + 1.69f) * 0.833333f;
    }
    return x;
}

inline float SoftLimit(float x) {
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

inline float SoftClip(float x) {
  if (x < -3.0f) {
    return -1.0f;
  } else if (x > 3.0f) {
    return 1.0f;
  } else {
    return SoftLimit(x);
  }
}