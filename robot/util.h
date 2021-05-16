#ifndef UTIL_H
#define UTIL_H
// Numeric Utilities
template <typename T> T clamp(T val, T min, T max) {
  if (val < min) { return min; }
  if (val > max) { return max; }
  return val;
}
template <typename T> T map(T val, T from_min, T from_max, T to_min, T to_max) {
  return (val - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}
template <typename T> T deadzone(T val, T mag) {
  if (val > -mag && val < mag) { return (T)0; }
  return val;
}
// 3D Math utilities
class Quaternion {
  public:
    float w;
    float x;
    float y;
    float z;

    Quaternion() {
      w = 1.0f;
      x = 0.0f;
      y = 0.0f;
      z = 0.0f;
    }
    Quaternion(float * n) {
      setQuaternion(n[0], n[1], n[2], n[3]);
    }
    Quaternion(long * n) {
      setQuaternion((float)n[0], (float)n[1], (float)n[2], (float)n[3]);
    }
    Quaternion setQuaternion(float * n) {
      setQuaternion(n[0], n[1], n[2], n[3]);
    }
    Quaternion setQuaternion(long * n) {
      setQuaternion((float)n[0], (float)n[1], (float)n[2], (float)n[3]);
    }
    Quaternion(float nw, float nx, float ny, float nz) {
      setQuaternion(nw, nx, ny, nz);
    }
    Quaternion setQuaternion(float nw, float nx, float ny, float nz) {
      w = nw;
      x = nx;
      y = ny;
      z = nz;
    }
    Quaternion getProduct(Quaternion q) {
      // Quaternion multiplication is defined by:
      //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
      //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
      //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
      //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
      return Quaternion(
          w*q.w - x*q.x - y*q.y - z*q.z,  // new w
          w*q.x + x*q.w + y*q.z - z*q.y,  // new x
          w*q.y - x*q.z + y*q.w + z*q.x,  // new y
          w*q.z + x*q.y - y*q.x + z*q.w); // new z
    }

    Quaternion getConjugate() {
      return Quaternion(w, -x, -y, -z);
    }

    float getMagnitude() {
      return sqrt(w*w + x*x + y*y + z*z);
    }

    Quaternion :: normalize() {
      float m = getMagnitude();
      w /= m;
      x /= m;
      y /= m;
      z /= m;
      return this;
    }

    Quaternion getNormalized() {
      Quaternion r(w, x, y, z);
      r.normalize();
      return r;
    }
};

class VectorInt16 {
  public:
    int16_t x;
    int16_t y;
    int16_t z;

    VectorInt16() {
      x = 0;
      y = 0;
      z = 0;
    }

    VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
      x = nx;
      y = ny;
      z = nz;
    }


    float getMagnitude() {
      return sqrt((float)x*(float)x + (float)y*(float)y + (float)z*(float)z);
    }

    VectorInt16 :: normalize() {
      float m = getMagnitude();
      x /= m;
      y /= m;
      z /= m;
      return this;
    }

    VectorInt16 getNormalized() {
      VectorInt16 r(x, y, z);
      r.normalize();
      return r;
    }

    VectorInt16 :: rotate(Quaternion *q) {
      // http://www.cprogramming.com/tutorial/3d/quaternions.html
      // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
      // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
      // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

      // P_out = q * P_in * conj(q)
      // - P_out is the output vector
      // - q is the orientation quaternion
      // - P_in is the input vector (a*aReal)
      // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
      Quaternion p(0, x, y, z);

      // quaternion multiplication: q * p, stored back in p
      p = q -> getProduct(p);

      // quaternion multiplication: p * conj(q), stored back in p
      p = p.getProduct(q -> getConjugate());

      // p quaternion is now [0, x', y', z']
      x = p.x;
      y = p.y;
      z = p.z;
      return this;
    }

    VectorInt16 getRotated(Quaternion *q) {
      VectorInt16 r(x, y, z);
      r.rotate(q);
      return r;
    }
};

class VectorFloat {
  public:
    float x;
    float y;
    float z;

    VectorFloat() {
      x = 0;
      y = 0;
      z = 0;
    }

    VectorFloat(float nx, float ny, float nz) {
      x = nx;
      y = ny;
      z = nz;
    }

    float getMagnitude() {
      return sqrt(x*x + y*y + z*z);
    }

    VectorFloat :: normalize() {
      float m = getMagnitude();
      x /= m;
      y /= m;
      z /= m;
      return this;
    }

    VectorFloat getNormalized() {
      VectorFloat r(x, y, z);
      r.normalize();
      return r;
    }

    VectorFloat :: rotate(Quaternion *q) {
      Quaternion p(0, x, y, z);

      // quaternion multiplication: q * p, stored back in p
      p = q -> getProduct(p);

      // quaternion multiplication: p * conj(q), stored back in p
      p = p.getProduct(q -> getConjugate());

      // p quaternion is now [0, x', y', z']
      x = p.x;
      y = p.y;
      z = p.z;
      return this;
    }

    VectorFloat getRotated(Quaternion *q) {
      VectorFloat r(x, y, z);
      r.rotate(q);
      return r;
    }
};
#endif
