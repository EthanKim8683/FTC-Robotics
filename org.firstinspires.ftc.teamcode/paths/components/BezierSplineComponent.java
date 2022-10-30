package org.firstinspires.ftc.teamcode;

public class BezierSplineComponent implements Component {
  private Vec[] points;
  private int dim;
  private int count;

  public BezierSplineComponent(Vec[] points) throws Vec.DimMismatchException {
    int dim = points[0].getDim();
    for (int i = 1; i < points.length; i++) if (points[i].getDim() != dim) throw new Vec.DimMismatchException();
    this.points = BezierSplineComponent.getControlPoints(points);
    this.dim = dim;
    this.count = points.length - 1;
  }

  private class BezierCurve {
    public static Vec getPosition(Vec v0, Vec v1, Vec v2, Vec v3, double t) throws Vec.DimMismatchException {
      double s = 1.0 - t;
      double a = s * s * s, b = 3.0 * s * s * t, c = 3.0 * s * t * t, d = t * t * t;
      return Vec.add(Vec.add(Vec.add(Vec.mul(a, v0), Vec.mul(b, v1)), Vec.mul(c, v2)), Vec.mul(d, v3));
    }

    public static Vec getTangent(Vec v0, Vec v1, Vec v2, Vec v3, double t) throws Vec.DimMismatchException {
      double s = 1.0 - t;
      double a = -3.0 * s * s, b = 3.0 * (t - 1.0) * (3.0 * t - 1.0), c = 3.0 * t * (3.0 * t - 2.0), d = 3.0 * t * t;
      return Vec.normalize(Vec.add(Vec.sub(Vec.add(Vec.mul(a, v0), Vec.mul(b, v1)), Vec.mul(c, v2)), Vec.mul(d, v3)));
    }
  }

  private static Vec[] getControlPoints(Vec[] points) throws Vec.DimMismatchException {
    int dim = points[0].getDim();
    int n = points.length - 1;
    Vec[] res = new Vec[4 * n];
    Vec[] q1 = new Vec[n], q2 = new Vec[n];
    Vec[] a = new Vec[n], b = new Vec[n], c = new Vec[n], d = new Vec[n];
    a[0] = new Vec(dim, 0.0);
    b[0] = new Vec(dim, 2.0);
    c[0] = new Vec(dim, 1.0);
    d[0] = Vec.add(points[0], Vec.mul(2.0, points[1]));
    for (int i = 1; i + 1 < n; i++) {
      a[i] = new Vec(dim, 1.0);
      b[i] = new Vec(dim, 4.0);
      c[i] = new Vec(dim, 1.0);
      d[i] = Vec.add(Vec.mul(4.0, points[i]), Vec.mul(2.0, points[i + 1]));
    }
    a[n - 1] = new Vec(dim, 2.0);
    b[n - 1] = new Vec(dim, 7.0);
    c[n - 1] = new Vec(dim, 0.0);
    d[n - 1] = Vec.add(Vec.mul(8.0, points[n - 1]), points[n]);
    for (int i = 1; i < n; i++) {
      Vec m = Vec.div(a[i], b[i - 1]);
      b[i] = Vec.sub(b[i], Vec.mul(m, c[i - 1]));
      d[i] = Vec.sub(d[i], Vec.mul(m, d[i - 1]));
    }
    q1[n - 1] = Vec.div(d[n - 1], b[n - 1]);
    q2[n - 1] = Vec.mul(0.5, Vec.add(points[n], q1[n - 1]));
    for (int i = n - 2; i >= 0; i--) q1[i] = Vec.div(Vec.sub(d[i], Vec.mul(c[i], q1[i + 1])), b[i]);
    for (int i = 0; i < n - 1; i++) q2[i] = Vec.sub(Vec.mul(2.0, points[i + 1]), q1[i + 1]);
    for (int i = 0; i < n; i++) {
      res[i * 4 + 0] = points[i];
      res[i * 4 + 1] = q1[i];
      res[i * 4 + 2] = q2[i];
      res[i * 4 + 3] = points[i + 1];
    }
    return res;
  }

  public int getDim() {
    return this.dim;
  }

  public int getCount() {
    return this.count;
  }

  public void getSegment(int i, Vec v0, Vec v1, Vec v2, Vec v3) throws Vec.DimMismatchException {
    v0.setDims(this.points[i * 4 + 0]);
    v1.setDims(this.points[i * 4 + 1]);
    v2.setDims(this.points[i * 4 + 2]);
    v3.setDims(this.points[i * 4 + 3]);
  }

  public Vec getPosition(double t) throws Vec.DimMismatchException {
    double s = t * this.count;
    int i = (int) t;
    double f = t % 1;
    Vec v0 = new Vec(), v1 = new Vec(), v2 = new Vec(), v3 = new Vec();
    this.getSegment(i, v0, v1, v2, v3);
    return BezierCurve.getPosition(v0, v1, v2, v3, f);
  }

  public Vec getTangent(double t) throws Vec.DimMismatchException {
    double s = t * this.count;
    int i = (int) t;
    double f = t % 1;
    Vec v0 = new Vec(), v1 = new Vec(), v2 = new Vec(), v3 = new Vec();
    this.getSegment(i, v0, v1, v2, v3);
    return BezierCurve.getTangent(v0, v1, v2, v3, f);
  }

  public double getLength(int l, int r) throws Vec.DimMismatchException {
    double length = 0.0;
    for (int i = l; i < r; i++) {
      Vec v0 = new Vec(), v1 = new Vec(), v2 = new Vec(), v3 = new Vec();
      this.getSegment(i, v0, v1, v2, v3);
      length += Vec.distance(v1, v2); // approx.
    }
    return length;
  }

  public double getLength() throws Vec.DimMismatchException {
    return this.getLength(0, this.count);
  }
}