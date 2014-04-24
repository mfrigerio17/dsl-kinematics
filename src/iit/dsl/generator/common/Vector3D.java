package iit.dsl.generator.common;

import iit.dsl.kinDsl.Vector3;

public class Vector3D {
    public double x;
    public double y;
    public double z;

    Vector3D (double xx, double yy, double zz) {
        x = xx;
        y = yy;
        z = zz;
    }

    public double norm() {
        return java.lang.Math.sqrt(x*x + y*y + z*z);
    }


    public static Vector3D convert(Vector3 vec) {
        double x = helper.asFloat(vec.getX());
        double y = helper.asFloat(vec.getY());
        double z = helper.asFloat(vec.getZ());
        return new Vector3D(x, y, z);
    }

    public static double norm(Vector3 vec) {
        return Vector3D.convert(vec).norm();
    }

    private static iit.dsl.generator.Common helper = new iit.dsl.generator.Common();
}
