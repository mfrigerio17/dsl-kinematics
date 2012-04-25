package iit.dsl.generator;

import iit.dsl.kinDsl.InertiaParams;
import iit.dsl.kinDsl.KinDslFactory;
import iit.dsl.kinDsl.Vector3;
import iit.dsl.kinDsl.FloatLiteral;
import iit.dsl.kinDsl.impl.KinDslFactoryImpl;

public class Utilities {

	private static KinDslFactory factory    = KinDslFactoryImpl.init();

	/**
     * Computes the inertia parameters of a rigid body in a different frame.
     * The last arguments of these function specify the rotation and the translation
     * which encode the pose of the new frame, with respect to the frame
     * in which the inertia-parameters are currently expressed.
     * @param inertia the input inertia parameters to be expressed in the new frame
     * @param tx translation along the X axis
     * @param ty translation along the Y axis
     * @param tz translation along the Z axis
     * @param rx rotation about the X axis
     * @param ry rotation about the Y axis
     * @param rz rotation about the Z axis
     * The translation is expressed in the current frame. Rotation values are basically
     * euler angles, consecutive rotations about x, y, and z axis, in this order, of the
     * current frame; each rotation is about the axis rotated by the previous one.
     * @return a new instance of InertiaParams that specifies the same inertial
     *         properties of the first parameter, but expressed in a different frame
     */
    public static InertiaParams rototranslate(InertiaParams inertia,
            float tx, float ty, float tz, float rx, float ry, float rz)
    {
        float mass  = inertia.getMass();
        Vector3 com = inertia.getCom();
        if( ! (com.getX() instanceof FloatLiteral) ||
            ! (com.getY() instanceof FloatLiteral) ||
            ! (com.getZ() instanceof FloatLiteral))
        {
            throw new RuntimeException("Cannot translate inertia parameters if the COM is defined with some variables");
        }

        float comx = ((FloatLiteral)com.getX()).getValue();
        float comy = ((FloatLiteral)com.getY()).getValue();
        float comz = ((FloatLiteral)com.getZ()).getValue();

        float ixx = inertia.getIx();
        float iyy = inertia.getIy();
        float izz = inertia.getIz();
        float ixy = inertia.getIxy();
        float ixz = inertia.getIxz();
        float iyz = inertia.getIyz();

        InertiaParams translated = factory.createInertiaParams();
        // The mass obviously does not change with a change in the reference frame:
        translated.setMass(mass);

        float sx = (float)Math.sin(rx);
        float sy = (float)Math.sin(ry);
        float sz = (float)Math.sin(rz);
        float cx = (float)Math.cos(rx);
        float cy = (float)Math.cos(ry);
        float cz = (float)Math.cos(rz);

        float tmp = 0;
        // Compute the COM position in the new frame, applying the rotation
        //  matrix defined by rx,ry,rz ...
        Vector3 new_com = factory.createVector3();
        translated.setCom(new_com);
        FloatLiteral tmpLiteral;
        // X ...
        tmpLiteral = factory.createFloatLiteral();
        tmp = comz*(sx*sz - cx*sy*cz) + comy*(cx*sz + sx*sy*cz) + comx*cy*cz  - tx;
        tmpLiteral.setValue(tmp);
        new_com.setX(tmpLiteral);
        // Y ...
        tmpLiteral = factory.createFloatLiteral();
        tmp = comy*(cx*cz - sx*sy*sz) + comz*(cx*sy*sz + sx*cz) - comx*cy*sz  - ty;
        tmpLiteral.setValue(tmp);
        new_com.setY(tmpLiteral);
        // Z ...
        tmpLiteral = factory.createFloatLiteral();
        tmp = comx*sy - comy*sx*cy + comz*cx*cy  - tz;
        tmpLiteral.setValue(tmp);
        new_com.setZ(tmpLiteral);

        // Now computes the inertia tensor in the new frame

        // First use the parallel axis theorem for the translation ...
        ixx += mass * (ty*ty + tz*tz);
        iyy += mass * (tx*tx + tz*tz);
        izz += mass * (tx*tx + ty*ty);
        ixy += mass * (tx * ty);
        ixz += mass * (tx * tz);
        iyz += mass * (ty * tz);

        //... then consider the rotation of the axes.
        // The following is the matrix M that transform coordinates in the original frame
        //  into coordinates of the new frame:
        // [  cos(ry)*cos(rz)    cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz)     sin(rx)*sin(rz) - cos(rx)*sin(ry)*cos(rz) ]
        // [                                                                                                             ]
        // [ - cos(ry)*sin(rz)   cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz)     cos(rx)*sin(ry)*sin(rz) + sin(rx)*cos(rz) ]
        // [                                                                                                             ]
        // [      sin(ry)                    - sin(rx)*cos(ry)                              cos(rx)*cos(ry)              ]
        float[][] M = {
                { cy*cz,   cx*sz + sx*sy*cz,     sx*sz - cx*sy*cz },
                {-cy*sz,   cx*cz - sx*sy*sz,     cx*sy*sz + sx*cz },
                {  sy  ,     - sx*cy       ,     cx*cy }
        };
        // The equations to transform the inertia-moments are very similar to the
        // equation in matrix form for the inertia tensor
        //  I' =  M * I * M^T
        // but some signs are different! Remember that our convention
        // states that ixy, ixz and iyz are the centrifugal moments, and NOT the elements of
        // the inertia tensor; they differ only for the minus sign.

        // Ixx
        tmp =     M[0][0] * M[0][0] * ixx +
                  M[0][1] * M[0][1] * iyy +
                  M[0][2] * M[0][2] * izz +
             -2 * M[0][0] * M[0][1] * ixy +
             -2 * M[0][0] * M[0][2] * ixz +
             -2 * M[0][1] * M[0][2] * iyz;
        translated.setIx(tmp);
        // Iyy
        tmp =     M[1][0] * M[1][0] * ixx +
                  M[1][1] * M[1][1] * iyy +
                  M[1][2] * M[1][2] * izz +
             -2 * M[1][0] * M[1][1] * ixy +
             -2 * M[1][0] * M[1][2] * ixz +
             -2 * M[1][1] * M[1][2] * iyz;
        translated.setIy(tmp);
        // Izz
        tmp =     M[2][0] * M[2][0] * ixx +
                  M[2][1] * M[2][1] * iyy +
                  M[2][2] * M[2][2] * izz +
             -2 * M[2][0] * M[2][1] * ixy +
             -2 * M[2][0] * M[2][2] * ixz +
             -2 * M[2][1] * M[2][2] * iyz;
        translated.setIz(tmp);
        // Ixy
        tmp =- M[0][0] * M[1][0] * ixx +
             - M[0][1] * M[1][1] * iyy +
             - M[0][2] * M[1][2] * izz +
             (M[0][0]*M[1][1] + M[0][1]*M[1][0]) * ixy +
             (M[0][0]*M[1][2] + M[0][2]*M[1][0]) * ixz +
             (M[0][1]*M[1][2] + M[0][2]*M[1][1]) * iyz;
        translated.setIxy(tmp);
        // Ixz
        tmp =- M[0][0] * M[2][0] * ixx +
             - M[0][1] * M[2][1] * iyy +
             - M[0][2] * M[2][2] * izz +
            (M[0][0]*M[2][1] + M[0][1]*M[2][0]) * ixy +
            (M[0][0]*M[2][2] + M[0][2]*M[2][0]) * ixz +
            (M[0][1]*M[2][2] + M[0][2]*M[2][1]) * iyz;
        translated.setIxz(tmp);
        // Iyz
        tmp =- M[1][0] * M[2][0] * ixx +
             - M[1][1] * M[2][1] * iyy +
             - M[1][2] * M[2][2] * izz +
             (M[1][0]*M[2][1] + M[1][1]*M[2][0]) * ixy +
             (M[1][0]*M[2][2] + M[1][2]*M[2][0]) * ixz +
             (M[1][1]*M[2][2] + M[1][2]*M[2][1]) * iyz;
        translated.setIyz(tmp);

        return translated;
    }

	public static float invert(float num) {
	    return -num;
	}
	public static boolean isZero(float num) {
		return num==0.0;
	}
	public static float mult(float a, float b) {
	    return a*b;
	}
	public static float div(float a, float b) {
	    return a/b;
	}
}
