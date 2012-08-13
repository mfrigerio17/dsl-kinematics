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
     * The frame in which the inertia-parameters (COM position and inertia tensor)
     * are currently expressed is C; the new frame in which such parameters
     * have to be expressed is N.
     * The arguments of this function specify the transformation between C and N.
     * If the last argument 'inverse' is false, then the rotation/translation
     * parameters encode the pose of N with respect to C. Otherwise they represent
     * the pose of C with respect to N.
     * The translation is expressed in the current frame. Rotation values are basically
     * euler angles, consecutive rotations about x, y, and z axis, in this order, of the
     * current frame; each rotation is about the axis rotated by the previous one.
     * @param inertia the input inertia parameters to be expressed in the new frame
     * @param tx translation along the X axis
     * @param ty translation along the Y axis
     * @param tz translation along the Z axis
     * @param rx rotation about the X axis
     * @param ry rotation about the Y axis
     * @param rz rotation about the Z axis
     * @param inverse if false, the previous arguments tell the pose of N wrt C; if
     *        true they express the pose of C wrt N
     * @return a new instance of InertiaParams that specifies the same inertial
     *         properties of the first parameter, but expressed in the new frame N
     */
    public static InertiaParams rototranslate(InertiaParams inertia,
            float tx, float ty, float tz, float rx, float ry, float rz, boolean inverse)
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

        float tmp = 0;
        if(inverse) {
            // The numbers encode the roto-translation to move from N to C, so I need
            //  to invert them in order to have the transformation C --> N, to express
            //  in N the inertia-parameters currently expressed in C
            tx = -tx;
            ty = -ty;
            tz = -tz;
            //transpose M
            tmp = M[0][1];
            M[0][1] = M[1][0];
            M[1][0] = tmp;

            tmp = M[0][2];
            M[0][2] = M[2][0];
            M[2][0] = tmp;

            tmp = M[1][2];
            M[1][2] = M[2][1];
            M[2][1] = tmp;
        }

        // Compute the COM position in the new frame, applying the translation
        //  and the rotation matrix defined by rx,ry,rz ...
        Vector3 new_com = factory.createVector3();
        translated.setCom(new_com);
        FloatLiteral tmpLiteral;
        // Translation:
        //   Position vector of the COM, with respect to the frame with origin as in N
        //   but same orientation as C:
        final float comx_N = comx - tx;
        final float comy_N = comy - ty;
        final float comz_N = comz - tz;
        // rotation:
        // X ...
        tmp = M[0][0]*comx_N + M[0][1]*comy_N + M[0][2]*comz_N;
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        new_com.setX(tmpLiteral);
        // Y ...
        tmp = M[1][0]*comx_N + M[1][1]*comy_N + M[1][2]*comz_N;
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        new_com.setY(tmpLiteral);
        // Z ...
        tmpLiteral = factory.createFloatLiteral();
        tmp = M[2][0]*comx_N + M[2][1]*comy_N + M[2][2]*comz_N;
        tmpLiteral.setValue(tmp);
        new_com.setZ(tmpLiteral);

        // Now computes the inertia tensor in the new frame

        // Parallel axis theorem; go from C to COM, and from COM to N, with two subsequent translations
        ixx += mass * (comy_N*comy_N + comz_N*comz_N - comy*comy - comz*comz);
        iyy += mass * (comx_N*comx_N + comz_N*comz_N - comx*comx - comz*comz);
        izz += mass * (comx_N*comx_N + comy_N*comy_N - comx*comx - comy*comy);
        ixy += mass * (comx_N*comy_N - comx*comy);
        ixz += mass * (comx_N*comz_N - comx*comz);
        iyz += mass * (comy_N*comz_N - comy*comz);

        // Now consider the rotation of the axes.
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

	public static double length(Vector3 vec) {
	    if( ! (vec.getX() instanceof FloatLiteral) ||
            ! (vec.getY() instanceof FloatLiteral) ||
            ! (vec.getZ() instanceof FloatLiteral))
        {
            throw new RuntimeException("Cannot compute the length of a vector defined with some variables");
        }
        float x = ((FloatLiteral)vec.getX()).getValue();
        float y = ((FloatLiteral)vec.getY()).getValue();
        float z = ((FloatLiteral)vec.getZ()).getValue();
        return Math.sqrt(x*x + y*y + z*z);
    }
}
