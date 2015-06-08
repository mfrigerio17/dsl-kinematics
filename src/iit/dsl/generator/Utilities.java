package iit.dsl.generator;

import iit.dsl.generator.common.Parameters;
import iit.dsl.generator.common.Vector3D;
import iit.dsl.kinDsl.InertiaParams;
import iit.dsl.kinDsl.Joint;
import iit.dsl.kinDsl.KinDslFactory;
import iit.dsl.kinDsl.Vector3;
import iit.dsl.kinDsl.FloatLiteral;
import iit.dsl.kinDsl.impl.KinDslFactoryImpl;

public class Utilities {

	private static KinDslFactory factory    = KinDslFactoryImpl.init();
	private static Common utils = Common.getInstance();

	/**
     * Computes the inertia parameters of a rigid body in a different frame.
     *
     * The frame in which the inertia-parameters (COM position and inertia tensor)
     * are currently expressed is C; the new frame in which such parameters
     * have to be expressed is N.
     * The arguments of this function specify the transformation between C and N.
     * If the last argument 'inverse' is false, then the rotation/translation
     * parameters encode the pose of N with respect to C. Otherwise they represent
     * the pose of C with respect to N.
     *
     * The translation is expressed in the current frame. Rotation values are basically
     * euler angles, consecutive rotations about x, y, and z axis, in this order, of the
     * current frame; each rotation is about the axis rotated by the previous one.
     *
     * This method does NOT support parametric properties, ie it only works
     * when the given inertia properties are all floating point constants.
     *
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
        if(Parameters.isParametric(inertia)) {
            throw new RuntimeException("Cannot roto-translate the inertia " +
                                       "properties if they are parametric");
        }
        float mass  = utils.asFloat(inertia.getMass());

        Vector3 com = inertia.getCom();
        float comx = utils.asFloat(com.getX());
        float comy = utils.asFloat(com.getY());
        float comz = utils.asFloat(com.getZ());

        float ixx = utils.asFloat(inertia.getIx());
        float iyy = utils.asFloat(inertia.getIy());
        float izz = utils.asFloat(inertia.getIz());
        float ixy = utils.asFloat(inertia.getIxy());
        float ixz = utils.asFloat(inertia.getIxz());
        float iyz = utils.asFloat(inertia.getIyz());

        InertiaParams translated = factory.createInertiaParams();
        // The mass obviously does not change with a change in the reference frame:
        FloatLiteral newMass = factory.createFloatLiteral();
        newMass.setValue(mass);
        translated.setMass(newMass);

        double[][] tmpM = rotated_X_original(rx, ry, rz);
        float[][] M = new float[3][3];
        for(int r=0; r<tmpM.length; r++) {
            for(int c=0; c<tmpM[0].length; c++) {
                M[r][c] = (float)tmpM[r][c];
            }
        }

        float tmp = 0;
        if(inverse) {
            // The numbers encode the roto-translation to move from N to C, so I need
            //  to invert them in order to have the transformation C --> N, to express
            //  in N the inertia-parameters currently expressed in C
            // Rotate and invert the translation vector
            float tmpx, tmpy, tmpz;
            tmpx = M[0][0] * tx + M[0][1] * ty + M[0][2] * tz;
            tmpy = M[1][0] * tx + M[1][1] * ty + M[1][2] * tz;
            tmpz = M[2][0] * tx + M[2][1] * ty + M[2][2] * tz;
            tx = -tmpx;
            ty = -tmpy;
            tz = -tmpz;
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
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        translated.setIx(tmpLiteral);
        // Iyy
        tmp =     M[1][0] * M[1][0] * ixx +
                  M[1][1] * M[1][1] * iyy +
                  M[1][2] * M[1][2] * izz +
             -2 * M[1][0] * M[1][1] * ixy +
             -2 * M[1][0] * M[1][2] * ixz +
             -2 * M[1][1] * M[1][2] * iyz;
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        translated.setIy(tmpLiteral);
        // Izz
        tmp =     M[2][0] * M[2][0] * ixx +
                  M[2][1] * M[2][1] * iyy +
                  M[2][2] * M[2][2] * izz +
             -2 * M[2][0] * M[2][1] * ixy +
             -2 * M[2][0] * M[2][2] * ixz +
             -2 * M[2][1] * M[2][2] * iyz;
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        translated.setIz(tmpLiteral);
        // Ixy
        tmp =- M[0][0] * M[1][0] * ixx +
             - M[0][1] * M[1][1] * iyy +
             - M[0][2] * M[1][2] * izz +
             (M[0][0]*M[1][1] + M[0][1]*M[1][0]) * ixy +
             (M[0][0]*M[1][2] + M[0][2]*M[1][0]) * ixz +
             (M[0][1]*M[1][2] + M[0][2]*M[1][1]) * iyz;
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        translated.setIxy(tmpLiteral);
        // Ixz
        tmp =- M[0][0] * M[2][0] * ixx +
             - M[0][1] * M[2][1] * iyy +
             - M[0][2] * M[2][2] * izz +
            (M[0][0]*M[2][1] + M[0][1]*M[2][0]) * ixy +
            (M[0][0]*M[2][2] + M[0][2]*M[2][0]) * ixz +
            (M[0][1]*M[2][2] + M[0][2]*M[2][1]) * iyz;
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        translated.setIxz(tmpLiteral);
        // Iyz
        tmp =- M[1][0] * M[2][0] * ixx +
             - M[1][1] * M[2][1] * iyy +
             - M[1][2] * M[2][2] * izz +
             (M[1][0]*M[2][1] + M[1][1]*M[2][0]) * ixy +
             (M[1][0]*M[2][2] + M[1][2]*M[2][0]) * ixz +
             (M[1][1]*M[2][2] + M[1][2]*M[2][1]) * iyz;
        tmpLiteral = factory.createFloatLiteral();
        tmpLiteral.setValue(tmp);
        translated.setIyz(tmpLiteral);

        return translated;
    }

    /**
     * Returns the inertia tensor as a 3x3 double array
     */
    public static double[][] getInertiaTensor(InertiaParams properties)
    {
        double[][] ret = new double[3][3];
        ret[0][0] = utils.asFloat(properties.getIx());
        ret[1][1] = utils.asFloat(properties.getIy());
        ret[2][2] = utils.asFloat(properties.getIz());

        ret[0][1] = ret[1][0] = - utils.asFloat(properties.getIxy());
        ret[0][2] = ret[2][0] = - utils.asFloat(properties.getIxz());
        ret[1][2] = ret[2][1] = - utils.asFloat(properties.getIyz());

        return ret;
    }

    /**
     * Roto-translates the given 3x3 inertia tensor
     * @param Iin the input tensor
     * @param mass the total mass of the rigid body
     * @param t the translation vector between the two reference frames
     * @param out_R_in the 3x3 rotation matrix that transforms coordinates of
     *         the original reference frame into the rotated (desired) frame
     * @return a 3x3 double array with the inertia tensor resulting from the
     *    translation and rotation of the given tensor
     */
    public static double[][] rototranslateITensor(double[][] Iin, double mass, Vector3D t, double[][] out_R_in)
    {
        // Translation; paralles axis theorem
        double[][] translated = new double[3][3];

        translated[0][0] = Iin[0][0] - mass * (t.z*t.z + t.y*t.y);
        translated[1][1] = Iin[1][1] - mass * (t.z*t.z + t.x*t.x);
        translated[2][2] = Iin[2][2] - mass * (t.y*t.y + t.x*t.x);

        translated[0][1] = translated[1][0] = Iin[0][1] + mass * t.x*t.y;
        translated[0][2] = translated[2][0] = Iin[0][2] + mass * t.x*t.z;
        translated[1][2] = translated[2][1] = Iin[1][2] + mass * t.y*t.z;

        // Rotation: I2 = 2_R_1 * I1 * (2_R_1)^T
        if(out_R_in == null) return translated;

        double[][] in_R_out = transpose3x3(out_R_in);
        double[][] tmp = matrix3x3Mult(out_R_in, translated);

        return matrix3x3Mult(tmp, in_R_out);
    }

    /**
     * Rounds each element of the given matrix according to the factor.
     *
     * Basically:
     *     mx[.][.] = round(mx[.][.] * factor) / factor
     *
     * @param mx
     * @param factor
     */
    public static void round(double[][] mx, int factor)
    {
        for(int r=0; r<mx.length; r++) {
            for(int c=0; c<mx[0].length; c++) {
                mx[r][c] = Math.round(mx[r][c]*factor)/ (double)(factor);
            }
        }
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

    /**
     * Creates and returns a 3x3 rotation matrix corresponding to the rotation angles
     * in the arguments. The rotation matrix is in the form rotated_X_original, that
     * is, it multiplies coordinate vectors expressed in a reference frame and gives
     * back the coordinates in the frame which is rotated with respect to the first one.
     * The arguments are interpreted as consecutive rotations about the x, y and z
     * axis, all expressed in radians.
     * @param rx the rotation amount about the x axis
     * @param ry the rotation amount about the y axis as it results after the first rotation
     * @param rz the rotation amount about the z axis, after the two previous rotations
     * @return a 3x3 rotation matrix that transforms coordinate vectors according to
     *          the rotation specified in the arguments
     */
    public static double[][] rotated_X_original(double rx, double ry, double rz) {
        double sx = Math.sin(rx);
        double sy = Math.sin(ry);
        double sz = Math.sin(rz);
        double cx = Math.cos(rx);
        double cy = Math.cos(ry);
        double cz = Math.cos(rz);

        // The following is the matrix M that transform coordinates in the original frame
        //  into coordinates of the new frame:  new_X_current
        // [  cos(ry)*cos(rz)    cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz)     sin(rx)*sin(rz) - cos(rx)*sin(ry)*cos(rz) ]
        // [                                                                                                             ]
        // [ - cos(ry)*sin(rz)   cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz)     cos(rx)*sin(ry)*sin(rz) + sin(rx)*cos(rz) ]
        // [                                                                                                             ]
        // [      sin(ry)                    - sin(rx)*cos(ry)                              cos(rx)*cos(ry)              ]
        double[][] M = {
                { cy*cz,   cx*sz + sx*sy*cz,     sx*sz - cx*sy*cz },
                {-cy*sz,   cx*cz - sx*sy*sz,     cx*sy*sz + sx*cz },
                {  sy  ,     - sx*cy       ,     cx*cy }
                };
        return M;
	}

	/**
	 * Computes the 3 rotation parameters (ie Euler angles) given a 3x3 rotation matrix.
	 * This is basically the inverse operation of rotated_X_original(double, double, double)
	 * See the documentation of such a function for the semantics of the angles and the matrix.
	 * @param mx a 3x3 rotation matrix in the form 'rotated_X_original'
	 * @return a vector of three elements, representing the consecutive rotations about the
	 *          x, y, and z axis that correspond to the given matrix
	 */
	public static double[] get_rxryrz(double mx[][]) {
	    double rotx = Math.atan2(-mx[2][1], mx[2][2]); // atan( sx cy , cx cy ) = atan( sx,cx )
	    double sx = Math.sin(rotx);
	    double roty = Math.atan2(mx[2][0], mx[2][1]/(-sx)); // atan( sy / (sx cy / sx) ) = atan( sy/cy )
	    double rotz = Math.atan2(-mx[1][0], mx[0][0]);

	    double[] ret = {rotx, roty, rotz};
	    return ret;
	}
    /**
     * See rotated_X_original()
     */
    public static double[][] original_X_rotated(double rx, double ry, double rz) {
        double sx = Math.sin(rx);
        double sy = Math.sin(ry);
        double sz = Math.sin(rz);
        double cx = Math.cos(rx);
        double cy = Math.cos(ry);
        double cz = Math.cos(rz);

        double[][] M = {
                {     cy*cz      ,       -cy*sz      ,      sy   },
                {cx*sz + sx*sy*cz,   cx*cz - sx*sy*sz,   - sx*cy },
                {sx*sz - cx*sy*cz,   cx*sy*sz + sx*cz,     cx*cy }
                };
        return M;
    }

    /**
     * Multiplies a 3x3 matrix with a 3x1 column vector.
     * @param mx
     * @param v
     * @return
     */
    public static Vector3D matrix3x3Mult(double[][] mx, Vector3D v)
    {
        Vector3D ret = new Vector3D(.0,.0,.0);

        ret.x = mx[0][0] * v.x + mx[0][1] * v.y + mx[0][2] * v.z;
        ret.y = mx[1][0] * v.x + mx[1][1] * v.y + mx[1][2] * v.z;
        ret.z = mx[2][0] * v.x + mx[2][1] * v.y + mx[2][2] * v.z;

        return ret;
    }
    /**
     * Multiplies two 3x3 matrices
     * @param mx1
     * @param mx2
     * @return
     */
    public static double[][] matrix3x3Mult(double[][] mx1, double[][] mx2)
    {
        double[][] ret = new double[3][3];

        double tmp = 0;
        for(int r=0; r<ret.length; r++) {
            for(int c=0; c<ret[0].length; c++) {
                for(int k=0; k<mx2.length; k++) {
                    tmp += mx1[r][k] * mx2[k][c];
                }
                ret[r][c] = tmp;
                tmp = 0;
            }
        }
        return ret;
    }

    public static double[][] transpose3x3(double[][] mx)
    {
        double[][] ret = new double[3][3];

        for(int r=0; r<3; r++) {
            for(int c=0; c<3; c++) {
                ret[r][c] = mx[c][r];
            }
        }
        return ret;
    }

	/**
	 * Calculates the joint axis unit vector, in the coordinate frame of the
	 * supporting link (i.e. the predecessor of the joint)
	 * @param joint
	 * @return
	 */
	public static Vector3D jointAxis(Joint joint)
	{
	    Vector3 rot = joint.getRefFrame().getRotation();
        float rx = utils.asFloat(rot.getX());
        float ry = utils.asFloat(rot.getY());
        float rz = utils.asFloat(rot.getZ());

        // The joint axis would be the third column of the rotation matrix
        //  'link_R_joint'. Since we can only get the transpose of such a matrix,
        //  we return the third row

        double[][] joint_R_link = rotated_X_original(rx, ry, rz);
        return new Vector3D(joint_R_link[2][0],joint_R_link[2][1],joint_R_link[2][2]);
	}
}
