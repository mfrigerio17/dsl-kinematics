package iit.dsl.generator;

import iit.dsl.kinDsl.InertiaParams;
import iit.dsl.kinDsl.KinDslFactory;
import iit.dsl.kinDsl.Vector3;
import iit.dsl.kinDsl.FloatLiteral;
import iit.dsl.kinDsl.impl.KinDslFactoryImpl;

public class Utilities {

	private static KinDslFactory factory    = KinDslFactoryImpl.init();

	/**
     * Express the inertia parameters in a frame parallel to the current one
     * @param inertia the input parameters to be expressed in the new frame
     * @param
     * TODO
     * @return a new instance of InertiaParams that specifies the same inertial
     *         properties of the first parameter, but expressed in a frame translated
     *         by 'vector'
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
        // First use the parallel axis theorem for the translation
        //   (note that the formula to translate the centrifugal moment has a '+',
        //    but since we are updating the elements of the inertia tensor, we
        //    use the minus sign in front of 'mass * ...')

        ixx = ixx + mass * (ty*ty + tz*tz);
        iyy = iyy + mass * (tx*tx + tz*tz);
        izz = izz + mass * (tx*tx + ty*ty);
        ixy = ixy - mass * (tx*ty);
        ixz = ixz - mass * (tx*tz);
        iyz = iyz - mass * (ty*tz);

        // Then do M * I * M^T, where M is the rotation matrix...
        // Ixx
        tmp = (sx*sz - cx*sy*cz) *
              (izz*(sx*sz - cx*sy*cz) - iyz*(cx*sz + sx*sy*cz) - ixz*cy*cz) + (cx*sz + sx*sy*cz) *
              (- iyz * (sx*sz - cx*sy*cz) + iyy*(cx*sz + sx*sy*cz) - ixy*cy*cz) +
              cy*cz*(- ixz*(sx*sz - cx*sy*cz)- ixy*(cx*sz + sx*sy*cz) + ixx*cy*cz);
        translated.setIx(tmp);
        // Iyy
        tmp = (cx*sy*sz + sx*cz) *
              (- iyz*(cx*cz - sx*sy*sz) + izz*(cx*sy*sz + sx*cz) + ixz*cy*sz) +
              (cx*cz - sx*sy*sz) *
              (iyy*(cx*cz - sx*sy*sz) - iyz*(cx*sy*sz + sx*cz) + ixy*cy*sz) -
              cy*sz*(- ixy*(cx*cz - sx*sy*sz) - ixz*(cx*sy*sz + sx*cz) - ixx*cy*sz);
        translated.setIy(tmp);
        // Izz
        tmp = cx*cy*(- ixz*sy + iyz*sx*cy + izz*cx*cy) - sx*cy*(- ixy*sy - iyy*sx*cy - iyz*cx*cy)
                + sy*(ixx*sy + ixy*sx*cy - ixz*cx*cy);
        translated.setIz(tmp);
        // Ixy
        tmp = (sx*sz - cx*sy*cz) * (- iyz*(cx*cz - sx*sy*sz) + izz*(cx*sy*sz + sx*cz) + ixz*cy*sz)
                        + (cx*sz + sx*sy*cz)*(iyy*(cx*cz - sx*sy*sz)
                        - iyz*(cx*sy*sz + sx*cz) + ixy*cy*sz)
                        + cy*cz*(- ixy*(cx*cz - sx*sy*sz)
                        - ixz*(cx*sy*sz + sx*cz) - ixx*cy*sz);
        translated.setIxy(tmp);
        // Ixz
        tmp = (- ixz*sy + iyz*sx*cy + izz*cx*cy)
                *(sx*sz - cx*sy*cz) + (- ixy*sy - iyy*sx*cy
                        - iyz*cx*cy)*(cx*sz + sx*sy*cz)
                        + cy*(ixx*sy + ixy*sx*cy - ixz*cx*cy)*cz;
        translated.setIxz(tmp);
        // Iyz
        tmp = (- ixy*sy - iyy*sx*cy - iyz*cx*cy)
                *(cx*cz - sx*sy*sz) + (- ixz*sy + iyz*sx*cy
                        + izz*cx*cy)*(cx*sy*sz + sx*cz)
                        - cy*(ixx*sy + ixy*sx*cy - ixz*cx*cy)*sz;
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
