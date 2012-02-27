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
	 * @param vector the displacement of the new frame with respect to the current one,
	 *        expressed in the latter
	 * @return a new instance of InertiaParams that specifies the same inertial
	 *         properties of the first parameter, but expressed in a frame translated
	 *         by 'vector'
	 */
	public static InertiaParams translate(InertiaParams inertia, Vector3 vector) {
		float mass  = inertia.getMass();
		Vector3 com = inertia.getCom();
		if( ! (com.getX() instanceof FloatLiteral) ||
			! (com.getY() instanceof FloatLiteral) ||
			! (com.getZ() instanceof FloatLiteral))
		{
			throw new RuntimeException("Cannot translate inertia parameters if the COM is defined with some variables");
		}

		float x = ((FloatLiteral)com.getX()).getValue();
		float y = ((FloatLiteral)com.getY()).getValue();
		float z = ((FloatLiteral)com.getZ()).getValue();

		InertiaParams translated = factory.createInertiaParams();
		translated.setCom(factory.createVector3());
		translated.setMass(mass);

		// The returned inertia is supposed to be specified at the center of mass,
		//  thus the center of mass position is 0,0,0 by definition
		Vector3 temp = translated.getCom();
		FloatLiteral zero = factory.createFloatLiteral();
		zero.setValue((float)0.0);
		temp.setX(zero);
		temp.setY(zero);
		temp.setZ(zero);

		// The parallel axis theorem:
		translated.setIx (inertia.getIx()  + mass * (y*y + z*z) );
		translated.setIy (inertia.getIy()  + mass * (x*x + z*z) );
		translated.setIz (inertia.getIz()  + mass * (x*x + y*y) );
		//  note that the formula to translate the centrifugal moment has a '+',
		//   but since we are updating the elements of the inertia tensor, we
		//   use the minus sign in front of 'mass * ...' :
		translated.setIxy(inertia.getIxy() - mass * (x*y));
		translated.setIxz(inertia.getIxz() - mass * (x*z));
		translated.setIyz(inertia.getIyz() - mass * (y*z));

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
