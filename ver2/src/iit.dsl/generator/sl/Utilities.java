package iit.dsl.generator.sl;

import iit.dsl.kinDsl.FloatLiteral;
import iit.dsl.kinDsl.InertiaParams;
import iit.dsl.kinDsl.KinDslFactory;
import iit.dsl.kinDsl.KinDslPackage;
import iit.dsl.kinDsl.Vector3;
import iit.dsl.kinDsl.impl.KinDslFactoryImpl;
import iit.dsl.kinDsl.impl.KinDslPackageImpl;

public abstract class Utilities {
	public static final String userConfigFolder = "config";
	public static final String linkParamsFile = "LinkParameters.cf";

	private static KinDslPackage dslPackage = KinDslPackageImpl.init();
	private static KinDslFactory factory    = KinDslFactoryImpl.init();

	public static InertiaParams tuneForSL(InertiaParams inertia) {
		float mass  = inertia.getMass();
		Vector3 com = inertia.getCom();

		InertiaParams tuned = (InertiaParams)factory.create(dslPackage.getInertiaParams());
		tuned.setCom((Vector3)factory.create(dslPackage.getVector3()));
		tuned.setMass(mass);

		// Multiply the com by the mass

		if( ! (com.getX() instanceof FloatLiteral) ||
			! (com.getY() instanceof FloatLiteral) ||
			! (com.getZ() instanceof FloatLiteral))
		{
			throw new RuntimeException("Cannot tune inertia parameters for SL if the COM is defined with some variables");
		}

		Vector3 newCOM = tuned.getCom();
		FloatLiteral newx = (FloatLiteral)factory.createFloatLiteral();
		FloatLiteral newy = (FloatLiteral)factory.createFloatLiteral();
		FloatLiteral newz = (FloatLiteral)factory.createFloatLiteral();
		newx.setValue( ((FloatLiteral)com.getX()).getValue() * mass);
		newy.setValue( ((FloatLiteral)com.getY()).getValue() * mass);
		newz.setValue( ((FloatLiteral)com.getZ()).getValue() * mass);

		newCOM.setX(newx);
		newCOM.setY(newy);
		newCOM.setZ(newz);

		// Add a minus sign in front of the centrifugal moments
		tuned.setIx (inertia.getIx());
		tuned.setIy (inertia.getIy());
		tuned.setIz (inertia.getIz());
		tuned.setIxy(-inertia.getIxy());
		tuned.setIxz(-inertia.getIxz());
		tuned.setIyz(-inertia.getIyz());
		return tuned;
	}
}
