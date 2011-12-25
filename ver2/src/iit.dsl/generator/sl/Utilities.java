package iit.dsl.generator.sl;

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
		Vector3 temp = tuned.getCom();
		temp.setX(com.getX() * mass);
		temp.setY(com.getY() * mass);
		temp.setZ(com.getZ() * mass);

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
