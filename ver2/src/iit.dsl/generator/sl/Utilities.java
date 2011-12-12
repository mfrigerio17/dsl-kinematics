package iit.dsl.generator.sl;

import iit.dsl.kinDsl.InertiaParams;
import iit.dsl.kinDsl.KinDslFactory;
import iit.dsl.kinDsl.KinDslPackage;
import iit.dsl.kinDsl.Vector3;
import iit.dsl.kinDsl.impl.KinDslFactoryImpl;
import iit.dsl.kinDsl.impl.KinDslPackageImpl;

import org.eclipse.emf.common.util.EList;

public abstract class Utilities {
	public static final String userConfigFolder = "config";
	public static final String linkParamsFile = "LinkParameters.cf";

	private static KinDslPackage dslPackage = KinDslPackageImpl.init();
	private static KinDslFactory factory    = KinDslFactoryImpl.init();

	public static InertiaParams tuneForSL(InertiaParams inertia) {
		float mass  = inertia.getMass();
		EList<Float> com = inertia.getCom().getItems();

		InertiaParams tuned = (InertiaParams)factory.create(dslPackage.getInertiaParams());
		tuned.setCom((Vector3)factory.create(dslPackage.getVector3()));
		tuned.setMass(mass);

		// Multiply the com by the mass
		EList<Float> temp = tuned.getCom().getItems();
		temp.add(com.get(0) * mass);
		temp.add(com.get(1) * mass);
		temp.add(com.get(2) * mass);

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
