package iit.dsl.generator;

import org.eclipse.emf.common.util.EList;

import iit.dsl.kinDsl.KinDslFactory;
import iit.dsl.kinDsl.KinDslPackage;
import iit.dsl.kinDsl.impl.KinDslPackageImpl;
import iit.dsl.kinDsl.impl.KinDslFactoryImpl;
import iit.dsl.kinDsl.InertiaParams;
import iit.dsl.kinDsl.Vector3;

public class Utilities {
	
	private static KinDslPackage dslPackage = KinDslPackageImpl.init();
	private static KinDslFactory factory    = KinDslFactoryImpl.init();

	public static InertiaParams translate(InertiaParams inertia, Vector3 vector) {
		float mass  = inertia.getMass();
		Vector3 com = inertia.getCom();
		float x = com.getItems().get(0);
		float y = com.getItems().get(1);
		float z = com.getItems().get(2);
		
		InertiaParams translated = (InertiaParams)factory.create(dslPackage.getInertiaParams());
		translated.setCom((Vector3)factory.create(dslPackage.getVector3()));
		translated.setMass(mass);

		// The returned inertia is supposed to be specified at the center of mass,
		//  thus the center of mass position is 0,0,0 by definition
		EList<Float> temp = translated.getCom().getItems();
		temp.add((float)0.0);
		temp.add((float)0.0);
		temp.add((float)0.0);

		translated.setIx (inertia.getIx()  + mass * (y*y + z*z) );
		translated.setIy (inertia.getIy()  + mass * (x*x + z*z) );
		translated.setIz (inertia.getIz()  + mass * (x*x + y*y) );
		translated.setIxy(inertia.getIxy() + mass * (x*y));
		translated.setIxz(inertia.getIxz() + mass * (x*z));
		translated.setIyz(inertia.getIyz() + mass * (y*z));
		return translated;
	}
}
