package iit.dsl.generator.sl;

import java.util.Arrays;
import java.util.List;

import org.eclipse.xtext.EcoreUtil2;

import iit.dsl.generator.common.Parameters;
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
	public static final String makefileFolder = "makefiles";

	private static KinDslPackage dslPackage = KinDslPackageImpl.init();
	private static KinDslFactory factory    = KinDslFactoryImpl.init();

	public static InertiaParams tuneForSL(InertiaParams inertia)
	{
	    if(Parameters.isParametric(inertia)) {
	        throw new RuntimeException("Cannot tune inertia properties for SL" +
	                 " if they depend on runtime parameters");
	    }
	    iit.dsl.generator.Common helper = iit.dsl.generator.Common.getInstance();
		float mass  = helper.asFloat(inertia.getMass());
		Vector3 com = inertia.getCom();

		InertiaParams tuned = (InertiaParams)factory.create(dslPackage.getInertiaParams());
		tuned.setCom((Vector3)factory.create(dslPackage.getVector3()));
		FloatLiteral sameMass = (FloatLiteral)factory.create(dslPackage.getFloatLiteral());
		sameMass.setValue(mass);
		tuned.setMass(sameMass);

		// Multiply the com by the mass

		Vector3 newCOM = tuned.getCom();
		FloatLiteral newx = factory.createFloatLiteral();
		FloatLiteral newy = factory.createFloatLiteral();
		FloatLiteral newz = factory.createFloatLiteral();
		newx.setValue( ((FloatLiteral)com.getX()).getValue() * mass);
		newy.setValue( ((FloatLiteral)com.getY()).getValue() * mass);
		newz.setValue( ((FloatLiteral)com.getZ()).getValue() * mass);

		newCOM.setX(newx);
		newCOM.setY(newy);
		newCOM.setZ(newz);

        // Make a copy and not just a set(), because apparently the set() also
        //  sets the original property to null...
        tuned.setIx(EcoreUtil2.copy(inertia.getIx()));
        tuned.setIy(EcoreUtil2.copy(inertia.getIy()));
        tuned.setIz(EcoreUtil2.copy(inertia.getIz()));
        // Invert the centrifugal moments because SL wants the elements of the inertia tensor
        tuned.setIxy(helper.invert(inertia.getIxy()));
        tuned.setIxz(helper.invert(inertia.getIxz()));
        tuned.setIyz(helper.invert(inertia.getIyz()));

		return tuned;
	}



	public static List<String> getDefaultMiscSensors() {
        if(defaultMiscSensors == null) {
            String foo[] =
            {"B_Q_0", "B_Q_1", "B_Q_2", "B_Q_3", "B_Qd_0", "B_Qd_1", "B_Qd_2", "B_Qd_3",
             "B_Qdd_0", "B_Qdd_1", "B_Qdd_2", "B_Qdd_3", "B_Ad_A", "B_Ad_B", "B_Ad_G",
             "B_Add_A", "B_Add_B", "B_Add_G", "B_X", "B_Y", "B_Z", "B_Xd", "B_Yd", "B_Zd",
             "B_Xdd", "B_Ydd", "B_Zdd", "TIME_MOTOR"};

            defaultMiscSensors = Arrays.asList(foo);
        }
        return defaultMiscSensors;
	}

	private static List<String> defaultMiscSensors = null;
}
