
using System;
using System.Collections.Generic;
using Alica;
using AD = AutoDiff;

/*PROTECTED REGION ID(ch1402488634525) ENABLED START*/
//Add additional using directives here
/*PROTECTED REGION END*/

namespace Alica.Validators {
	public partial class ConstraintFunctions {
		//Plan:AttackPlan

		/*		
		 * Tasks: 
		 * - EP:1402488646221 : DefaultTask (1225112227903)
		 *
		 * States:
		 * - Attack (1402488646220)
		 * - Shoot (1402489396914)
		 *
		 * Vars:				
		 * - TestVar1 (1403772778288) 				
		 * - VarTest2 (1403772797469) 				
		 * - NewVar (1403772816953) 				
		 * - ABC (1403772834750) 
		 */

// State: Attack

// State: Attack

		/*		
		 * Transition: 
		 * - Name: 
		 * - Formula:  
		 *		(haveAnotherBall & haveBall  )  
		 *
		 * Plans in State: 				
		 * - Plan - (Name): Tackle, (PlanID): 1402489318663 				
		 * - Plan - (Name): AttackOppDefault, (PlanID): 1402489366699 
		 */
		public static void GetConstraint1402489460549(ConstraintDescriptor c, RunningPlan rp) {
			/*PROTECTED REGION ID(cc1402489459382) ENABLED START*/
			//WorldModel wm = WorldModel.Get();
			/*PROTECTED REGION END*/
		}

// State: Shoot

		/*		
		 * Transition: 
		 * - Name: Condition-Name-Shoot-Attack
		 * - ConditionString: Some nice comment!
		 *
		 * Plans in State: 				
		 * - Plan - (Name): AttackDefault, (PlanID): 1402488866727 
		 * Static Variables: [TestVar1, VarTest2, NewVar, ABC]
		 * Domain Variables:

		 */
		public static void GetConstraint1402489462088(ConstraintDescriptor c, RunningPlan rp) {
			/*PROTECTED REGION ID(cc1402489460694) ENABLED START*/
			//WorldModel wm = WorldModel.Get();
			/*PROTECTED REGION END*/
		}

// State: Shoot

	}
}
