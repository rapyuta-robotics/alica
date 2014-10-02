using System;
using System.Collections.Generic;
using Alica;

/*PROTECTED REGION ID(eph1402488634525) ENABLED START*/	//Add additional using directives here
/*PROTECTED REGION END*/
namespace Alica.Validators {
	//Plan:AttackPlan
	public partial class Expressions : AbstractExpressions {

		/* generated comment
		 
		 Task: DefaultTask  -> EntryPoint-ID: 1402488646221

		 */
		public static UtilityFunction GetUtilityFunction1402488634525(Plan plan) {
			/*PROTECTED REGION ID(1402488634525) ENABLED START*/

			return new DefaultUtilityFunction(plan);

			/*PROTECTED REGION END*/
		}

		//State: Attack in Plan: AttackPlan

		/*
		 *		
		 * Transition:
		 *   - Name: , Comment : 
		 *	- Formula:  
		 *		(haveAnotherBall & haveBall  )  
		 *	
		 * Plans in State: 				
		 *   - Plan - (Name): Tackle, (PlanID): 1402489318663 				
		 *   - Plan - (Name): AttackOppDefault, (PlanID): 1402489366699 
		 *
		 * Tasks: 
		 *   - DefaultTask (1225112227903) (Entrypoint: 1402488646221)
		 *
		 * States:
		 *   - Attack (1402488646220)
		 *   - Shoot (1402489396914)
		 */
		public static bool F1402489459382(RunningPlan rp) {
			/*PROTECTED REGION ID(1402489459382) ENABLED START*/
			//WorldModel wm = WorldModel.Get();
			--> "Transition: 1402489459382  not implemented";
			// return false;									
			/*PROTECTED REGION END*/

			bool haveAnotherBall;
			bool haveBall;
		}

		//State: Shoot in Plan: AttackPlan

		/*
		 *		
		 * Transition:
		 *   - Name: Condition-Name-Shoot-Attack, ConditionString: Some nice comment!, Comment :  
		 *
		 * Plans in State: 				
		 *   - Plan - (Name): AttackDefault, (PlanID): 1402488866727 
		 *
		 * Tasks: 
		 *   - DefaultTask (1225112227903) (Entrypoint: 1402488646221)
		 *
		 * States:
		 *   - Attack (1402488646220)
		 *   - Shoot (1402489396914)
		 *
		 * Vars:				
		 *	- TestVar1 (1403772778288) 				
		 *	- VarTest2 (1403772797469) 				
		 *	- NewVar (1403772816953) 				
		 *	- ABC (1403772834750) 
		 */
		public static bool F1402489460694(RunningPlan rp) {
			/*PROTECTED REGION ID(1402489460694) ENABLED START*/
			//WorldModel wm = WorldModel.Get();
			--> "Transition: 1402489460694  not implemented";
			// return false;

			/*PROTECTED REGION END*/

		}

	}
}
