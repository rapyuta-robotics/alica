using System;
using System.Collections.Generic;
using Alica;

/*PROTECTED REGION ID(eph1402488870347) ENABLED START*/	//Add additional using directives here
/*PROTECTED REGION END*/
namespace Alica.Validators {
	//Plan:GoalPlan
	public partial class Expressions : AbstractExpressions {

		//Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): test, (Comment) :  

		/* 
		 * Available Vars:
		 *	- test (1403773747758)
		 */
		public static bool F1403773741874(RunningPlan rp) {
			/*PROTECTED REGION ID(1403773741874) ENABLED START*/
			//WorldModel wm = WorldModel.Get();
			return true;
			/*PROTECTED REGION END*/
		}

		/* generated comment
		 
		 Task: DefaultTask  -> EntryPoint-ID: 1402488881800

		 */
		public static UtilityFunction GetUtilityFunction1402488870347(Plan plan) {
			/*PROTECTED REGION ID(1402488870347) ENABLED START*/

			return new DefaultUtilityFunction(plan);

			/*PROTECTED REGION END*/
		}

		//State: Shoot in Plan: GoalPlan

		//State: Miss in Plan: GoalPlan

		//State: Scored in Plan: GoalPlan

	}
}
