#include "SerializationMasterPlan373109241446504968.h"
/*PROTECTED REGION ID(eph373109241446504968) ENABLED START*/
// Add additional options here
#include "alica_tests/TestWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SerializationMasterPlan (373109241446504968)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 125037907796569874)
//
// States:
//   - PlanB (174185769149002104)
//   - PlanC (458399185905834888)
//   - PlanD (837657643540052235)
//   - EntryState (1886795261620096590)
//   - PlanA (4556827380180239242)
SerializationMasterPlan373109241446504968::SerializationMasterPlan373109241446504968(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con373109241446504968) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SerializationMasterPlan373109241446504968::~SerializationMasterPlan373109241446504968()
{
    /*PROTECTED REGION ID(dcon373109241446504968) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 125037907796569874
 */
std::shared_ptr<UtilityFunction> UtilityFunction373109241446504968::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(373109241446504968) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1491726255888784762 (1491726255888784762)
 *   - Comment:
 *   - Source2Dest: EntryState --> PlanD
 *
 * Precondition: 1693256954385338259 (1693256954385338259)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 */
bool PreCondition1693256954385338259::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1491726255888784762) ENABLED START*/
    auto* worldModel = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return worldModel->serializationTestD;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 2057783493960201520 (2057783493960201520)
 *   - Comment:
 *   - Source2Dest: EntryState --> PlanA
 *
 * Precondition: 3932287302905544988 (3932287302905544988)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 */
bool PreCondition3932287302905544988::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(2057783493960201520) ENABLED START*/
    auto* worldModel = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return worldModel->serializationTestA;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 2606234571502403331 (2606234571502403331)
 *   - Comment:
 *   - Source2Dest: EntryState --> PlanB
 *
 * Precondition: 3461968191733792853 (3461968191733792853)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 */
bool PreCondition3461968191733792853::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(2606234571502403331) ENABLED START*/
    auto* worldModel = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return worldModel->serializationTestB;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3214980101932259560 (3214980101932259560)
 *   - Comment:
 *   - Source2Dest: EntryState --> PlanC
 *
 * Precondition: 2915681556800498724 (2915681556800498724)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 */
bool PreCondition2915681556800498724::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3214980101932259560) ENABLED START*/
    auto* worldModel = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return worldModel->serializationTestC;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods373109241446504968) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
