package de.unikassel.vs.alica.planDesigner.serialize;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.modelMixIns.*;

import java.io.File;
import java.io.IOException;

public class JSonSerializer {

    public static void main(String[] args) {

        ObjectMapper mapper = new ObjectMapper();
        mapper.enable(SerializationFeature.INDENT_OUTPUT);
        mapper.configure(DeserializationFeature.USE_JAVA_ARRAY_FOR_JSON_ARRAY, true);
        mapper.addMixIn(EntryPoint.class, EntryPointMixIn.class);
        mapper.addMixIn(VariableBinding.class, VariableBindingMixIn.class);
        mapper.addMixIn(AnnotatedPlan.class, AnnotatedPlanMixIn.class);
        mapper.addMixIn(Plan.class, PlanMixIn.class);
        mapper.addMixIn(Quantifier.class, QuantifierMixIn.class);
        mapper.addMixIn(State.class, StateMixIn.class);
        mapper.addMixIn(Synchronisation.class, SynchronizationMixIn.class);
        mapper.addMixIn(Task.class, TaskMixIn.class);
        mapper.addMixIn(Transition.class, TransitionMixIn.class);

        Plan plan = new Plan();
        plan.setName("TestMasterPlan2");
        plan.setMasterPlan(true);
        plan.setComment("Test Comment String");
        plan.setRelativeDirectory("");

        Plan plan2 = new Plan();
        plan2.setName("abc");
        plan2.setMasterPlan(true);
        plan2.setComment("Test Comment String");
        plan2.setRelativeDirectory("");

        State state = new State();
        state.setName("Stop");
        state.setComment("Stops the robot");
        state.setParentPlan(plan);

        Behaviour behaviour = new Behaviour();
        behaviour.setFrequency(30);
        behaviour.setComment("Behaviour Comment String");
        behaviour.setName("TestBehaviour");
        behaviour.setRelativeDirectory("result.json");
        ConfAbstractPlanWrapper wrapper = new ConfAbstractPlanWrapper();
        wrapper.setAbstractPlan(behaviour);
        state.addConfAbstractPlanWrapper(wrapper);

        Behaviour behaviour2 = new Behaviour();
        behaviour2.setFrequency(30);
        behaviour2.setComment("Behaviour Comment String");
        behaviour2.setName("TestBehaviour2");
        behaviour2.setRelativeDirectory("result.json");
        ConfAbstractPlanWrapper wrapper2 = new ConfAbstractPlanWrapper();
        wrapper2.setAbstractPlan(behaviour2);
        state.addConfAbstractPlanWrapper(wrapper2);

        plan.getStates().add(state);

        State state2 = new State();
        state2.setName("Stop2");
        state2.setComment("Stops the robot too");
        state2.setParentPlan(plan);
        ConfAbstractPlanWrapper wrapper3 = new ConfAbstractPlanWrapper();
        wrapper3.setAbstractPlan(plan);
        state2.addConfAbstractPlanWrapper(wrapper3);

        plan.getStates().add(state2);

        EntryPoint entryPoint = new EntryPoint();
        entryPoint.setMinCardinality(1);
        entryPoint.setMaxCardinality(2);
        entryPoint.setState(state);
        entryPoint.setName("TestEP");
        entryPoint.setPlan(plan);

        state.setEntryPoint(entryPoint);
        plan.getEntryPoints().add(entryPoint);

        TaskRepository taskRepository = new TaskRepository();
        taskRepository.setName("taskrepository");
        taskRepository.setRelativeDirectory("result.json");

        Task task =  new Task();
        task.setName("DefaultTask");

        entryPoint.setTask(task);
        task.setTaskRepository(taskRepository);
        taskRepository.addTask(task);

        Transition transition = new Transition();
        transition.setInState(state2);
        transition.setOutState(state);
        state.getOutTransitions().add(transition);
        state2.getInTransitions().add(transition);
        plan.getTransitions().add(transition);

//        Behaviour behaviour = new Behaviour();
//        behaviour.setFrequency(30);
//        behaviour.setComment("Behaviour Comment String");
//        behaviour.setName("TestBehaviour");
//
//        Variable var = new Variable();
//        var.setQuantifierType("Variable Type String");
//        var.setComment("Variable Comment String");
//        var.setName("TestVariable");
//        behaviour.getVariables().put(var);
//
//        PreCondition preCondition = new PreCondition();
//        preCondition.setEnabled(true);
//        preCondition.setPluginName("Plugin Name String");
//        preCondition.setComment("PreCondition Comment String");
//        preCondition.setName("TestPreCondition");
//        behaviour.setPreCondition(preCondition);
//
//        RuntimeCondition runtimeCondition = new RuntimeCondition();
//        runtimeCondition.setPluginName("Plugin Name String");
//        runtimeCondition.setComment("RuntimeCondition Comment String");
//        runtimeCondition.setName("TestRuntimeCondition");
//        behaviour.setRuntimeCondition(runtimeCondition);
//
//        PostCondition postCondition = new PostCondition();
//        postCondition.setPluginName("Plugin Name String");
//        postCondition.setComment("PostCondition Comment String");
//        postCondition.setName("TestPostCondition");
//        behaviour.setPostCondition(postCondition);
//
//        behaviour.setRelativeDirectory("result.json");
//
//        CapValue val = new CapValue();
//        val.setName("TestCapValue");
//        val.setComment("CapValue Comment String");
//
//        Characteristic charac = new Characteristic();
//        charac.setName("TestCharacteristic");
//        charac.setComment("Characteristic Comment String");
//        charac.setValue(val);
//
//        Role role = new Role();
//        role.setName("TestRole");
//        role.setComment("Role Comment String");
//        role.getCharacteristics().put(charac);

        try {

        File outfile = new File(plan.getName() + ".pml");
        //TODO adds list brackets to serialized json causing Cannot deserialize instance of `Plan` out of START_ARRAY tokenk
            mapper.writeValue(outfile, plan);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}
