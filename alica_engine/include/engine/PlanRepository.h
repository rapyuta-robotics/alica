#pragma once

#include <iostream>
#include <unordered_map>

namespace alica
{
class Behaviour;
class Capability;
class Characteristic;
class EntryPoint;
class Plan;
class PlanType;
class Quantifier;
class RoleSet;
class Role;
class State;
class Synchronisation;
class TaskRepository;
class Task;
class Transition;
class Condition;
class Variable;
class Configuration;
class ConfAbstractPlanWrapper;
class ModelFactory;
class Factory;
class ExpressionHandler;
class ModelManager;
class TransitionCondition;
class TransitionConditionRepository;

/**
 * The PlanRepository holds the ALICA program, neatly separated into different Dictionaries.
 * It is especially useful to map element Ids back to their object, e.g., when receiving messages referring to plan
 * elements.
 */
class PlanRepository
{
public:
    PlanRepository();
    virtual ~PlanRepository();

    template <typename T>
    using MapType = std::unordered_map<int64_t, T*>;

    template <typename T>
    class Accessor
    {
        class iterator
        {
        public:
            iterator(typename MapType<T>::const_iterator inner)
                    : _innerIter(inner)
            {
            }
            bool operator!=(const iterator& o) const { return _innerIter != o._innerIter; }
            bool operator==(const iterator& o) const { return _innerIter == o._innerIter; }
            const T* operator*() const { return _innerIter->second; }

            iterator& operator++()
            {
                ++_innerIter;
                return *this;
            }

        private:
            typename MapType<T>::const_iterator _innerIter;
        };

    public:
        Accessor(const MapType<T>& map)
                : _ref(map)
        {
        }

        const T* operator[](int64_t id) const { return find(id); }
        const T* find(int64_t id) const
        {
            typename MapType<T>::const_iterator it = _ref.find(id);
            return (it == _ref.end() ? nullptr : it->second);
        }
        Accessor(const Accessor&) = delete;
        Accessor(Accessor&&) = default;

        Accessor& operator=(const Accessor&) = delete;
        Accessor& operator=(Accessor&&) = delete;

        iterator begin() const { return iterator(_ref.begin()); }
        iterator end() const { return iterator(_ref.end()); }
        unsigned long size() const { return _ref.size(); }

    private:
        const MapType<T>& _ref;
    };

    const Accessor<Behaviour> getBehaviours() const { return Accessor<Behaviour>(_behaviours); }
    const Accessor<Capability> getCapabilities() const { return Accessor<Capability>(_capabilities); }
    const Accessor<Characteristic> getCharacteristics() const { return Accessor<Characteristic>(_characteristics); }
    const Accessor<EntryPoint> getEntryPoints() const { return Accessor<EntryPoint>(_entryPoints); }
    const Accessor<Plan> getPlans() const { return Accessor<Plan>(_plans); }
    const Accessor<PlanType> getPlanTypes() const { return Accessor<PlanType>(_planTypes); }
    const Accessor<Quantifier> getQuantifiers() const { return Accessor<Quantifier>(_quantifiers); }
    const Accessor<RoleSet> getRoleDefinitionSets() const { return Accessor<RoleSet>(_roleSets); }
    const Accessor<Role> getRoles() const { return Accessor<Role>(_roles); }
    const Accessor<State> getStates() const { return Accessor<State>(_states); }
    const Accessor<Synchronisation> getSynchronisations() const { return Accessor<Synchronisation>(_synchronisations); }
    const Accessor<TaskRepository> getTaskRepositories() const { return Accessor<TaskRepository>(_taskRepositories); }
    const Accessor<Task> getTasks() const { return Accessor<Task>(_tasks); }
    const Accessor<Transition> getTransitions() const { return Accessor<Transition>(_transitions); }
    const Accessor<Condition> getConditions() const { return Accessor<Condition>(_conditions); }
    const Accessor<Variable> getVariables() const { return Accessor<Variable>(_variables); }
    const Accessor<ConfAbstractPlanWrapper> getConfAbstractPlanWrappers() const
    {
        return Accessor<ConfAbstractPlanWrapper>(_confAbstractPlanWrapperRepository);
    }
    const Accessor<Configuration> getConfigurations() const { return Accessor<Configuration>(_configurationRepository); }
    const Accessor<TransitionCondition> getTransitionConditions() const { return Accessor<TransitionCondition>(_transitionConditions); }
    const Accessor<TransitionConditionRepository> getTransitionConditionRepositories() const { return Accessor<TransitionConditionRepository>(_transitionConditionRepositories); }

    PlanRepository(const PlanRepository&) = delete;
    PlanRepository(PlanRepository&&) = delete;

    PlanRepository& operator=(const PlanRepository&) = delete;
    PlanRepository& operator=(PlanRepository&&) = delete;

    bool verifyPlanBase() const;

private:
    friend ModelFactory;
    friend Factory;
    friend ModelManager;
    friend ExpressionHandler;
    MapType<Plan> _plans;
    MapType<Task> _tasks;
    MapType<Behaviour> _behaviours;
    MapType<PlanType> _planTypes;
    MapType<Role> _roles;
    MapType<Characteristic> _characteristics;
    MapType<Capability> _capabilities;
    MapType<State> _states;
    MapType<EntryPoint> _entryPoints;
    MapType<Transition> _transitions;
    MapType<Condition> _conditions;
    MapType<Synchronisation> _synchronisations;
    MapType<Quantifier> _quantifiers;
    MapType<Variable> _variables;
    MapType<RoleSet> _roleSets;
    MapType<TaskRepository> _taskRepositories;
    MapType<ConfAbstractPlanWrapper> _confAbstractPlanWrapperRepository;
    MapType<Configuration> _configurationRepository;
    MapType<TransitionCondition> _transitionConditions;
    MapType<TransitionConditionRepository> _transitionConditionRepositories;
};
} // namespace alica
