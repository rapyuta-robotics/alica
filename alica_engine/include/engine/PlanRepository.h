#pragma once

#include <iostream>
#include <unordered_map>

namespace alica
{
class BehaviourConfiguration;
class Behaviour;
class Capability;
class Characteristic;
class EntryPoint;
class Plan;
class PlanType;
class Quantifier;
class RoleDefinitionSet;
class Role;
class State;
class Synchronisation;
class TaskRepository;
class Task;
class Transition;
class Condition;
class Variable;
class PlanningProblem;
class ModelFactory;
class Factory;
class ExpressionHandler;

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

    const Accessor<BehaviourConfiguration> getBehaviourConfigurations() const { return Accessor<BehaviourConfiguration>(_behaviourConfigurations); }
    const Accessor<Behaviour> getBehaviours() const { return Accessor<Behaviour>(_behaviours); }
    const Accessor<Capability> getCapabilities() const { return Accessor<Capability>(_capabilities); }
    const Accessor<Characteristic> getCharacteristics() const { return Accessor<Characteristic>(_characteristics); }
    const Accessor<EntryPoint> getEntryPoints() const { return Accessor<EntryPoint>(_entryPoints); }
    const Accessor<Plan> getPlans() const { return Accessor<Plan>(_plans); }
    const Accessor<PlanType> getPlanTypes() const { return Accessor<PlanType>(_planTypes); }
    const Accessor<Quantifier> getQuantifiers() const { return Accessor<Quantifier>(_quantifiers); }
    const Accessor<RoleDefinitionSet> getRoleDefinitionSets() const { return Accessor<RoleDefinitionSet>(_roleDefinitionSets); }
    const Accessor<Role> getRoles() const { return Accessor<Role>(_roles); }
    const Accessor<State> getStates() const { return Accessor<State>(_states); }
    const Accessor<Synchronisation> getSynchronisations() const { return Accessor<Synchronisation>(_synchronisations); }
    const Accessor<TaskRepository> getTaskRepositorys() const { return Accessor<TaskRepository>(_taskRepositories); }
    const Accessor<Task> getTasks() const { return Accessor<Task>(_tasks); }
    const Accessor<Transition> getTransitions() const { return Accessor<Transition>(_transitions); }
    const Accessor<Condition> getConditoins() const { return Accessor<Condition>(_conditions); }
    const Accessor<Variable> getVariables() const { return Accessor<Variable>(_variables); }
    const Accessor<PlanningProblem> getPlanningProblems() const { return Accessor<PlanningProblem>(_planningProblems); }

    PlanRepository(const PlanRepository&) = delete;
    PlanRepository(PlanRepository&&) = delete;

    PlanRepository& operator=(const PlanRepository&) = delete;
    PlanRepository& operator=(PlanRepository&&) = delete;

    bool verifyPlanBase() const;

private:
    friend ModelFactory;
    friend Factory;
    friend ExpressionHandler;
    MapType<Plan> _plans;
    MapType<Task> _tasks;
    MapType<Behaviour> _behaviours;
    MapType<BehaviourConfiguration> _behaviourConfigurations;
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
    MapType<RoleDefinitionSet> _roleDefinitionSets;
    MapType<TaskRepository> _taskRepositories;
    MapType<PlanningProblem> _planningProblems;
};
} // namespace alica
