// Copyright 2009 Distributed Systems Group, University of Kassel
// This program is distributed under the GNU Lesser General Public License (LGPL).
//
// This file is part of the Carpe Noctem Software Framework.
//
//    The Carpe Noctem Software Framework is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    The Carpe Noctem Software Framework is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
package de.uni_kassel.vs.cn.planDesigner.ui.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.Path;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Edge;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.exception.PlanNotFoundException;

public class TaskGraphBuilder {

	private Map<Task, TaskWrapper> taskNodeMapping = new HashMap<Task, TaskWrapper>();

	/**
	 * The resource which holds the roleset and from where we can get the master
	 * plan which is associated with that roleset via the 'usableWithPlanID'
	 * attribute.
	 */
	private Resource rolesetResource;

	private Resource graphResource;

	private IFile possibleSavedGraphFile;

	/**
	 * A possible saved Graph.
	 */
	private TaskGraph savedGraph;

	/**
	 * The shared editing domain.
	 */
	private PMLTransactionalEditingDomain domain;

	private RoleSet roleset;

	public TaskGraphBuilder(PMLTransactionalEditingDomain editingDomain,
			Resource rolesetResource) {
		this.domain = editingDomain;
		this.rolesetResource = rolesetResource;
	}

	public TaskGraph createFreshTaskGraph() throws PlanNotFoundException {
		// Get the masterPlan
		Plan masterPlan = RolesetEditorUtils.findPlan(domain, getRoleset()
				.getUsableWithPlanID());

		// clean the mapping
		taskNodeMapping.clear();

		// Create a new graph
		TaskGraph graph = AlicaFactory.eINSTANCE.createTaskGraph();

		// Create a rootNode
		Node root = AlicaFactory.eINSTANCE.createNode();
		// Add the root to the graph
		graph.getNodes().add(root);

		// Connect all children
		List<Node> children = getChildNodes(graph, masterPlan, new ArrayList<Plan>());

		connectChildren(graph, root, children);

		// Check for unnecessary edges
		LinkedList<Edge> unnecessaryEdges = new LinkedList<Edge>();
		LinkedList<Node> open = new LinkedList<Node>();
		LinkedList<Node> closed = new LinkedList<Node>();
		
		open.add(root);
		
		while(!open.isEmpty())
		{
			cleanGraph(open, closed, unnecessaryEdges);
		}
//		
//		for (Edge e : graph.getEdges()) {
//			if (isUnnecessaryEdge(e)) {
//				unnecessaryEdges.add(e);
//			}
//		}

		// Remove all unnecessary edges
		for (Edge e : unnecessaryEdges) {
			e.setFrom(null);
			e.setTo(null);
			graph.getEdges().remove(e);
		}

		return graph;
	}

	private void cleanGraph(LinkedList<Node> open,
			LinkedList<Node> closed, LinkedList<Edge> unnecessaryEdges)
	{
		Node first = open.poll();

		if(first != null && !closed.contains(first)) // There are still nodes to check, get the first
		{
			closed.add(first);
			
			// Expand
			for(Edge e : first.getOutEdge())
			{
				Node to = e.getTo();
				if(!open.contains(to))
				{
					open.add(to);
				}
				
				// If we have a self-cycle or if we already
				// visited the target node, we consider the edge
				// to be unnecessary
				if(first == to || closed.contains(to))
				{
					unnecessaryEdges.add(e);
				}
			}
		}
		
	}

	private void connectChildren(TaskGraph graph, Node parent,
			List<Node> children) {
		// Connect each child via an edge to the parent
		for (Node child : children) {
			// Create an edge
			Edge e = AlicaFactory.eINSTANCE.createEdge();
			// Add it to the graph
			graph.getEdges().add(e);
			// Set in/out
			e.setFrom(parent);
			e.setTo(child);

		}
	}

	/*private boolean isUnnecessaryEdge(Edge e) {
//		Node fromNode = e.getFrom();
//		Node toNode = e.getTo();
//		boolean cycle = false;
//		// Quick check for self cycle
//		if (fromNode.equals(toNode)) {
//			cycle = true;
//		} else {
//			// Check if fromNode can be reached in some
//			// way from toNode
//			List<Node> childNodes = new ArrayList<Node>();
//			retrieveChildNodes(toNode, childNodes);
//
//			cycle = childNodes.contains(fromNode);
//		}
//		return cycle;
		return false;
	}*/

	/*private void retrieveChildNodes(Node n, List<Node> collectedNodes) {
		for (Edge e : n.getOutEdge()) {
			Node toNode = e.getTo();
			if (!collectedNodes.contains(toNode)) {
				collectedNodes.add(toNode);
				retrieveChildNodes(toNode, collectedNodes);
			}
		}
	}*/

	private List<Node> getChildNodes(TaskGraph graph, Plan plan, Collection<Plan> visitedPlans) {
		List<Node> children = new ArrayList<Node>();
		// For every entryPoint...
		for (EntryPoint ep : plan.getEntryPoints()) {
			// Get the task
			Task task = ep.getTask();
			// ...check if there is already a wrapper for that task
			TaskWrapper wrapper = taskNodeMapping.get(task);
			if (wrapper == null) {
				// Nope -> create a new wrapper...
				wrapper = AlicaFactory.eINSTANCE.createTaskWrapper();
				// ...set the task...
				wrapper.setTask(ep.getTask());
				// ...Add the wrapper to the graph...
				graph.getNodes().add(wrapper);
				// ...and put it in the map
				taskNodeMapping.put(task, wrapper);
			}
			children.add(wrapper);

			List<Plan> reachableChildren = getReachableChildPlans(ep);

			for (Plan p : reachableChildren) {
				if(!visitedPlans.contains(p))
				{
					visitedPlans.add(p);
					connectChildren(graph, wrapper, getChildNodes(graph, p, visitedPlans));
				}
			}

		}
		return children;
	}

	private List<Plan> getReachableChildPlans(EntryPoint ep) {
		List<Plan> reachablePlans = new ArrayList<Plan>();

		State state = ep.getState();
		if(state == null) return reachablePlans;
		List<State> reachableStates = new ArrayList<State>();
		List<State> tovisit = new ArrayList<State>();
		tovisit.add(state);
		System.out.println("Begin with "+state.getName());
		while(!tovisit.isEmpty()) {
			State cur = tovisit.remove(0);
			reachableStates.add(cur);
			System.out.println("Next is "+cur.getName());
			for(Transition t : cur.getOutTransitions()) {
				if(!tovisit.contains(t.getOutState()) && !reachableStates.contains(t.getOutState())) {
					tovisit.add(t.getOutState());
				}
			}
		}
		for(State s : reachableStates) {
			for (AbstractPlan absPlan : s.getPlans()) {
				if (absPlan instanceof Plan) {
					reachablePlans.add((Plan) absPlan);
				} else if (absPlan instanceof PlanType) {
					for(AnnotatedPlan p : ((PlanType)absPlan).getPlans()) {
						reachablePlans.add(p.getPlan());
					}
				}
			}
		}
		return reachablePlans;
	}

	/**
	 * Trys to load an then return a savedGraph instance. This is only
	 * successful if:
	 * <ol>
	 * <li>A savedGraph is available, i.e. there is a corresponding .graph file
	 * existing</li>
	 * <li>The contents of this loaded resource is not empty and really is a
	 * taskGraph</li>
	 * </ol>
	 * 
	 * @return The saved TaskGraph instance if available, else NULL.
	 */
	public TaskGraph getSavedGraph() {
		if (savedGraph == null) {
			Resource savedGraphResource = getGraphResource();
			if (savedGraphResource != null
					&& !savedGraphResource.getContents().isEmpty()) {
				Object contents = savedGraphResource.getContents().get(0);
				if (contents instanceof TaskGraph)
					savedGraph = (TaskGraph) contents;
			}
		}
		return savedGraph;
	}

	/**
	 * Compares two graphs with each other and returns a set of deltas. Each
	 * delta describes an operation with the in the delta referenced task. The
	 * type of the delta indicates the following cases:
	 * <ul>
	 * <li><b>GraphDelta.ADD</b>: This means, that a new Taskobject was added
	 * but there was a task with the same name in the savedGraph.</li>
	 * <li><b>GraphDelta.NEW</b>: This means, that a new Taskobject was added
	 * and there was not task with the same name.</li>
	 * <li><b>GraphDelta.REMOVE</b>: This means, that the Taskobject was
	 * removed.</li>
	 * </ul>
	 * 
	 * @param savedGraph
	 * @param freshGraph
	 * @return
	 */
	public static Set<GraphDelta> getGraphDelta(TaskGraph savedGraph,
			TaskGraph freshGraph) {
		Set<GraphDelta> deltaSet = new HashSet<GraphDelta>();

		// Get a Set of all tasks in the savedGraph and freshGraph
		Set<Task> tasksInSaved = extractTasks(savedGraph);
		Set<Task> tasksInFresh = extractTasks(freshGraph);

		// Check for removed tasks
		for (Task t : tasksInSaved) {
			if (!tasksInFresh.contains(t))
				deltaSet.add(new GraphDelta(t, GraphDelta.TYPE_REMOVE));
		}

		// Check for new tasks. A task is considered new if it is not
		// contained in the tasksInSaved set.
		for (Task t : tasksInFresh) {
			if (!tasksInSaved.contains(t)) {
				deltaSet.add(new GraphDelta(t, GraphDelta.TYPE_NEW));
			}

		}

		List<Edge> freshEdges = freshGraph.getEdges();
		List<Edge> savedEdges = savedGraph.getEdges();

		// TODO: This could be more efficient ;)
		// Check for added edges
		for (Edge e : freshEdges) {
			boolean match = false;
			for (Edge e2 : savedEdges) {
				if (equalsEdge(e, e2)) {
					match = true;
					break;
				}
			}
			if (!match) {
				// An edge was added. We don't need any further information
				// about the edge, so we don't have to add it to the delta
				deltaSet.add(new GraphDelta(GraphDelta.TYPE_EDGE));
			}

		}

		// Check for removed edges
		for (Edge e : savedEdges) {
			boolean match = false;
			for (Edge e2 : freshEdges) {
				if (equalsEdge(e, e2)) {
					match = true;
					break;
				}
			}
			if (!match) {
				// An edge was added. We don't need any further information
				// about the edge, so we don't have to add it to the delta
				deltaSet.add(new GraphDelta(GraphDelta.TYPE_EDGE));
			}
		}
		return deltaSet;
	}

	private static Set<Task> extractTasks(TaskGraph graph) {
		Set<Task> extracted = new HashSet<Task>();
		for (Node node : graph.getNodes()) {
			if (node instanceof TaskWrapper)
				extracted.add(((TaskWrapper) node).getTask());
		}
		return extracted;
	}

	/*private void getReachableStates(List<State> reachableStates, State state) {
		for (Transition t : state.getOutTransitions()) {
			State nextState = t.getOutState();
			// If we didn't visit the nextState yet...
			if (!reachableStates.contains(nextState)) {
				// ...we add it to the reachableStates...
				reachableStates.add(nextState);
				// ...and visit all reachable states from the nextState
				getReachableStates(reachableStates, nextState);
			}
		}
	}*/

	private static boolean equalsEdge(Edge e1, Edge e2) {
		Node n1 = (Node) e1.getFrom();
		Node n2 = (Node) e2.getFrom();

		boolean result = true;

		if (n1.eClass() == AlicaPackage.eINSTANCE.getTaskWrapper()
				&& n2.eClass() == AlicaPackage.eINSTANCE.getTaskWrapper())
			result &= ((TaskWrapper) n1).getTask().equals(
					((TaskWrapper) n2).getTask());

		n1 = e1.getTo();
		n2 = e2.getTo();

		result &= ((TaskWrapper) n1).getTask().equals(
				((TaskWrapper) n2).getTask());

		return result;
	}

	private RoleSet getRoleset() {
		if (roleset == null) {
			roleset = (RoleSet) rolesetResource.getContents().get(0);
		}
		return roleset;
	}

	/**
	 * Returns the resource where the TaskGraph should be contained in. This
	 * method either loads an existing .graph file or implicitly creates a new
	 * one by creating a new resource.
	 * 
	 * @return
	 */
	public Resource getGraphResource() {
		if (graphResource == null) {
			// If there is already a .graph file we will load it
			if (isSavedGraphAvailable())
				graphResource = domain.load(getPossibleSavedGraphPath());
			else
				// If not, we will implicitly create it
				graphResource = domain.getResourceSet().createResource(
						URI.createPlatformResourceURI(
								getPossibleSavedGraphPath().getFullPath()
										.toString(), true));
		}
		return graphResource;
	}

	/**
	 * Checks weather a saved graph is available. The check is done in the
	 * following way:
	 * <ol>
	 * <li>The path of a possible saved graph will be determined. That is done
	 * by switching the fileextension of the rolesetResources filename from
	 * .rset to .graph</li>
	 * <li>If that file exists in the same directory as the rolesetResource,
	 * true will be returned, false otherwise</li>
	 * <ol>
	 * 
	 * @return
	 */
	private boolean isSavedGraphAvailable() {
		return getPossibleSavedGraphPath().exists();
	}

	/**
	 * Gets the IFile handle to a possible saved instance of a graph.
	 * 
	 * @return
	 */
	private IFile getPossibleSavedGraphPath() {
		if (possibleSavedGraphFile == null) {
			IFile rolesetFile = WorkspaceSynchronizer
					.getUnderlyingFile(rolesetResource);
			String graphPathString = RolesetEditorUtils.removeFileExtension(
					rolesetFile.getFullPath().toOSString()).concat(".graph");
			IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
			possibleSavedGraphFile = root.getFile(new Path(graphPathString));
		}
		return possibleSavedGraphFile;
	}

	/**
	 * Handles the given delta for the given rolesetresource. Handle means:
	 * <ol>
	 * <li>For each GraphDelta.TYPE_REMOVE remove the assignement.</li>
	 * <li>For each GraphDelta.TYPE_ADD look for the weighting of an existing
	 * assignment and add a new one to each role</li>
	 * <li>For each GraphDelta.TYPE_NEW add a new assignment to each role with a
	 * standard weighting.</li>
	 * </ol>
	 * 
	 * After calling this method, the given roleset is sync with all tasks in
	 * the task hierarchy. That means, that for every role and every task a
	 * taskpriority is available
	 * 
	 * @param rolesetResource
	 * @param delta
	 */
	public static Command handleDeltaForRoleset(
			PMLTransactionalEditingDomain domain, Resource rolesetResource,
			Set<GraphDelta> deltaSet) {
		// Get the roleset from the resource
		RoleSet rSet = (RoleSet) rolesetResource.getContents().get(0);

		CompoundCommand cmp = new CompoundCommand(
				"Adjust roleset due to graph changes");
		for (GraphDelta delta : deltaSet) {
			Task task = delta.getAffectedTask();
			switch (delta.getType()) {
			case GraphDelta.TYPE_ADD:
				// There is at least one task with the same name. Look
				// for a mapping with that task, get the weighting and add
				// a new mapping
				cmp.append(TaskGraphBuilder.handleAddTask(domain, rSet, delta
						.getExistingTaskID(), task));
				break;
			case GraphDelta.TYPE_NEW:
				// There is a new task added which didn't exitst
				// before (didn't exist means that there was a task with
				// the same name before). Just create a new default mapping
				cmp.append(TaskGraphBuilder.handleNewTask(domain, rSet, task));
				break;
			case GraphDelta.TYPE_REMOVE:
				// TODO: Something is wrong with the affected
				// Task, since the reference is null...
				// There was a task removed, so delete all mappings which
				// point to that task
				cmp.append(TaskGraphBuilder
						.handleRemoveTask(domain, rSet, task));
				break;
			}
		}

		return cmp;
	}

	/**
	 * Removes all mappings which point to that task.
	 * 
	 * @param set
	 * @param task
	 */
	private static Command handleRemoveTask(
			PMLTransactionalEditingDomain domain, RoleSet set, final Task task) {
		CompoundCommand removeTaskCmp = new CompoundCommand("Remove Tasks");
		for (final RoleTaskMapping mapping : set.getMappings()) {
			removeTaskCmp.append(new RecordingCommand(domain) {
				@Override
				protected void doExecute() {
					mapping.getTaskPriorities().remove(new Long(task.getId()));
				}
			});
		}
		return removeTaskCmp;
	}

	/**
	 * A new task was added. For each mapping add a taskpriority with a default
	 * of 0.5.
	 * 
	 * @param set
	 * @param task
	 */
	private static Command handleNewTask(PMLTransactionalEditingDomain domain,
			RoleSet set, final Task task) {
		CompoundCommand newTaskCmp = new CompoundCommand("New Tasks");
		for (final RoleTaskMapping mapping : set.getMappings()) {
			newTaskCmp.append(new RecordingCommand(domain) {
				@Override
				protected void doExecute() {
					mapping.getTaskPriorities().put(task.getId(), 0.5d);
				}
			});

		}
		return newTaskCmp;
	}

	/**
	 * A task was added which existed before, i.e. a task with the same name as
	 * another task was added. To keep the graph consistent we have to ensure
	 * that all tasks with the same name also have the same taskpriority in each
	 * mapping.
	 * 
	 * @param set
	 *            The Roleset
	 * @param existingTaskID
	 *            A hint which task in a mapping has the same name as the new
	 *            task
	 * @param task
	 *            The task to add
	 */
	private static Command handleAddTask(PMLTransactionalEditingDomain domain,
			RoleSet set, final long existingTaskID, final Task task) {
		CompoundCommand addTaskCmp = new CompoundCommand("Add Tasks");
		// For each mapping....
		for (final RoleTaskMapping mapping : set.getMappings()) {
			addTaskCmp.append(new RecordingCommand(domain) {
				@Override
				protected void doExecute() {
					// ... look for a task with the same name. Use the hint to
					// quick access the priority
					double foundPriority = mapping.getTaskPriorities().get(
							existingTaskID);
					// Put a new taskpriority for the new task with the
					// foundPriority
					mapping.getTaskPriorities()
							.put(task.getId(), foundPriority);
				}
			});

		}
		return addTaskCmp;
	}

	/**
	 * This method fills the taskWrappers with InternalRoleTaskMapping. That
	 * actually is the transformation from the roleset representation of
	 * mappings (which is more role based) into the taskgraph representation
	 * (which is more task based).
	 * 
	 * @param graphToUse
	 * @param rolesetResource
	 */
	public static Command refreshTaskWrappers(
			PMLTransactionalEditingDomain domain, TaskGraph graphToUse,
			RoleSet roleSet) {
		CompoundCommand cmp = new CompoundCommand("Refresh TaskWrappers");

		// For each TaskGroup...
		for (final Node group : graphToUse.getNodes()) {
			if (group instanceof TaskWrapper) {
				// ...remove all mappings...
				cmp.append(new RecordingCommand(domain) {
					@Override
					protected void doExecute() {
						((TaskWrapper) group).getMappings().clear();
					}
				});
				// ...look at the task which that wrapper wraps. Then take a
				// look
				// into the roleset for each Role that defines a mapping to that
				// task
				// (which should be every role since we generate default
				// mappings) and
				// create a new InternalRoleTaskMapping which is associated with
				// that
				// role and maintains the priority
				Task task = ((TaskWrapper) group).getTask();
				// Create InternalRoleTaskMappings for each role and
				// taskpriority
				for (RoleTaskMapping mapping : roleSet.getMappings()) {

					InternalRoleTaskMapping internal = AlicaFactory.eINSTANCE
							.createInternalRoleTaskMapping();
					internal.setRole(mapping.getRole());
					try {
						internal.setPriority(mapping.getTaskPriorities().get(
								task.getId()));
					} catch (Exception e) {
						System.out.println(e);
					}

					cmp.append(CreateChildCommand.create(domain, group,
							new CommandParameter(group, AlicaPackage.eINSTANCE
									.getTaskWrapper_Mappings(), internal),
							Collections.emptyList()));
				}
			}
		}

		// Execute the command
		return cmp;
	}

}
