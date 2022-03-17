package de.unikassel.vs.alica.planDesigner.command;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;
import java.util.Stack;

/**
 * The CommandStack manages the undo and redo stack.
 * Every time the stacks change all observers are informed about the change.
 * You can also ask the CommandStack about the dirty state of a plan
 * which is determined by the save flag of the {@link Command}s on the undo stack
 */
public class CommandStack extends Observable {
    private static final Logger LOG = LogManager.getLogger(CommandStack.class);
    private Stack<Command> undoStack = new Stack<>();
    private Stack<Command> redoStack = new Stack<>();
    private ArrayList<Observer> observers = new ArrayList<>();

    /**
     * Executes a command, puts it on the undo stack and clears the redo stack.
     * @param command
     */
    public void storeAndExecute(Command command) {
        command.doCommand();
//        command.setSaved(false);
        undoStack.push(command);
        redoStack.clear();
        notifyObserversCompound();
    }

    /**
     * executes first command on undo stack and puts it from there to the redo stack.
     */
    public void undo() {
        Command undone = undoStack.pop();
        undone.undoCommand();
//        undone.setSaved(false);
        redoStack.push(undone);
        notifyObserversCompound();
    }

    /**
     * executes first command on redo stack and puts it from there to the undo stack.
     */
    public void redo() {
        Command redone = redoStack.pop();
        redone.doCommand();
//        redone.setSaved(false);
        undoStack.push(redone);
        notifyObserversCompound();
    }

    private void notifyObserversCompound() {
        this.setChanged();
        notifyObservers();
    }

    public void addObserver(Observer observer) {
        observers.add(observer);
    }

    public void removeObserver(Observer observer) {
        observers.remove(observer);
    }

    @Override
    public void notifyObservers() {
        for (Observer observer : observers) {
            observer.update(this, null);
        }
    }

    public boolean isUndoPossible() {
        return undoStack.size() > 0 ;
    }

    public boolean isRedoPossible() {
        return redoStack.size() > 0;
    }


    /**
     *
     * @param plan
     * @return
     */
//    public boolean isAbstractPlanInItsCurrentFormSaved(PlanElement plan) {
//        for (int i = undoStack.indexOf(undoStack.lastElement()); i >= 0; i--) {
//            AbstractCommand currentCommand = undoStack.elementAt(i);
//            if (currentCommand.getAffectedPlan().equals(plan)) {
//                return currentCommand.isSaved();
//            }
//        }
//        return true;
//    }

//    public void setSavedForAbstractPlan(Long abstractPlanId) {
//        Optional<AbstractCommand> abstractCommand = undoStack
//                .stream()
//                .filter(e -> e.getAffectedPlan().getId() == abstractPlanId)
//                .findFirst();
//        if (abstractCommand.isPresent()) {
//            abstractCommand.getPmlUiExtension().setSaved(true);
//        }
//    }

    /**
     *
     * @return saved the saved flag of the top most command on the undo stack.
     */
//    public boolean isCurrentCommandSaved() {
//        if (undoStack.empty()) {
//            return true;
//        } else {
//            return undoStack.peek().isSaved();
//        }
//    }

    public Stack<Command> getUndoStack() {
        return undoStack;
    }

    public Stack<Command> getRedoStack() {
        return redoStack;
    }
}
