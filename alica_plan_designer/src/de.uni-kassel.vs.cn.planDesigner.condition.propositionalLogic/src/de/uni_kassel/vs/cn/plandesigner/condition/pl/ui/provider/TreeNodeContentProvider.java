package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.TreeNode;
import org.eclipse.jface.viewers.Viewer;

public class TreeNodeContentProvider implements ITreeContentProvider {

	@Override
	public Object[] getChildren(Object parentElement) {
		return ((TreeNode) parentElement).getChildren();
	}

	@Override
	public Object getParent(Object element) {
		return ((TreeNode) element).getParent();
	}

	@Override
	public boolean hasChildren(Object element) {
		return ((TreeNode) element).hasChildren();
	}

	@Override
	public Object[] getElements(Object inputElement) {
		List<TreeNode> temp = (List<TreeNode>) inputElement;
		TreeNode[] nodes = new TreeNode[temp.size()];

		for (int i = 0; i < temp.size(); ++i) {
			TreeNode node = temp.get(i);
			nodes[i] = node;
		}
		return nodes;
	}

	@Override
	public void dispose() {

	}

	@Override
	public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {

	}

}
