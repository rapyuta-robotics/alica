package de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding;

import java.util.ArrayList;
import java.util.List;

/**
 * Class which represents an aspect from a template
 * @author philipp
 *
 */
public class Aspect {
	/**
	 * Name of the aspect
	 */
	private String name;
	
	private List<AspectCode> codes;
	
	public String getName() {
		return name;
	}
	public void setName(String name) {
		this.name = name;
	}
	
	public void addToCodes(AspectCode code){
		getCodes().add(code);
	}
	
	public List<AspectCode> getCodes(){
		if(codes == null){
			codes = new ArrayList<AspectCode>();
		}
		
		return codes;
	}
	
}
