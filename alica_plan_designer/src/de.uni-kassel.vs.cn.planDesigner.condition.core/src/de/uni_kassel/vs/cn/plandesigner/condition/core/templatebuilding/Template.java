package de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

/**
 * Class which represents a template for codegeneration
 * 
 * @author philipp
 * 
 */
public class Template {
	private File file;

	private List<Aspect> aspects;

	public File getFile() {
		return file;
	}

	public void setFile(File file) {
		this.file = file;
	}

	public String getName() {
		return file.getName();
	}

	public List<Aspect> getAspects() {
		if (aspects == null) {
			aspects = new ArrayList<Aspect>();
		}

		return aspects;
	}

	public void setAspects(List<Aspect> aspects) {
		this.aspects = aspects;
	}

	public boolean containsAspect(Aspect value) {
		if (value == null) {
			return false;
		}

		for (Aspect aspect : getAspects()) {
			if (aspect.getName().equals(value.getName())) {
				return true;
			}
		}

		return false;
	}

	public Aspect getAspect(String name) {
		for (Aspect aspect : getAspects()) {
			if (aspect.getName().equals(name)) {
				return aspect;
			}
		}
		return null;

	}

	public Template() {

	}

	public boolean persist() {

		try {

			Charset encoding = Charset.forName("ISO-8859-1");
			PrintWriter writer = new PrintWriter(new OutputStreamWriter(new FileOutputStream(file, false), encoding));
			writer.println("«IMPORT alica»");
			for (Aspect aspect : getAspects()) {
				writer.println(aspect.getName());
				for (AspectCode code : aspect.getCodes()) {
					writer.println(code.getCode());
				}
				writer.println("«ENDAROUND»");
			}

			writer.close();
			return true;

		} catch (Exception e) {
			e.printStackTrace();
		}

		return false;
	}

	/**
	 * Creates template from file
	 * 
	 * @param file
	 */
	public Template(File file) {
		if (!file.exists()) {
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		Template temp = parseTemplate(file);
		this.file = file;
		this.aspects = temp.aspects;
	}

	/**
	 * Returns name of all aspects in template file
	 * 
	 * @param mainTemplate
	 * @return
	 */
	private List<Aspect> parseAspects(File mainTemplate) {

		try {
			List<Aspect> aspects = new ArrayList<Aspect>();
			BufferedReader reader = getReaderWithEncoding(mainTemplate);
			String readedLine;

			while ((readedLine = reader.readLine()) != null) {
				if (readedLine.contains("AROUND") && !readedLine.contains("ENDAROUND")) {
					// String expression = getExpressionAfterKeyWord(readedLine,
					// "AROUND");
					String expression = readedLine;

					if (expression != null) {
						Aspect aspect = new Aspect();
						aspect.setName(expression);
						aspects.add(aspect);
					}
				}
			}

			reader.close();

			return aspects;
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	/**
	 * Returns the aspect code from the aspect with the name aspectName from the
	 * given template file.
	 * 
	 * @param aspectName
	 * @param pluginTemplate
	 * @return
	 */
	private AspectCode getAspectCode(String aspectName, File pluginTemplate) {
		try {
			BufferedReader reader = getReaderWithEncoding(pluginTemplate);
			String readedLine;
			StringBuilder codeBuilder = new StringBuilder();
			boolean aspectStarted = false;

			while ((readedLine = reader.readLine()) != null) {
				if (readedLine.contains("ENDAROUND") && aspectStarted) {
					break;
				}

				if (aspectStarted) {
					codeBuilder.append(readedLine + "\n");
				}

				if (readedLine.contains(aspectName)) {
					// now everything till endaround (code above)
					aspectStarted = true;
				}
			}
			reader.close();

			AspectCode code = new AspectCode();
			code.setCode(codeBuilder.toString());
			return code;

		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	/**
	 * Returns a reader with an ecoding specified in the config file.
	 * 
	 * @param file
	 * @return
	 */
	private BufferedReader getReaderWithEncoding(File file) {
		try {
			Charset encoding = Charset.forName("ISO-8859-1");
			return new BufferedReader(new InputStreamReader(new FileInputStream(file), encoding));
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	private Template parseTemplate(File f) {
		List<Aspect> aspects = parseAspects(f);
		
		// fill aspects with code
		for (Aspect aspect : aspects) {
			AspectCode aspectCode = getAspectCode(aspect.getName(), f);
			aspect.addToCodes(aspectCode);
		}

		Template template = new Template();
		template.setAspects(aspects);
		template.setFile(f);

		return template;
	}

}
