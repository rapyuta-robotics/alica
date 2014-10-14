/*******************************************************************************
 * Copyright (c) 2009 itemis AG (http://www.itemis.eu) and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Daniel Weber - Initial implementation 
 * (see https://bugs.eclipse.org/bugs/show_bug.cgi?id=197871)
 * Karsten Thoms - Adaption into Xpand
 *******************************************************************************/

package org.eclipse.xpand.support.cdt;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.eclipse.cdt.core.dom.ast.IASTTranslationUnit;
import org.eclipse.cdt.core.dom.ast.gnu.cpp.GPPLanguage;
import org.eclipse.cdt.core.formatter.DefaultCodeFormatterConstants;
import org.eclipse.cdt.core.formatter.DefaultCodeFormatterOptions;
import org.eclipse.cdt.core.model.ILanguage;
import org.eclipse.cdt.core.parser.CodeReader;
import org.eclipse.cdt.core.parser.ParserUtil;
import org.eclipse.cdt.core.parser.ScannerInfo;
import org.eclipse.cdt.internal.formatter.CodeFormatterVisitor;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.emf.mwe.core.resources.ResourceLoaderFactory;
import org.eclipse.jface.text.Document;
import org.eclipse.jface.text.IDocument;
import org.eclipse.xpand2.output.FileHandle;
import org.eclipse.xpand2.output.JavaBeautifier;
import org.eclipse.xpand2.output.PostProcessor;

/**
 * An Xpand post processor for C/C++ code formatting based on cdt's code
 * formatter.
 * 
 * It uses some internal CDT classes to do its job, but this was the only way to
 * get this to work outside an Eclipse runtime. Use it like this (from an MWE
 * workflow):
 * 
 * <pre>
 *    &lt;postprocessor class=&quot;org.eclipse.xpand.support.cdt.CppBeautifier&quot; configFile=&quot;myCFormatterOptions.xml&quot;/&gt;
 * </pre>
 * 
 * In order to create a configuration file, use CDT's preferences to set
 * formatter options and then export that configuration to a file.
 * 
 * 
 * @author DaWeber@harmanbecker.com
 * @author Karsten Thoms karsten.thoms@itemis.de
 */
@SuppressWarnings("restriction")
public class CppBeautifier implements PostProcessor {
	private static final String DEFAULT_CDT_OPTIONS = "cdtformat-default.xml";
	private String configFile = DEFAULT_CDT_OPTIONS;

	/** Logger instance. */
	private final Log log = LogFactory.getLog(getClass());
	
	private DefaultCodeFormatterOptions formatterOptions;
	
	public CppBeautifier () {
		// Initialize with default settings
		formatterOptions = getCodeFormatterSettings();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @seeorg.openarchitectureware.xpand2.output.PostProcessor#afterClose(org.
	 * openarchitectureware.xpand2.output.FileHandle)
	 */
	public void afterClose(final FileHandle impl) {
	}

	/*
	 * The implementation could have been much easier:
	 * 
	 * <pre> CodeFormatter formatter =
	 * ToolFactory.createDefaultCodeFormatter(getFormatterOptions()); //
	 * CodeFormatter formatter = ToolFactory.createDefaultCodeFormatter(null);
	 * String SOURCE = impl.getBuffer().toString(); IDocument document = new
	 * Document(SOURCE); try { formatter
	 * .format(CodeFormatter.K_COMPILATION_UNIT, SOURCE, 0, SOURCE.length(), 0,
	 * &quot;\n&quot;) .apply(document); impl.setBuffer(new
	 * StringBuffer(document.get())); } catch(MalformedTreeException e) {
	 * log.error(e.getMessage(), e); } catch(BadLocationException e) {
	 * log.error(e.getMessage(), e); } </pre>
	 * 
	 * but unfortunately, this requires the cdt plug-in to be up and running in
	 * an Eclipse runtime, which does not work when running oaw workflows in a
	 * standalone environment.
	 * 
	 * @see
	 * org.openarchitectureware.xpand2.output.PostProcessor#beforeWriteAndClose
	 * (org.openarchitectureware.xpand2.output.FileHandle)
	 */
	public void beforeWriteAndClose(final FileHandle fileHandle) {
		final DefaultCodeFormatterOptions settings = getCodeFormatterSettings();
		String source = fileHandle.getBuffer().toString();
		try {
			CodeFormatterVisitor codeFormatter = new CodeFormatterVisitor(
					settings, 0, source.length());
			IASTTranslationUnit ast = createTranslationUnit(settings.getMap(), source);
			IDocument document = new Document(source);
			codeFormatter.format(source, ast).apply(document);
			fileHandle.setBuffer(document.get());
		} catch (Exception e) {
			log.error(e.getMessage());
			log.debug(e);
		}
	}

	/**
	 * @return the configuration file for the formatter
	 */
	public String getConfigFile() {
		return configFile;
	}

	/**
	 * @param configFile
	 *            configuration file for the formatter
	 */
	public void setConfigFile(final String configFile) {
		this.configFile = configFile;
		this.formatterOptions = null;
		this.formatterOptions = getCodeFormatterSettings();
	}

	/**
	 * @param options
	 * @return
	 */
	private DefaultCodeFormatterOptions getCodeFormatterSettings() {
		if (formatterOptions == null) {
			formatterOptions = DefaultCodeFormatterOptions
					.getDefaultSettings();
			formatterOptions.line_separator = System.getProperty("line.separator");
		
			Properties config = readConfig(getConfigFile());
			if (config != null) {
				Map<String,String> options = new HashMap<String,String>();
				for (Object key : config.keySet()) {
					options.put(key.toString(), config.getProperty(key.toString()));
				}
				formatterOptions.set(options);
			} else {
				log.warn("Using default settings.");
			}
		}
		return formatterOptions;
	}

	/**
	 * @param options
	 * @param source
	 * @return
	 * @throws CoreException
	 */
	private IASTTranslationUnit createTranslationUnit(Map<?, ?> options, String source)
			throws CoreException {
		return getLanguage(options).getASTTranslationUnit(
				new CodeReader(source.toCharArray()), new ScannerInfo(), null,
				null, ParserUtil.getParserLogService());
	}

	/**
	 * Determines the language to be used based on the given options.
	 * 
	 * @param options
	 *            CDT formatter options
	 * @return a suitable ILanguage. GPPLanguage as default.
	 */
	private ILanguage getLanguage(Map<?, ?> options) {
		ILanguage language = (ILanguage) options
				.get(DefaultCodeFormatterConstants.FORMATTER_LANGUAGE);
		if (language == null) {
			language = GPPLanguage.getDefault();
		}
		return language;
	}

	/**
	 * Return a Java Properties instance representing the options that are in the
	 * specified configuration file. In order to use this, simply export a
	 * formatter configuration to a file.
	 * 
	 * This code is mainly copied from {@link JavaBeautifier}, it might make
	 * sense to factor this out to somewhere else so that more formatters can
	 * make use of this.
	 * 
	 * @see JavaBeautifier
	 */
	private Properties readConfig(final String filename) {
		BufferedInputStream stream = null;
		BufferedReader reader = null;

		try {
			InputStream is = getConfig(filename);
			final Properties formatterOptions = new Properties();
			if (filename.endsWith(".xml")) {
				Pattern pattern = Pattern
						.compile("<setting id=\"([^\"]*)\" value=\"([^\"]*)\"\\/>");
				reader = new BufferedReader(new InputStreamReader(is));
				for (String line = reader.readLine(); line != null; line = reader
						.readLine()) {
					Matcher matcher = pattern.matcher(line);
					if (matcher.matches()) {
						formatterOptions
								.put(matcher.group(1), matcher.group(2));
					}
				}
			} else {
				stream = new BufferedInputStream(is);
				formatterOptions.load(stream);
			}
			return formatterOptions;
		} catch (IOException e) {
			log.warn("Problem reading code formatter config file ("
					+ e.getMessage() + ").");
		} finally {
			if (stream != null) {
				try {
					stream.close();
				} catch (IOException e) {
					/* ignore */
				}
			}
			if (reader != null) {
				try {
					reader.close();
				} catch (IOException e) {
					/* ignore */
				}
			}
		}
		return null;
	}

	/**
	 * @param filename
	 *            Path to a formatter configuration file
	 * @return a corresponding File instance if the file can be located
	 * @throws IOException
	 *             If the file could not be found
	 */
	private InputStream getConfig(final String filename) throws IOException {
		URL url = ResourceLoaderFactory.createResourceLoader()
				.getResource(filename);
		if (url == null) {
			url = CppBeautifier.class.getResource(filename);
		}
		if (url == null) {
			throw new IOException("Could not find config file [" + filename
					+ "]");
		}
		InputStream is = url.openStream();
		if (is == null) {
			throw new IOException("Config file [" + filename
					+ "] does not exist.");
		}
		return is;
	}

}
