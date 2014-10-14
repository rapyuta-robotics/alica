package de.uni_kassel.vs.cn.planDesigner.codegeneration;

import java.util.Map;

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
import org.eclipse.jface.text.Document;
import org.eclipse.jface.text.IDocument;
import org.eclipse.xpand2.output.FileHandle;
import org.eclipse.xpand2.output.PostProcessor;

public class CNBeautifierCpp implements PostProcessor {
	private static final String DEFAULT_CDT_OPTIONS = "ConfigCpp.xml";
	private String configFile = "";

	/** Logger instance. */
	private final Log log = LogFactory.getLog(getClass());

	private DefaultCodeFormatterOptions formatterOptions;

	public CNBeautifierCpp() {
		 formatterOptions = getCodeFormatterSettings();
		// getConfig(DEFAULT_CDT_OPTIONS);
	}

	public void afterClose(final FileHandle impl) {
	}


	public void beforeWriteAndClose(final FileHandle fileHandle) {
		final DefaultCodeFormatterOptions settings = getCodeFormatterSettings();
		settings.setKandRSettings();

		String source = fileHandle.getBuffer().toString();
		try {
			CodeFormatterVisitor codeFormatter = new CodeFormatterVisitor(
					settings, 0, source.length());
			IASTTranslationUnit ast = createTranslationUnit(settings.getMap(),
					source);
			IDocument document = new Document(source);
			codeFormatter.format(source, ast).apply(document);
			fileHandle.setBuffer(document.get());
		} catch (Exception e) {
			log.error(e.getMessage());
			log.debug(e);
		}
	}


	public String getConfigFile() {
		return configFile;
	}

	public void setConfigFile(final String configFile) {
		this.configFile = configFile;
		this.formatterOptions = null;
		this.formatterOptions = getCodeFormatterSettings();
	}

	private DefaultCodeFormatterOptions getCodeFormatterSettings() {

		formatterOptions = DefaultCodeFormatterOptions.getGNUSettings();
		formatterOptions.line_separator = System.getProperty("line.separator");
		formatterOptions.brace_position_for_namespace_declaration = DefaultCodeFormatterConstants.NEXT_LINE;
		formatterOptions.brace_position_for_block = DefaultCodeFormatterConstants.END_OF_LINE;
		formatterOptions.brace_position_for_type_declaration = DefaultCodeFormatterConstants.END_OF_LINE;
		formatterOptions.brace_position_for_block_in_case = DefaultCodeFormatterConstants.END_OF_LINE;
		formatterOptions.brace_position_for_initializer_list = DefaultCodeFormatterConstants.END_OF_LINE;
		formatterOptions.brace_position_for_method_declaration = DefaultCodeFormatterConstants.END_OF_LINE;
		formatterOptions.brace_position_for_switch = DefaultCodeFormatterConstants.END_OF_LINE;
		formatterOptions.insert_new_line_after_template_declaration = true;
		formatterOptions.indent_body_declarations_compare_to_namespace_header = true;
		formatterOptions.indent_access_specifier_compare_to_type_header = true;
		formatterOptions.indent_body_declarations_compare_to_access_specifier = true;
		formatterOptions.indent_access_specifier_extra_spaces = 0;

		return formatterOptions;
	}

	private IASTTranslationUnit createTranslationUnit(Map<?, ?> options,
			String source) throws CoreException {
		return getLanguage(options).getASTTranslationUnit(
				new CodeReader(source.toCharArray()), new ScannerInfo(), null,
				null, ParserUtil.getParserLogService());
	}

	private ILanguage getLanguage(Map<?, ?> options) {
		ILanguage language = (ILanguage) options
				.get(DefaultCodeFormatterConstants.FORMATTER_LANGUAGE);
		if (language == null) {
			language = GPPLanguage.getDefault();
		}
		return language;
	}



}
