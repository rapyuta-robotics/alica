package de.unikassel.vs.alica.generator;

import de.unikassel.vs.alica.generator.cpp.parser.AlicaConfLexer;
import de.unikassel.vs.alica.generator.cpp.parser.AlicaConfParser;
import de.unikassel.vs.alica.generator.cpp.parser.AlicaConfigurationVisitor;
import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;

import java.io.IOException;

public class CodeGeneratorAlicaConf {

    public String generateAlicaConf(String path) throws IOException {
        CharStream codePointCharStream = CharStreams.fromFileName(path);
        AlicaConfLexer lexer  = new AlicaConfLexer(codePointCharStream);
        CommonTokenStream tokens = new CommonTokenStream(lexer);
        AlicaConfParser parser = new AlicaConfParser(tokens);

        ParseTree tree = parser.all_text();
        AlicaConfigurationVisitor alicaConfigurationVisitor = new AlicaConfigurationVisitor();
        alicaConfigurationVisitor.visit(tree);

        StringBuilder builder = new StringBuilder();
        for (String value : alicaConfigurationVisitor.getCreateYAMLTreeStructure()) {
            builder.append(value);
        }
        String yamlAlicaConf = builder.toString();
        return yamlAlicaConf;
    }
}
