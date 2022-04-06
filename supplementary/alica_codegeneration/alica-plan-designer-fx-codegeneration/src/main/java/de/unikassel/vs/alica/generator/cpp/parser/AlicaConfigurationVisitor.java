package de.unikassel.vs.alica.generator.cpp.parser;

import  de.unikassel.vs.alica.generator.cpp.parser.AlicaConfBaseVisitor;
import java.util.ArrayList;
import java.util.List;

public class AlicaConfigurationVisitor extends AlicaConfBaseVisitor <String>{
    public AlicaConfigurationVisitor() {
        super();
        createYAMLTreeStructure = new ArrayList<>();
    }
    private List<String> createYAMLTreeStructure = new ArrayList<>();
    private int counter = 0;
    public List<String> getCreateYAMLTreeStructure() {
        return createYAMLTreeStructure;
    }

    @Override
    public String visitHeadStart(de.unikassel.vs.alica.generator.cpp.parser.AlicaConfParser.HeadStartContext ctx) {
        visitChildren(ctx);
        createYAMLTreeStructure.add(ctx.getChild(1) + ": {");
        counter++;
        return super.visitHeadStart(ctx);
    }

    @Override
    public String visitHeadEnd(de.unikassel.vs.alica.generator.cpp.parser.AlicaConfParser.HeadEndContext ctx) {
        if(ctx.children != null) {
            if(counter <= 1) {
                createYAMLTreeStructure.add("}");

            } else {
                createYAMLTreeStructure.add("},");
            }
            counter--;
        }
        return super.visitHeadEnd(ctx);
    }

    @Override
    public String visitAll_text(de.unikassel.vs.alica.generator.cpp.parser.AlicaConfParser.All_textContext ctx) {
        return super.visitAll_text(ctx);
    }

    @Override
    public String visitKeyValue(de.unikassel.vs.alica.generator.cpp.parser.AlicaConfParser.KeyValueContext ctx) {
        visitChildren(ctx);
        createYAMLTreeStructure.add(ctx.getChild(0) + ": " + ctx.getChild(2) +  ",");
        return super.visitKeyValue(ctx);
    }
}
