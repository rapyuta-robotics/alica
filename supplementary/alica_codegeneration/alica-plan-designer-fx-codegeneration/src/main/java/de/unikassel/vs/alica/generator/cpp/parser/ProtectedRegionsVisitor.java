package de.unikassel.vs.alica.generator.cpp.parser;

import java.util.HashMap;
import java.util.Map;
import de.unikassel.vs.alica.generator.cpp.parser.ProtectedRegionsVisitor;
import de.unikassel.vs.alica.generator.cpp.parser.CommentsBaseVisitor;
import de.unikassel.vs.alica.generator.cpp.parser.CommentsParser.Protected_regionContext;

/**
 * This is an ANTLR generated Visitor. It creates a {@link Map} of protected regions id and the protected code.
 */
public class ProtectedRegionsVisitor extends CommentsBaseVisitor<Void> {

    private Map<String, String> protectedRegions;

    public ProtectedRegionsVisitor() {
        super();
        protectedRegions = new HashMap<>();
    }

    /**
     * It returns a map of protected region ids and their respective code
     * @return
     */
    public Map<String, String> getProtectedRegions() {
        return protectedRegions;
    }

    @Override
    public Void visitProtected_region(Protected_regionContext ctx) {
        protectedRegions.put(ctx.protected_region_header().id.getText(), ctx.content.getText().substring(1));
        return super.visitProtected_region(ctx);
    }
}
