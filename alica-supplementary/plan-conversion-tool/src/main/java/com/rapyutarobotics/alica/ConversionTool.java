package com.rapyutarobotics.alica;

public class ConversionTool {

    public ConversionTool() {
    }

    public void convert(String[] args) {
        ConversionProcess conversionProcess = new ConversionProcess();
        conversionProcess.setInputDirectories(args[0], args[1], args[2]);
        conversionProcess.setOutputDirectory(args[3]);
        conversionProcess.run(args[4]);
    }

    public static void main(String[] args) {
        if (args.length < 5) {
            System.err.println("Usage: java -jar ConversionTool(..).jar <plans-path> <tasks-path> <roles-path> <output-directory> <path-to-MasterPlan.pml>");
            return;
        }

        ConversionTool conversionTool = new ConversionTool();
        conversionTool.convert(args);
    }
}