package com.rapyutarobotics.alica;

public class ConversionTool {

    public ConversionTool() {
    }

    public void convert(String[] args) {
        ConversionProcess conversionProcess = new ConversionProcess();
        if (args.length == 5) {
            // short version with default sub folders
            conversionProcess.setInputDirectory(args[0]);
            conversionProcess.setOutputDirectories(args[1], args[2]);
            conversionProcess.setPluginsPath(args[3]);
            conversionProcess.run(args[4]);
        } else if (args.length == 7) {
            // long version, specify each folder for plans, tasks, roles
            conversionProcess.setInputDirectories(args[0], args[1], args[2]);
            conversionProcess.setOutputDirectories(args[3], args[4]);
            conversionProcess.setPluginsPath(args[5]);
            conversionProcess.run(args[6]);
        }
    }

    public static void main(String[] args) {
        if (args.length > 7) {
            System.err.println("Usage: java -jar ConversionTool(..).jar <plans-path> <tasks-path> <roles-path> <base-output-directory> <autogen-folder> <plugins-path> (<path-to-MasterPlan.pml>|\"*\")");
            return;
        } else if (args.length < 5 || args.length > 5) {
            System.err.println("Usage: java -jar ConversionTool(..).jar <base-input-directory> <base-output-directory> <autogen-folder> <plugins-path> (<path-to-MasterPlan.pml>|\"*\")");
            return;
        }

        ConversionTool conversionTool = new ConversionTool();
        conversionTool.convert(args);
    }
}