package de.unikassel.vs.alica.generator.plugin;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * The {@link PluginManager} holds a list of available Plugins and sets the active plugin for the current session.
 */
public class PluginManager {

    // SINGLETON
    private static volatile PluginManager instance;

    public static PluginManager getInstance() {
        if (instance == null) {
            synchronized (PluginManager.class) {
                if (instance == null) {
                    instance = new PluginManager();
                }
            }
        }

        return instance;
    }

    private static Logger LOG = LogManager.getLogger(PluginManager.class);
    private Map<String,IPlugin<?>> availablePlugins;
    private IPlugin<?> defaultPlugin;

    /**
     * The PluginManager initializes its plugin list at construction.
     * The first available plugin is set as active.
     * The plugin directory is not monitored for changes. That means there is no hot plug functionality here.
     */
    private PluginManager() {
        availablePlugins = new HashMap<>();
    }

    /**
     * Updates the list of available plugins. Must be
     * called from outside at least once for making the PluginManager work.
     */
    public void updateAvailablePlugins(String pluginsFolder) {
        availablePlugins.clear();
        if (pluginsFolder == null || pluginsFolder.isEmpty()) {
            System.out.println("PluginManager: Setting empty plugin folder ignored.");
            return;
        }

        File folder = new File(pluginsFolder);

        if(!folder.exists()) {
            System.err.println("Plugin folder " + folder.getAbsolutePath() + " does not exist");
            return;
        }

        List<File> jars = collectJarsRecursively(folder);

        for (File currentFile : jars) {
            System.out.println("PluginManager: " + currentFile.getName());

            Enumeration<JarEntry> e = null;
            try {
                e = new JarFile(currentFile).entries();
            } catch (IOException e1) {
                e1.printStackTrace();
            }

            // Partially copied from: https://stackoverflow.com/questions/11016092/how-to-load-classes-at-runtime-from-a-folder-or-jar
            while (e != null && e.hasMoreElements()) {
                JarEntry je = e.nextElement();
                if (je.isDirectory() || !je.getName().endsWith(".class")) {
                    continue;
                }
                String className = je.getName().substring(0, je.getName().length() - String.valueOf(".class").length());
                // Replacing both / and \. It seems the returned file names use / as separator even under windows
                // Replacing both to guarantee a valid result (neither / nor \ should appear in class or package names
                // anyways)
                className = className.replaceAll("[/\\\\]", ".");
                try {

                    //URLClassLoader classLoader = (URLClassLoader) ClassLoader.getSystemClassLoader();

                    /**
                     * Adding the jar as source for the Class Loader:
                     * The addURL method is protected, thus we call it through reflection anyway!
                     *
                     * If you know a way to avoid using reflection, go for it.
                     */
                    URL url = new File( currentFile.getAbsolutePath() ).toURI().toURL();
                    URLClassLoader classLoader = new URLClassLoader(new URL[]{url});
                    Class sysClass = URLClassLoader.class;
                    Method method = sysClass.getDeclaredMethod("addURL", URL.class);
                    method.setAccessible(true);
                    method.invoke(classLoader, currentFile.toURI().toURL());

                    // Load the class through the class loader.
                    Class c = classLoader.loadClass(className);
                    Object o = c.getDeclaredConstructor().newInstance();

                    // Only put the class, if it an instance of IPlugin.
                    if (o instanceof IPlugin) {
                        ((IPlugin) o).setPluginFile(currentFile);
                        availablePlugins.put(className, (IPlugin<?>) o);
                    }
                } catch (NoSuchMethodException |
                        MalformedURLException |
                        InvocationTargetException |
                        ClassNotFoundException |
                        IllegalAccessException |
                        InstantiationException exception) {
                    exception.printStackTrace();
                }
            }
        }

        if (!availablePlugins.containsValue(defaultPlugin)) {
            setDefaultPlugin(null);
        }
    }

    private List<File> collectJarsRecursively(File folder) {
        List<File> jars = new ArrayList<>();

        for (String fileName : folder.list()) {
            File currentFile = Paths.get(folder.getAbsolutePath(),fileName).toFile();
            if (currentFile.isDirectory()) {
                jars.addAll(collectJarsRecursively(currentFile));
            } else if (currentFile.isFile() && fileName.endsWith(".jar")) {
                jars.add(currentFile);
            }
        }

        return jars;
    }

    public ObservableList<String> getAvailablePluginNames() {
        ObservableList<String> pluginNamesList = FXCollections.observableArrayList();
        for (IPlugin plugin : availablePlugins.values()) {
            pluginNamesList.add(plugin.getName());
        }
        return pluginNamesList;
    }

    /**
     * Searches through the {@link PluginManager#availablePlugins}.
     *
     * @param name of the wanted plugin
     * @return plugin with matching name otherwise null
     */
    public IPlugin getPlugin(String name) {
        for (IPlugin plugin : availablePlugins.values()) {
            if (plugin.getName().equals(name)) {
                return plugin;
            }
        }

        return null;
    }

    /**
     * Sets the defaultplugin.
     *
     * @param defaultPluginName the plugin that should be active now
     */
    public void setDefaultPlugin(String defaultPluginName) {
        for (IPlugin<?> plugin : availablePlugins.values()) {
            if (plugin.getName().equals(defaultPluginName)) {
                this.defaultPlugin = plugin;
                return;
            }
        }
    }

    /**
     * Getter for the default plugin.
     *
     * @return The default plugin
     */
    public IPlugin<?> getDefaultPlugin() {
        return defaultPlugin;
    }
}
